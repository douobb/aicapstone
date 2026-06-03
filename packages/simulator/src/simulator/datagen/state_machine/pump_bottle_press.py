"""Franka 在 pump bottle press 任務中的有限狀態機。"""

from __future__ import annotations

import math

import torch
from isaaclab.utils.math import (
    axis_angle_from_quat,
    matrix_from_quat,
    quat_apply,
    quat_from_euler_xyz,
    quat_inv,
    quat_mul,
)

from leisaac.datagen.state_machine.base import StateMachineBase

# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------
_PUMP_BOTTLE_NAME = "pump_bottle"
_PUMP_PRESS_JOINT_NAME = "PrismaticJoint_Pressure_pump_3_up"
_PUMP_HEAD_BODY_NAME = "E_pump_1"
_EE_BODY_NAME = "panda_hand"
_FRANKA_ARM_JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

_GRIPPER_OPEN = 1.0
_GRIPPER_CLOSE = -1.0
_PRESS_GRIPPER_CMD = _GRIPPER_CLOSE

_MAX_CARTESIAN_DELTA = 0.018
_MAX_ROT_DELTA = 0.08
_IK_DLS_LAMBDA = 0.01

_HOVER_Z_OFFSET = 0.18
_ALIGN_Z_OFFSET = 0.10
_PRESS_START_Z_OFFSET = 0.055
_PRESS_TARGET_Z_OFFSET = 0.015
_RETREAT_Z_OFFSET = 0.20

_PUMP_HEAD_XY_OFFSET = (0.0, 0.0)

_GRIPPER_DOWN_ROLL_W = math.pi
_GRIPPER_DOWN_PITCH_W = 0.0
_GRIPPER_DOWN_YAW_OFFSET_RANGE = (-math.pi, math.pi)

_SUCCESS_PRESS_THRESHOLD = -0.015
_SUCCESS_MAX_TILT_DEG = 20.0
_SUCCESS_MIN_UP_DOT = math.cos(math.radians(_SUCCESS_MAX_TILT_DEG))

_FRANKA_REST_JOINT_POS = {
    "panda_joint1": 0.0,
    "panda_joint2": -math.pi / 4.0,
    "panda_joint3": 0.0,
    "panda_joint4": -3.0 * math.pi / 4.0,
    "panda_joint5": 0.0,
    "panda_joint6": math.pi / 2.0,
    "panda_joint7": math.pi / 4.0,
    "panda_finger_joint1": 0.04,
    "panda_finger_joint2": 0.04,
}

# move_above, align, descend, press, hold, release_up, retreat
_PHASE_DURATIONS = (180, 120, 80, 50, 25, 40, 40)


def _constant_gripper(num_envs: int, device: torch.device, value: float) -> torch.Tensor:
    return torch.full((num_envs, 1), value, device=device)


def _clamp_delta(delta: torch.Tensor, max_norm: float = _MAX_CARTESIAN_DELTA) -> torch.Tensor:
    norm = torch.linalg.norm(delta, dim=-1, keepdim=True).clamp_min(1e-6)
    scale = torch.clamp(max_norm / norm, max=1.0)
    return delta * scale


def _shortest_quat(quat: torch.Tensor) -> torch.Tensor:
    return torch.where(quat[:, 0:1] < 0.0, -quat, quat)


def _yaw_from_quat_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """從 (w, x, y, z) quaternion 取出 world z 軸 yaw。"""

    w, x, y, z = quat_wxyz[:, 0], quat_wxyz[:, 1], quat_wxyz[:, 2], quat_wxyz[:, 3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return torch.atan2(siny_cosp, cosy_cosp)


def _quat_up_dot_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """計算 local +Z 與 world +Z 的對齊程度。"""

    x = quat_wxyz[:, 1]
    y = quat_wxyz[:, 2]
    return 1.0 - 2.0 * (x * x + y * y)


def _find_body_index(robot, body_name: str) -> int:
    if hasattr(robot, "find_bodies"):
        body_ids, _ = robot.find_bodies(body_name)
        if len(body_ids) > 0:
            return int(body_ids[0])

    body_names = getattr(robot.data, "body_names", None)
    if body_names is not None and body_name in body_names:
        return body_names.index(body_name)

    return -1


class PumpBottlePressStateMachine(StateMachineBase):
    """以固定流程控制 Franka 完成一次 pump bottle 按壓。"""

    MAX_STEPS: int = sum(_PHASE_DURATIONS) + 80

    def __init__(self) -> None:
        self._step_count: int = 0
        self._episode_done: bool = False
        self._press_complete: bool = False  # mark press as successful
        self._ee_body_idx: int = -1
        self._jacobi_body_idx: int = -1
        self._pump_head_body_idx: int = -1
        self._arm_joint_ids: list[int] = []
        self._jacobi_joint_ids: list[int] = []
        self._press_joint_idx: int = -1
        self._rest_joint_pos: torch.Tensor | None = None
        self._initial_ee_pos_w: torch.Tensor | None = None
        self._gripper_down_yaw_w: torch.Tensor | None = None
        self._gripper_down_yaw_offset_w: torch.Tensor | None = None
        self._event: int = 0
        self._events_dt: list[int] = list(_PHASE_DURATIONS)

    # ------------------------------------------------------------------
    # StateMachineBase interface
    # ------------------------------------------------------------------

    def setup(self, env) -> None:
        robot = env.scene["robot"]
        bottle = env.scene[_PUMP_BOTTLE_NAME]

        self._ee_body_idx = _find_body_index(robot, _EE_BODY_NAME)
        joint_names = list(robot.data.joint_names)
        missing_joint_names = [
            joint_name for joint_name in _FRANKA_ARM_JOINT_NAMES if joint_name not in joint_names
        ]
        if missing_joint_names:
            raise ValueError(f"Could not find required Franka joints {missing_joint_names} in joints: {joint_names}")
        self._arm_joint_ids = [joint_names.index(joint_name) for joint_name in _FRANKA_ARM_JOINT_NAMES]

        if self._ee_body_idx < 0:
            raise ValueError(f"Could not find required body '{_EE_BODY_NAME}' in Franka bodies.")
        if robot.is_fixed_base:
            self._jacobi_body_idx = self._ee_body_idx - 1
            self._jacobi_joint_ids = self._arm_joint_ids
        else:
            self._jacobi_body_idx = self._ee_body_idx
            self._jacobi_joint_ids = [joint_id + 6 for joint_id in self._arm_joint_ids]

        bottle_joint_ids, _ = bottle.find_joints([_PUMP_PRESS_JOINT_NAME])
        if len(bottle_joint_ids) != 1:
            raise ValueError(f"Could not resolve exactly one joint named {_PUMP_PRESS_JOINT_NAME!r}")
        self._press_joint_idx = int(bottle_joint_ids[0])
        self._pump_head_body_idx = _find_body_index(bottle, _PUMP_HEAD_BODY_NAME)
        if self._pump_head_body_idx < 0:
            raise ValueError(f"Could not find required pump-head body '{_PUMP_HEAD_BODY_NAME}' in bottle bodies.")

        self._rest_joint_pos = torch.zeros(env.num_envs, len(joint_names), device=env.device)
        for idx, name in enumerate(joint_names):
            if name in _FRANKA_REST_JOINT_POS:
                self._rest_joint_pos[:, idx] = _FRANKA_REST_JOINT_POS[name]

        robot.write_joint_state_to_sim(
            position=self._rest_joint_pos,
            velocity=torch.zeros_like(self._rest_joint_pos),
        )
        env.sim.step(render=False)
        env.scene.update(dt=env.physics_dt)

    def check_press_complete(self, env) -> bool:
        bottle = env.scene[_PUMP_BOTTLE_NAME]
        pressed = bottle.data.joint_pos[:, self._press_joint_idx] <= _SUCCESS_PRESS_THRESHOLD
        upright = _quat_up_dot_wxyz(bottle.data.root_quat_w) >= _SUCCESS_MIN_UP_DOT
        return bool(torch.logical_and(pressed, upright).all().item())

    def check_success(self, env) -> bool:
        bottle = env.scene[_PUMP_BOTTLE_NAME]
        sprung_back = bottle.data.joint_pos[:, self._press_joint_idx] >= -0.005  # the joint has sprung back
        return self._press_complete and sprung_back

    def pre_step(self, env) -> None:
        if self._press_complete is False and self.check_press_complete(env):
            self._press_complete = True
        return

    def get_action(self, env) -> torch.Tensor:
        robot = env.scene["robot"]
        bottle = env.scene[_PUMP_BOTTLE_NAME]
        robot.write_joint_damping_to_sim(damping=10.0)

        num_envs = env.num_envs
        device = env.device

        if self._step_count == 0 and self._event == 0:
            self._initial_ee_pos_w = self._ee_pos_w(robot).clone()

        # 以可動的 pump head body 為基準，比用 articulation root 更接近真正的按壓目標。
        pump_head_pos_w = self._pump_head_pos_w(bottle).clone()
        pump_head_target_w = pump_head_pos_w.clone()
        pump_head_target_w[:, 0] += _PUMP_HEAD_XY_OFFSET[0]
        pump_head_target_w[:, 1] += _PUMP_HEAD_XY_OFFSET[1]

        target_quat_w = self._gripper_down_quat_w(
            self._pump_head_quat_w(bottle), num_envs, device, bottle.data.root_quat_w.dtype
        )

        if self._event == 0:
            target_pos_w, gripper_cmd = self._phase_move_above_pump(pump_head_target_w, num_envs, device)
        elif self._event == 1:
            target_pos_w, gripper_cmd = self._phase_align_over_head(pump_head_target_w, num_envs, device)
        elif self._event == 2:
            target_pos_w, gripper_cmd = self._phase_descend_to_press_start(
                pump_head_target_w, num_envs, device
            )
        elif self._event == 3:
            target_pos_w, gripper_cmd = self._phase_press_down(pump_head_target_w, num_envs, device)
        elif self._event == 4:
            target_pos_w, gripper_cmd = self._phase_hold_press(pump_head_target_w, num_envs, device)
        elif self._event == 5:
            target_pos_w, gripper_cmd = self._phase_release_up(pump_head_target_w, num_envs, device)
        else:
            target_pos_w, gripper_cmd = self._phase_retreat(pump_head_target_w, num_envs, device)

        return self._joint_position_franka_action(env, target_pos_w, target_quat_w, gripper_cmd)

    def advance(self) -> None:
        if self._episode_done:
            return

        self._step_count += 1
        if self._step_count < self._events_dt[self._event]:
            # go to next event if pressing is complete
            if not ((self._event == 2 or self._event == 3 or self._event == 4) and self._press_complete):
                return

        self._event += 1
        self._step_count = 0
        if self._event >= len(self._events_dt):
            self._episode_done = True

    def reset(self) -> None:
        self._step_count = 0
        self._episode_done = False
        self._press_complete = False
        self._event = 0
        self._initial_ee_pos_w = None
        self._gripper_down_yaw_w = None
        self._gripper_down_yaw_offset_w = None

    # ------------------------------------------------------------------
    # Phase helpers
    # ------------------------------------------------------------------

    def _phase_move_above_pump(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _HOVER_Z_OFFSET
        if self._initial_ee_pos_w is not None:
            denom = max(self._events_dt[0] - 1, 1)
            alpha = min(self._step_count / denom, 1.0)
            target_pos_w = (1.0 - alpha) * self._initial_ee_pos_w + alpha * target_pos_w
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_align_over_head(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _ALIGN_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_descend_to_press_start(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _PRESS_START_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_press_down(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _PRESS_TARGET_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_hold_press(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _PRESS_TARGET_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_release_up(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _ALIGN_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    def _phase_retreat(self, bottle_target_w, num_envs, device):
        target_pos_w = bottle_target_w.clone()
        target_pos_w[:, 2] += _RETREAT_Z_OFFSET
        return target_pos_w, _constant_gripper(num_envs, device, _PRESS_GRIPPER_CMD)

    # ------------------------------------------------------------------
    # IK / control helpers
    # ------------------------------------------------------------------

    def _ee_pos_w(self, robot) -> torch.Tensor:
        body_idx = self._ee_body_idx if self._ee_body_idx >= 0 else -1
        return robot.data.body_pos_w[:, body_idx, :]

    def _ee_quat_w(self, robot) -> torch.Tensor:
        body_idx = self._ee_body_idx if self._ee_body_idx >= 0 else -1
        return robot.data.body_quat_w[:, body_idx, :]

    def _pump_head_pos_w(self, bottle) -> torch.Tensor:
        if self._pump_head_body_idx < 0:
            raise RuntimeError("PumpBottlePressStateMachine.setup() must run before requesting actions.")
        return bottle.data.body_pos_w[:, self._pump_head_body_idx, :]

    def _pump_head_quat_w(self, bottle) -> torch.Tensor:
        if self._pump_head_body_idx < 0:
            raise RuntimeError("PumpBottlePressStateMachine.setup() must run before requesting actions.")
        return bottle.data.body_quat_w[:, self._pump_head_body_idx, :]

    def _joint_position_franka_action(
        self,
        env,
        target_pos_w: torch.Tensor,
        target_quat_w: torch.Tensor,
        gripper_cmd: torch.Tensor,
    ) -> torch.Tensor:
        robot = env.scene["robot"]
        root_pos_w = robot.data.root_pos_w
        root_quat_w = robot.data.root_quat_w
        root_quat_inv = quat_inv(root_quat_w)

        target_pos_root = quat_apply(root_quat_inv, target_pos_w - root_pos_w)
        ee_pos_root = quat_apply(root_quat_inv, self._ee_pos_w(robot) - root_pos_w)
        delta_pos_root = _clamp_delta(target_pos_root - ee_pos_root)

        delta_quat_w = _shortest_quat(quat_mul(target_quat_w, quat_inv(self._ee_quat_w(robot))))
        delta_rot_w = axis_angle_from_quat(delta_quat_w)
        delta_rot_root = _clamp_delta(quat_apply(root_quat_inv, delta_rot_w), _MAX_ROT_DELTA)

        pose_delta_root = torch.cat([delta_pos_root, delta_rot_root], dim=-1)
        joint_pos_target = self._arm_joint_pos(robot) + self._compute_delta_joint_pos(
            pose_delta_root, self._ee_jacobian_root(robot)
        )
        joint_pos_target = self._clamp_arm_joint_pos(robot, joint_pos_target)
        return torch.cat([joint_pos_target, gripper_cmd], dim=-1)

    def _arm_joint_pos(self, robot) -> torch.Tensor:
        if not self._arm_joint_ids:
            raise RuntimeError("PumpBottlePressStateMachine.setup() must run before requesting actions.")
        return robot.data.joint_pos[:, self._arm_joint_ids]

    def _ee_jacobian_root(self, robot) -> torch.Tensor:
        if self._jacobi_body_idx < 0 or not self._jacobi_joint_ids:
            raise RuntimeError("PumpBottlePressStateMachine.setup() must run before requesting actions.")

        jacobian = robot.root_physx_view.get_jacobians()[
            :, self._jacobi_body_idx, :, self._jacobi_joint_ids
        ].clone()
        root_rot_matrix = matrix_from_quat(quat_inv(robot.data.root_quat_w))
        jacobian[:, :3, :] = torch.bmm(root_rot_matrix, jacobian[:, :3, :])
        jacobian[:, 3:, :] = torch.bmm(root_rot_matrix, jacobian[:, 3:, :])
        return jacobian

    def _compute_delta_joint_pos(self, pose_delta: torch.Tensor, jacobian: torch.Tensor) -> torch.Tensor:
        jacobian_t = torch.transpose(jacobian, dim0=1, dim1=2)
        lambda_matrix = (_IK_DLS_LAMBDA**2) * torch.eye(
            jacobian.shape[1], device=jacobian.device, dtype=jacobian.dtype
        )
        delta_joint_pos = (
            jacobian_t @ torch.inverse(jacobian @ jacobian_t + lambda_matrix) @ pose_delta.unsqueeze(-1)
        )
        return delta_joint_pos.squeeze(-1)

    def _clamp_arm_joint_pos(self, robot, joint_pos: torch.Tensor) -> torch.Tensor:
        joint_pos_limits = getattr(robot.data, "soft_joint_pos_limits", None)
        if joint_pos_limits is None:
            joint_pos_limits = getattr(robot.data, "joint_pos_limits", None)
        if joint_pos_limits is None:
            return joint_pos

        arm_joint_pos_limits = joint_pos_limits[:, self._arm_joint_ids, :]
        return torch.clamp(joint_pos, arm_joint_pos_limits[..., 0], arm_joint_pos_limits[..., 1])

    def _gripper_down_quat_w(
        self,
        bottle_quat_w: torch.Tensor,
        num_envs: int,
        device: torch.device,
        dtype: torch.dtype,
    ) -> torch.Tensor:
        if self._gripper_down_yaw_w is None or self._gripper_down_yaw_w.shape[0] != num_envs:
            base_yaw = _yaw_from_quat_wxyz(bottle_quat_w).to(device=device, dtype=dtype)
            self._gripper_down_yaw_offset_w = torch.empty(num_envs, device=device, dtype=dtype).uniform_(
                _GRIPPER_DOWN_YAW_OFFSET_RANGE[0],
                _GRIPPER_DOWN_YAW_OFFSET_RANGE[1],
            )
            self._gripper_down_yaw_w = (base_yaw + self._gripper_down_yaw_offset_w).clone()

        roll = torch.full((num_envs,), _GRIPPER_DOWN_ROLL_W, device=device, dtype=dtype)
        pitch = torch.full((num_envs,), _GRIPPER_DOWN_PITCH_W, device=device, dtype=dtype)
        yaw = self._gripper_down_yaw_w.to(device=device, dtype=dtype)
        return quat_from_euler_xyz(roll, pitch, yaw)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_episode_done(self) -> bool:
        return self._episode_done

    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def task_object_names(self) -> tuple[str, ...]:
        return (_PUMP_BOTTLE_NAME,)
