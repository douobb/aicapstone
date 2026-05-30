import math

import isaaclab.sim as sim_utils
import torch

from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from leisaac.utils.general_assets import parse_usd_and_create_subassets
from simulator import ASSETS_ROOT
from simulator.assets.scenes.bathroom import BATHROOM_CFG, BATHROOM_USD_PATH
from simulator.tasks.template.single_arm_franka_cfg import (
    SingleArmFrankaObservationsCfg,
    SingleArmFrankaTaskEnvCfg,
    SingleArmFrankaTaskSceneCfg,
    SingleArmFrankaTerminationsCfg,
)
from simulator.utils.object_poses_loader import ObjectPoseConfig

BATHROOM_OBJECTS_ROOT = ASSETS_ROOT / "scenes" / "bathroom" / "objects"
PUMP_BOTTLE_USD_PATH = str(BATHROOM_OBJECTS_ROOT / "pump_bottle" / "model_pressure_pump_3.usd")

# pump bottle 內已確認存在的 prismatic joint 名稱
PUMP_PRESS_JOINT_NAME = "PrismaticJoint_Pressure_pump_3_up"
# 總行程 0.02 中，先以按下約 75% 當成功門檻
PUMP_PRESS_THRESHOLD = -0.015
# 要求瓶身保持直立，先容許最多 20 度傾斜
BOTTLE_MAX_TILT_DEG = 20.0
BOTTLE_MIN_UP_DOT = math.cos(math.radians(BOTTLE_MAX_TILT_DEG))

PUMP_BOTTLE_INIT_POS = (0.56, -0.38, 0.00)

TAG_TO_OBJECT = {1: "pump_bottle"}
ANCHOR_TAG_ID = 13
ANCHOR_WORLD_POSE = (PUMP_BOTTLE_INIT_POS[0], PUMP_BOTTLE_INIT_POS[1], 0.0)
OBJECT_Z = PUMP_BOTTLE_INIT_POS[2]


def _quat_up_dot_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """計算 local +Z 軸與 world +Z 軸的對齊程度。"""

    x = quat_wxyz[:, 1]
    y = quat_wxyz[:, 2]
    return 1.0 - 2.0 * (x * x + y * y)


def pump_button_pressed(
    env,
    bottle_name: str,
    joint_name: str,
    press_threshold: float,
    min_up_dot: float,
) -> torch.Tensor:
    """成功條件：pump joint 被壓下，且瓶身仍保持直立。"""

    bottle = env.scene[bottle_name]
    joint_ids, _ = bottle.find_joints([joint_name])
    if len(joint_ids) != 1:
        raise ValueError(f"Could not resolve exactly one joint named {joint_name!r}")
    joint_idx = int(joint_ids[0])

    pressed = bottle.data.joint_pos[:, joint_idx] <= press_threshold
    upright = _quat_up_dot_wxyz(bottle.data.root_quat_w) >= min_up_dot
    return torch.logical_and(pressed, upright)


@configclass
class PumpBottlePressSceneCfg(SingleArmFrankaTaskSceneCfg):
    """Pump bottle press 任務的場景設定。"""

    scene: AssetBaseCfg = BATHROOM_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    # 這裡必須使用 ArticulationCfg，因為 bottle 內部包含 prismatic joint
    pump_bottle: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Scene/pump_bottle",
        spawn=sim_utils.UsdFileCfg(
            usd_path=PUMP_BOTTLE_USD_PATH,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            # 先給固定基準位置，真正的每回合差異由 reset randomization 產生
            pos=PUMP_BOTTLE_INIT_POS,
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={},
    )


@configclass
class TerminationsCfg(SingleArmFrankaTerminationsCfg):
    """Pump bottle press 任務的終止條件設定。"""

    success = DoneTerm(
        func=pump_button_pressed,
        params={
            "bottle_name": "pump_bottle",
            "joint_name": PUMP_PRESS_JOINT_NAME,
            "press_threshold": PUMP_PRESS_THRESHOLD,
            "min_up_dot": BOTTLE_MIN_UP_DOT,
        },
    )


@configclass
class PumpBottlePressEnvCfg(SingleArmFrankaTaskEnvCfg):
    """Pump bottle press 任務的環境設定。"""

    scene: PumpBottlePressSceneCfg = PumpBottlePressSceneCfg(env_spacing=8.0)
    observations: SingleArmFrankaObservationsCfg = SingleArmFrankaObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    task_description: str = "press the pump bottle once."

    def __post_init__(self) -> None:
        super().__post_init__()

        self.viewer.eye = (0.8, 0.87, 0.67)
        self.viewer.lookat = (0.4, -1.3, -0.2)
        self.dynamic_reset_gripper_effort_limit = False

        self.scene.robot.init_state.pos = (0.35, -0.74, 0.01)
        self.scene.robot.init_state.rot = (0.707, 0.0, 0.0, 0.707)
        self.scene.robot.init_state.joint_pos = {
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

        self.object_pose_cfg = ObjectPoseConfig(
            tag_to_object=TAG_TO_OBJECT,
            anchor_tag_id=ANCHOR_TAG_ID,
            anchor_world_pose=ANCHOR_WORLD_POSE,
            object_z=OBJECT_Z,
        )

        parse_usd_and_create_subassets(BATHROOM_USD_PATH, self)
