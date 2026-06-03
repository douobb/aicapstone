import math

import gymnasium as gym
import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.seed import configure_seed

from leisaac.utils.domain_randomization import domain_randomization, randomize_object_uniform
from leisaac.utils.general_assets import parse_usd_and_create_subassets
from simulator.assets.scenes.bathroom import BATHROOM_CFG, BATHROOM_USD_PATH
from simulator.tasks.template.single_arm_franka_cfg import (
    SingleArmFrankaObservationsCfg,
    SingleArmFrankaTaskEnvCfg,
    SingleArmFrankaTaskSceneCfg,
    SingleArmFrankaTerminationsCfg,
)

from simulator.tasks.pump_bottle_press.pump_bottle_press_env_cfg import (
    BOTTLE_MIN_UP_DOT,
    PUMP_BOTTLE_INIT_POS,
    PUMP_BOTTLE_NAME,
    PUMP_BOTTLE_USD_PATH,
    PUMP_PRESS_JOINT_NAME,
    PUMP_PRESS_THRESHOLD,
)


configure_seed(42)


def _quat_up_dot_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """計算 local +Z 軸與 world +Z 軸的對齊程度。"""

    x = quat_wxyz[:, 1]
    y = quat_wxyz[:, 2]
    return 1.0 - 2.0 * (x * x + y * y)


def pump_button_pressed(
    env,
    joint_name: str,
    press_threshold: float,
    min_up_dot: float,
) -> torch.Tensor:
    """成功條件：pump joint 被壓下，且瓶身仍保持直立。"""

    bottle: Articulation = env.scene[PUMP_BOTTLE_NAME]
    joint_ids, _ = bottle.find_joints([joint_name])
    if len(joint_ids) != 1:
        raise ValueError(f"Could not resolve exactly one joint named {joint_name!r}")
    joint_idx = int(joint_ids[0])

    pressed = bottle.data.joint_pos[:, joint_idx] <= press_threshold
    upright = _quat_up_dot_wxyz(bottle.data.root_quat_w) >= min_up_dot
    return torch.logical_and(pressed, upright)


@configclass
class PumpBottlePressEvalSceneCfg(SingleArmFrankaTaskSceneCfg):
    """Pump bottle press rollout/eval 專用場景設定。"""

    scene: AssetBaseCfg = BATHROOM_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    pump_bottle: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Scene/pump_bottle",
        spawn=sim_utils.UsdFileCfg(
            usd_path=PUMP_BOTTLE_USD_PATH,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=PUMP_BOTTLE_INIT_POS,
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={},
    )


@configclass
class EvalTerminationsCfg(SingleArmFrankaTerminationsCfg):
    """Pump bottle press rollout/eval 專用終止條件。"""

    success = DoneTerm(
        func=pump_button_pressed,
        params={
            "joint_name": PUMP_PRESS_JOINT_NAME,
            "press_threshold": PUMP_PRESS_THRESHOLD,
            "min_up_dot": BOTTLE_MIN_UP_DOT,
        },
    )


@configclass
class PumpBottlePressEvalEnvCfg(SingleArmFrankaTaskEnvCfg):
    """Pump bottle press rollout/eval 專用環境設定。"""

    scene: PumpBottlePressEvalSceneCfg = PumpBottlePressEvalSceneCfg(env_spacing=8.0)
    observations: SingleArmFrankaObservationsCfg = SingleArmFrankaObservationsCfg()
    terminations: EvalTerminationsCfg = EvalTerminationsCfg()
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

        parse_usd_and_create_subassets(BATHROOM_USD_PATH, self)

        domain_randomization(
            self,
            random_options=[
                randomize_object_uniform(
                    PUMP_BOTTLE_NAME,
                    pose_range={
                        "x": (-0.25, 0.25),
                        "y": (-0.25, 0.25),
                        "z": (0.0, 0.0),
                        "yaw": (-math.pi, math.pi),
                    },
                ),
            ],
        )


TASK_ID = "Private-PumpBottlePress-Eval-v0"

gym.register(
    id=TASK_ID,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}:PumpBottlePressEvalEnvCfg"},
)
