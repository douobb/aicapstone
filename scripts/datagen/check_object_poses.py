"""檢查 synthetic object_poses.json 是否可被 loader 正確解析。"""

from __future__ import annotations

import argparse
import math

from simulator.tasks.pump_bottle_press.pump_bottle_press_env_cfg import (
    ANCHOR_TAG_ID,
    ANCHOR_WORLD_POSE,
    OBJECT_Z,
    TAG_TO_OBJECT,
)
from simulator.utils.object_poses_loader import ObjectPoseConfig, load_episode_poses


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate synthetic object_poses.json for pump_bottle_press.")
    parser.add_argument(
        "--object_poses",
        type=str,
        required=True,
        help="要檢查的 object_poses.json 路徑。",
    )
    parser.add_argument(
        "--show",
        type=int,
        default=5,
        help="顯示前幾筆 episode 的 world pose。",
    )
    return parser


def yaw_from_quat_wxyz(quat: tuple[float, float, float, float]) -> float:
    w, x, y, z = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main() -> None:
    args = build_argparser().parse_args()

    cfg = ObjectPoseConfig(
        tag_to_object=TAG_TO_OBJECT,
        anchor_tag_id=ANCHOR_TAG_ID,
        anchor_world_pose=ANCHOR_WORLD_POSE,
        object_z=OBJECT_Z,
    )

    episodes = load_episode_poses(args.object_poses, cfg)
    print(f"Loaded {len(episodes)} episode(s) from {args.object_poses}")

    show_count = min(args.show, len(episodes))
    for idx in range(show_count):
        pose = episodes[idx]["pump_bottle"]
        pos, quat = pose
        yaw_deg = yaw_from_quat_wxyz(quat)
        print(
            f"[episode {idx:03d}] "
            f"pos=({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}) "
            f"yaw={yaw_deg:+.2f} deg"
        )


if __name__ == "__main__":
    main()
