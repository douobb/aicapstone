"""產生符合現有 loader schema 的 synthetic object_poses.json。"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate synthetic object_poses.json episodes.")
    parser.add_argument(
        "--num_samples",
        type=int,
        required=True,
        help="要產生的 episode 數量。",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("object_poses.synthetic.json"),
        help="輸出的 object_poses.json 路徑。",
    )
    parser.add_argument(
        "--object_name",
        type=str,
        default="pump_bottle",
        help="物件名稱，預設為 pump_bottle。",
    )
    parser.add_argument(
        "--video_name",
        type=str,
        default="converted_60fps_raw_video.mp4",
        help="寫入 JSON 的 video_name 欄位。",
    )
    parser.add_argument(
        "--frame_span",
        type=int,
        default=1,
        help="每個 episode_range 的長度。",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="隨機種子。",
    )
    parser.add_argument(
        "--x_min",
        type=float,
        default=-0.03,
        help="anchor frame 下 tvec[0] 最小值。",
    )
    parser.add_argument(
        "--x_max",
        type=float,
        default=0.03,
        help="anchor frame 下 tvec[0] 最大值。",
    )
    parser.add_argument(
        "--y_min",
        type=float,
        default=-0.03,
        help="anchor frame 下 tvec[1] 最小值。",
    )
    parser.add_argument(
        "--y_max",
        type=float,
        default=0.03,
        help="anchor frame 下 tvec[1] 最大值。",
    )
    parser.add_argument(
        "--yaw_min_deg",
        type=float,
        default=-15.0,
        help="物件 yaw 最小值（度）。",
    )
    parser.add_argument(
        "--yaw_max_deg",
        type=float,
        default=15.0,
        help="物件 yaw 最大值（度）。",
    )
    parser.add_argument(
        "--z_value",
        type=float,
        default=0.0,
        help="寫入 tvec[2] 的值。loader 目前不使用它，但仍保留欄位。",
    )
    return parser


def sample_episode(
    *,
    index: int,
    video_name: str,
    object_name: str,
    frame_span: int,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    yaw_range_deg: tuple[float, float],
    z_value: float,
) -> dict:
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    yaw_rad = math.radians(random.uniform(*yaw_range_deg))

    return {
        "video_name": video_name,
        "episode_range": [index * frame_span, (index + 1) * frame_span],
        "objects": [
            {
                "object_name": object_name,
                # 目前 loader 只會取 yaw，所以用 z 軸 rotation vector 即可。
                "rvec": [0.0, 0.0, yaw_rad],
                "tvec": [x, y, z_value],
            }
        ],
        "status": "full",
    }


def main() -> None:
    parser = build_argparser()
    args = parser.parse_args()

    if args.num_samples <= 0:
        raise ValueError("--num_samples must be > 0")
    if args.frame_span <= 0:
        raise ValueError("--frame_span must be > 0")
    if args.x_min > args.x_max:
        raise ValueError("--x_min must be <= --x_max")
    if args.y_min > args.y_max:
        raise ValueError("--y_min must be <= --y_max")
    if args.yaw_min_deg > args.yaw_max_deg:
        raise ValueError("--yaw_min_deg must be <= --yaw_max_deg")

    random.seed(args.seed)

    episodes = [
        sample_episode(
            index=i,
            video_name=args.video_name,
            object_name=args.object_name,
            frame_span=args.frame_span,
            x_range=(args.x_min, args.x_max),
            y_range=(args.y_min, args.y_max),
            yaw_range_deg=(args.yaw_min_deg, args.yaw_max_deg),
            z_value=args.z_value,
        )
        for i in range(args.num_samples)
    ]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(episodes, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Wrote {len(episodes)} episodes to {args.output}")


if __name__ == "__main__":
    main()
