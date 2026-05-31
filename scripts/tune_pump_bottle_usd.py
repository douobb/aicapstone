"""調整 pump bottle USD 的 joint drive 參數並另存新檔。"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Usd


PUMP_JOINT_PATH = "/root/E_pump_1/PrismaticJoint_Pressure_pump_3_up"


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune pump bottle USD joint drive settings.")
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="原始 USD 路徑。",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="調整後輸出的 USD 路徑。",
    )
    parser.add_argument(
        "--stiffness",
        type=float,
        default=60.0,
        help="新的 stiffness，預設 60.0。",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=10.0,
        help="新的 damping，預設 10.0。",
    )
    parser.add_argument(
        "--target_position",
        type=float,
        default=0.0,
        help="新的 targetPosition，預設 0.0。",
    )
    return parser


def set_attr(prim: Usd.Prim, attr_name: str, value) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        raise ValueError(f"Attribute not found: {attr_name}")
    attr.Set(value)


def main() -> None:
    args = build_argparser().parse_args()
    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()

    if not input_path.exists():
        raise FileNotFoundError(f"Input USD not found: {input_path}")

    stage = Usd.Stage.Open(str(input_path))
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {input_path}")

    prim = stage.GetPrimAtPath(PUMP_JOINT_PATH)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Joint prim not found: {PUMP_JOINT_PATH}")

    set_attr(prim, "drive:linear:physics:stiffness", args.stiffness)
    set_attr(prim, "drive:linear:physics:damping", args.damping)
    set_attr(prim, "drive:linear:physics:targetPosition", args.target_position)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(output_path))

    print(f"Wrote tuned USD to {output_path}")
    print(
        f"Updated {PUMP_JOINT_PATH}: "
        f"stiffness={args.stiffness}, damping={args.damping}, targetPosition={args.target_position}"
    )


if __name__ == "__main__":
    main()
