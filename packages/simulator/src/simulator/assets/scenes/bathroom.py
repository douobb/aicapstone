from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from simulator import ASSETS_ROOT

"""Configuration for the Bathroom Scene."""
SCENES_ROOT = Path(ASSETS_ROOT) / "scenes"

BATHROOM_USD_PATH = str(SCENES_ROOT / "bathroom" / "scene.usd")

BATHROOM_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=BATHROOM_USD_PATH,
    )
)
