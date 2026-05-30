import gymnasium as gym


gym.register(
    id="HCIS-PumpBottlePress-SingleArm-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pump_bottle_press_env_cfg:PumpBottlePressEnvCfg",
    },
)
