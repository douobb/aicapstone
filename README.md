# AI Capstone Course Project

專案基於 [HCIS-Lab/aicapstone](https://github.com/HCIS-Lab/aicapstone) 修改與擴充。

原始專案提供：

- NVIDIA Isaac Sim / Isaac Lab 模擬框架
- UMI demonstration processing pipeline
- LeRobot training / rollout pipeline
- 三個既有任務：
  - `HCIS-CupStacking-SingleArm-v0`
  - `HCIS-CutleryArrangement-SingleArm-v0`
  - `HCIS-ToyBlocksCollection-SingleArm-v0`

我們這次作業的重點是新增一個進階任務：

- `HCIS-PumpBottlePress-SingleArm-v0`

---

## 專案簡介

`pump_bottle_press` 是一個單手 Franka 機械手臂的模擬操作任務。目標是讓機械手臂接近按壓瓶的 pump head，完成一次有效按壓，並保持瓶身直立。

目前的任務設計重點：

- 成功條件：`pressed AND upright`
- 物件：使用帶有 `prismatic joint` 的 `pump_bottle` 資產
- 資料生成方向：
  - 前期以 `teleop smoke test` 驗證環境與互動
  - 後續以 `FSM datagen` 自動產生訓練資料
- datagen 初始化：
  - 使用 synthetic `object_poses.json`

---

## 新增任務細節

### 環境

新增場景物件支援：

- `packages/simulator/src/simulator/assets/scenes/bathroom.py`

新增任務設定：

- `packages/simulator/src/simulator/tasks/pump_bottle_press/__init__.py`
- `packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py`

任務內部重點：

- `pump_bottle` 使用 `ArticulationCfg`
- 讀取內部 `prismatic joint` 狀態判斷是否成功
- 成功條件：
  - 按壓深度達門檻
  - 瓶身維持直立
- 已支援 `object_pose_cfg`
  - 可吃 synthetic `object_poses.json`

### 資產

使用的 bottle 資產位於：

- `packages/simulator/assets/scenes/bathroom/objects/pump_bottle/`

主 USD：

- `packages/simulator/assets/scenes/bathroom/objects/pump_bottle/model_pressure_pump_3.usd`
- `packages/simulator/assets/scenes/bathroom/objects/pump_bottle/model_pressure_pump_3_tuned.usd`

已確認的重要物理資訊：

- `prismatic joint`（按壓頭可沿單一軸向滑動）
- `lowerLimit = -0.02`（最大按下行程下限）
- `upperLimit = 0.0`（未按壓時的初始上限位置）
- `stiffness = 80.0`（回彈剛性）
- `damping = 5.0`（回彈阻尼）
- `maxForce = inf`（joint drive 未設明確推力上限）
- `targetPosition = 0.0`（回彈後的目標位置）
- `mass = 1.0`（瓶身主要剛體質量）
- `dynamicFriction = 0.28`（動摩擦）
- `staticFriction = 0.30`（靜摩擦）
- `restitution = 0.30`（碰撞彈性）

### FSM 與 datagen

新增 FSM：

- `packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py`

FSM 目前包含 phases：

1. move above
2. align
3. descend
4. press
5. hold
6. release up
7. retreat

並已接入：

- `scripts/datagen/generate.py`

另外新增 synthetic pose 與資產調參工具：

- `scripts/datagen/generate_object_poses.py`
- `scripts/datagen/check_object_poses.py`
- `scripts/tune_pump_bottle_usd.py`

新增 evaluation env：

- `eval/pump_bottle_press_eval.py`

---

## 修改的主要檔案與功能
### 新增檔案

- `packages/simulator/src/simulator/assets/scenes/bathroom.py`
  - 定義 bathroom scene 資產入口
- `packages/simulator/src/simulator/tasks/pump_bottle_press/__init__.py`
  - 註冊 `HCIS-PumpBottlePress-SingleArm-v0`
- `packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py`
  - 任務主環境設定、成功條件、object pose 設定
- `packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py`
  - `pump_bottle_press` 的 FSM 資料生成邏輯
- `scripts/datagen/generate_object_poses.py`
  - 產生 synthetic `object_poses.json`
- `scripts/datagen/check_object_poses.py`
  - 驗證 synthetic `object_poses.json` 是否可被 loader 正確解析
- `scripts/tune_pump_bottle_usd.py`
  - 修改 pump bottle USD 內的 joint drive 參數並另存新檔
- `eval/pump_bottle_press_eval.py`
  - rollout / evaluation 專用 env
  - 以固定基準位置加上 `domain_randomization(...)` 做正式評估分布
- `packages/simulator/assets/scenes/bathroom/objects/pump_bottle/model_pressure_pump_3_tuned.usd`
  - 調整過回彈參數的測試用資產

### 更新檔案

- `packages/simulator/src/simulator/tasks/__init__.py`
  - 匯入並註冊新 task
- `scripts/datagen/generate.py`
  - 把 `PumpBottlePressStateMachine` 接入 `TASK_REGISTRY`

---

## 執行步驟

### Step 1. 環境準備

```bash
su - glows
git clone https://github.com/douobb/aicapstone.git
cd aicapstone
make submodules
uv sync
source .venv/bin/activate
hf auth login --token <YOUR_HF_TOKEN>
export HF_USER=<your-huggingface-username>
```

啟動 Isaac Lab container：

```bash
make launch-isaaclab-glowsai-4090
# 或
make launch-isaaclab-glowsai-l40s
```

### Step 2. Simulation Data Generation

先產生 synthetic `object_poses.json`：

```bash
python scripts/datagen/generate_object_poses.py \
    --num_samples 100 \
    --output data/bathroom/object_poses.json
```

再執行 datagen：

```bash
python scripts/datagen/generate.py \
    --task HCIS-PumpBottlePress-SingleArm-v0 \
    --num_envs 1 \
    --device cuda \
    --enable_cameras \
    --record \
    --use_lerobot_recorder \
    --lerobot_dataset_repo_id ${HF_USER}/<generated_dataset_repo> \
    --quality \
    --object_poses data/bathroom/object_poses.json
```

### Step 3. Policy Training

在 host 端訓練：

```bash
wandb login
```

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/<generated_dataset_repo> \
  --policy.type=diffusion \
  --output_dir=output \
  --job_name=pump_bottle_press \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/<policy_repo>
```

### Step 4. Rollout / Evaluation

```bash
python scripts/rollout.py \
    --task=eval/pump_bottle_press_eval.py \
    --policy_type=lerobot-diffusion \
    --policy_checkpoint_path=<path/to/checkpoint> \
    --policy_action_horizon=16 \
    --device=cuda \
    --enable_cameras \
    --rendering_mode=quality \
    --quality \
    --eval_rounds=10 \
    --episode_length_s=30
```
