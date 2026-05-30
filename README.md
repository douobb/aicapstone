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

已確認的重要物理資訊：

- `prismatic joint`（按壓頭可沿單一軸向滑動）
- `lowerLimit = -0.02`（最大按下行程下限）
- `upperLimit = 0.0`（未按壓時的初始上限位置）
- `stiffness = 80.0`（回彈剛性）
- `damping = 5.0`（回彈阻尼）
- `targetPosition = 0.0`（回彈後的目標位置）

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

另外新增 synthetic pose 工具：

- `scripts/datagen/generate_object_poses.py`
- `scripts/datagen/check_object_poses.py`

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

### 更新檔案

- `packages/simulator/src/simulator/tasks/__init__.py`
  - 匯入並註冊新 task
- `scripts/datagen/generate.py`
  - 把 `PumpBottlePressStateMachine` 接入 `TASK_REGISTRY`

---

## 備註
其他細節與後續待辦請參考：
  - [TODO.md](TODO.md)
