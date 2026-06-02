# Pump Bottle Press

## 目標

建立新任務 `HCIS-PumpBottlePress-SingleArm-v0`，讓 Franka 完成一次 pump bottle 按壓，後續可用於：

- teleop smoke test
- FSM 自動資料生成
- LeRobot 訓練
- rollout 評估

## 已完成

### 任務與資產

- 已新增 task：
  - `packages/simulator/src/simulator/tasks/pump_bottle_press/__init__.py`
  - `packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py`
- 已新增 scene helper：
  - `packages/simulator/src/simulator/assets/scenes/bathroom.py`
- 已更新：
  - `packages/simulator/src/simulator/tasks/__init__.py`
  - `scripts/datagen/generate.py`
- 已新增 datagen 工具：
  - `packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py`
  - `scripts/datagen/generate_object_poses.py`
- 資產位置：
  - `packages/simulator/assets/scenes/bathroom/objects/pump_bottle/`

### 已確認的 USD 資訊

- prismatic joint：
  - `/root/E_pump_1/PrismaticJoint_Pressure_pump_3_up`
- `body0`：
  - `/root/E_body_2`
- `body1`：
  - `/root/E_pump_1`
- `axis`：
  - `Z`
- `lowerLimit`：
  - `-0.02`
- `upperLimit`：
  - `0.0`
- drive：
  - `stiffness = 80.0`：回彈剛性
  - `damping = 5.0`：回彈阻尼
  - `maxForce = inf`：未設明確推力上限
  - `targetPosition = 0.0`
- mass：
  - `physics:mass = 1.0`：瓶身主要剛體質量
- material：
  - `dynamicFriction = 0.28`
  - `staticFriction = 0.30`
  - `restitution = 0.30`

### Env 設定

- success 已實作為：
  - `pressed AND upright`
- 成功門檻：
  - `PUMP_PRESS_THRESHOLD = -0.015`
- 直立門檻：
  - 傾斜不超過 `20°`
- `pump_bottle` 使用 `ArticulationCfg`
- 已補上 `object_pose_cfg`
- 目前 datagen 走 `object_poses.json` 初始化路線

### FSM 與 datagen

- 已新增 FSM：
  - `packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py`
- FSM 目前設計：
  - success = `pressed AND upright`
  - gripper 預設閉合下壓
  - phases：
    1. move above
    2. align
    3. descend
    4. press
    5. hold
    6. release up
    7. retreat
- 已接入 datagen：
  - `scripts/datagen/generate.py`
- 已新增 synthetic pose 產生腳本：
  - `scripts/datagen/generate_object_poses.py`
- 已新增 `object_poses.json` 檢查腳本：
  - `scripts/datagen/check_object_poses.py`

## 重要待確認

- bottle 的 mass / inertia 尚未明確設定
- `_PUMP_HEAD_XY_OFFSET` 尚未靠實測校正
- FSM 的各段 `Z_OFFSET` 與 phase duration 尚未實測校正
- success threshold 與 tilt threshold 仍需實測確認

## 操作流程

起始位置：

- 已在 Docker container 內
- 與 Entry Level 的 Step 3 相同
- 若使用 GlowsAI L40S，先在主機層執行：

```bash
cd ~/Desktop/aicapstone
make launch-isaaclab-glowsai-l40s
```

容器內工作目錄：

```bash
/workspace/aicapstone
```

### 1. teleop smoke test

```bash
python scripts/teleop.py \
    --task HCIS-PumpBottlePress-SingleArm-v0 \
    --teleop_device keyboard \
    --num_envs 1 \
    --device cuda \
    --enable_cameras
```

#### 在正式錄成dataset時加上以下options:
```bash
    --record \
    --use_lerobot_recorder \
    --lerobot_dataset_repo_id ${HF_USER}/<repo_id> \
    --num_demos 10
```

每個episode的操作流程:

1. 操控機器手臂完成任務
2. 按 `N` 標記任務成功
3. 或按 `R` 來捨棄並重新錄製

設定 `--num_demos 0` :取消episode數量限制 (按 Ctrl+C 停止)。
加上 `--resume` 選項: 將結果併入已存在的dataset。

#### 可調的flags:

| Flag | 用途 |
|------|---------|
| `--sensitivity <float>` | Scale translation + rotation step sizes |
| `--step_hz <int>` | Environment stepping rate (default 60) |
| `--seed <int>` | Deterministic env seed |
| `--quality` | Enable high-quality render mode (FXAA) |

輸入指令 `scripts/teleop.py --help` 以查看所有flags。

### 2. teleop 操作

#### 平移
| Key | Axis |
|-----|------|
| `W` / `S` | +x / -x |
| `A` / `D` | +y / -y |
| `J` / `K` | +z / -z |

#### 旋轉
| Key | Axis |
|-----|------|
| `H` / `L` | roll- / roll+ |
| `U` / `I` | pitch- / pitch+ |
| `Q` / `E` | yaw- / yaw+ |

#### 夾爪
| Key | Action |
|-----|--------|
| `C` | 打開 |
| `M` | 關閉 |

#### 控制
| Key | Action |
|-----|--------|
| `B` | Start to control robot arm |
| `R` | Reset environment / advance to next replay episode |
| `N` | Mark current episode as success (when recording) |

先點 Isaac Sim 視窗，確保鍵盤焦點在 viewport。

### 3. smoke test 要觀察的點

- `pump_bottle` 是否正常出現
- robot 起始姿態是否合理
- bottle 是否保持直立
- pump head 是否回到未按壓狀態
- 是否能穩定接近 pump head
- 是否能成功壓下
- 放手後是否回彈
- bottle 是否太容易翻倒
- success 是否在合理時刻觸發

### 4. 可調參數

- success 門檻
  - `PUMP_PRESS_THRESHOLD`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L27
- 直立門檻
  - `BOTTLE_MAX_TILT_DEG`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L29
- bottle 基準位置
  - `PUMP_BOTTLE_INIT_POS`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L32
- 視角
  - `self.viewer.eye`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L115
  - `self.viewer.lookat`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L116
- robot 起始姿態
  - `self.scene.robot.init_state.pos`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L119
  - `self.scene.robot.init_state.rot`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L120
  - `self.scene.robot.init_state.joint_pos`
  - 位置：`packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py` L121


## datagen 流程
### 生成 object_poses.json

先產生 synthetic `object_poses.json`：

```bash
python scripts/datagen/generate_object_poses.py \
    --num_samples 100 \
    --output object_poses.pump_bottle.json
```

常用參數：

- `--num_samples`
  - 要產生的 episode 數量
- `--output`
  - 輸出的 `object_poses.json` 路徑
- `--seed`
  - 隨機種子
- `--x_min --x_max`
  - `tvec[0]` 範圍
- `--y_min --y_max`
  - `tvec[1]` 範圍
- `--yaw_min_deg --yaw_max_deg`
  - yaw 範圍（度）
- `--object_name`
  - 預設為 `pump_bottle`
- `--video_name`
  - 預設為 `converted_60fps_raw_video.mp4`

先做輕量驗證：

```bash
python scripts/datagen/check_object_poses.py \
    --object_poses object_poses.pump_bottle.json
```

可選：

```bash
python scripts/datagen/check_object_poses.py \
    --object_poses object_poses.pump_bottle.json \
    --show 10
```

用途：

- 驗證 `object_poses.json` schema 是否正確
- 驗證 `object_pose_cfg` 是否能正確轉成 world pose
- 快速檢查前幾筆 `pump_bottle` 的 world `pos / yaw`

### 進行 datagen (先不用跑)
確認無誤後，再執行 datagen：

```bash
python scripts/datagen/generate.py \
    --task HCIS-PumpBottlePress-SingleArm-v0 \
    --num_envs 1 \
    --device cuda \
    --enable_cameras \
    --record \
    --use_lerobot_recorder \
    --lerobot_dataset_repo_id <repo_id> \
    --object_poses <object_poses.json>
```

備註：

- `pump_bottle_press` 已有最小版 `object_pose_cfg`
- `object_poses.json` 目前是 datagen 的主要初始化來源

#### datagen 階段可調參數

- FSM 按壓幾何
  - `_PUMP_HEAD_XY_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L49
  - `_HOVER_Z_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L43
  - `_ALIGN_Z_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L44
  - `_PRESS_START_Z_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L45
  - `_PRESS_TARGET_Z_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L46
  - `_RETREAT_Z_OFFSET`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L47
- FSM 姿態與夾爪
  - `_PRESS_GRIPPER_CMD`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L37
  - `_GRIPPER_DOWN_YAW_OFFSET_RANGE`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L53
- FSM 成功條件
  - `_SUCCESS_PRESS_THRESHOLD`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L55
  - `_SUCCESS_MAX_TILT_DEG`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L56
- FSM 節奏
  - `_PHASE_DURATIONS`
  - 位置：`packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py` L72

## 待辦

### 第一階段：teleop smoke test

- 完成第一輪 teleop smoke test
- 確認 bottle 質量與回彈手感是否合理
- 視實測結果調整：
  - `PUMP_PRESS_THRESHOLD`
  - `BOTTLE_MAX_TILT_DEG`
  - `PUMP_BOTTLE_INIT_POS`
  - viewer 視角
  - robot 初始姿態

### 第二階段：datagen 前置驗證

- 產生 synthetic `object_poses.json`
- 用 `check_object_poses.py` 驗證：
  - schema 是否正確
  - `object_pose_cfg` 是否能轉成合理 world pose
- 視需要調整：
  - synthetic `object_poses` 範圍

### 第三階段：datagen 與 FSM 調整

- 執行第一輪 datagen
- 校正 `_PUMP_HEAD_XY_OFFSET`
- 校正 FSM 的 `Z_OFFSET`
- 視實測結果調整：
  - `_PRESS_GRIPPER_CMD`
  - `_GRIPPER_DOWN_YAW_OFFSET_RANGE`
  - `_PHASE_DURATIONS`
  - `drive` 參數

### 第四階段：訓練與 rollout

- 視需要補 mass / inertia
- 產 dataset 後訓練 baseline policy
- 用 rollout 驗證成功率

## 0531修改

- `pump_bottle_press_env_cfg.py`
  - 保留 `actuators={}`，確認 bottle articulation 可正常運作
  - 恢復 reset randomization，供 teleop smoke test 使用
  - 可調參數：
    - `PUMP_BOTTLE_INIT_POS`
    - `PUMP_BOTTLE_RANDOM_X_RANGE`
    - `PUMP_BOTTLE_RANDOM_Y_RANGE`
    - `PUMP_BOTTLE_RANDOM_YAW_RANGE_DEG`
  - 以上會影響 bottle 初始化的基準位置、平面隨機範圍與朝向隨機範圍
  - 保留 `object_pose_cfg`，供 datagen 使用 `object_poses.json`
- `check_object_poses.py`
  - 改成不再 import env cfg
  - 直接在腳本內定義：
    - `TAG_TO_OBJECT`
    - `ANCHOR_TAG_ID`
    - `ANCHOR_WORLD_POSE`
    - `OBJECT_Z`
  - 目的：避免不必要載入 Isaac Lab / Omniverse 依賴鏈
- `tune_pump_bottle_usd.py`
  - 新增 USD 調參腳本
  - 可直接修改：
    - `stiffness`
    - `damping`
    - `targetPosition`
  - 並另存成新的 tuned USD
- `model_pressure_pump_3_tuned.usd`
  - 已產生 tuned 版資產
  - 目前確認修改成功：
    - `stiffness: 80.0 -> 60.0`
    - `damping: 5.0 -> 10.0`
    - `targetPosition: 0.0`（維持不變）
- `README.md`
  - 補上 tuned 版 USD 工具與額外物理參數說明

目前建議：

- teleop smoke test 可先比較：
  - 原始版 `model_pressure_pump_3.usd`
  - 調整版 `model_pressure_pump_3_tuned.usd`

### `tune_pump_bottle_usd.py` 可指定參數

- `--input`
  - 原始 USD 路徑
- `--output`
  - 調整後輸出的 USD 路徑
- `--stiffness`
  - 回彈剛性
  - 數值越大，回彈越硬、越強
- `--damping`
  - 回彈阻尼
  - 數值越大，回彈越穩、越不容易震盪
- `--target_position`
  - 回彈目標位置
  - 一般維持 `0.0`，代表未按壓時的上限位置

特別注意：

- 之後若要改用調整後的 USD，
  **要記得同步修改**
  `packages/simulator/src/simulator/tasks/pump_bottle_press/pump_bottle_press_env_cfg.py`
  裡的 `PUMP_BOTTLE_USD_PATH`
- 或至少要把檔名改成目前 env cfg 實際引用的正確名稱

### 補充：作業平面範圍

- 依 `bathroom/scene.usd` 中的 `/world/counter_right_main_group/geometry_0` 推估，
  目前主要作業平面約為：
  - `x ≈ 0.0032 ~ 0.7032`
  - `y ≈ -0.6768 ~ -0.0268`
  - 上表面 `z ≈ 0.0409`
- 目前 `pump_bottle` 基準位置 `PUMP_BOTTLE_INIT_POS = (0.56, -0.38, 0.00)` 落在這塊 counter 的合理工作區內。
- 建議第一版安全工作區先縮小為：
  - `PUMP_BOTTLE_RANDOM_X_RANGE = (-0.06, 0.06)`
  - `PUMP_BOTTLE_RANDOM_Y_RANGE = (-0.06, 0.06)`
- 這組範圍可同時用在：
  - `pump_bottle_press_env_cfg.py` 的 reset randomization
  - `scripts/datagen/generate_object_poses.py` 的 `--x_min --x_max --y_min --y_max`
- 若之後在 Isaac Sim 內量到更精確的檯面邊界，再同步更新上述兩處設定。

## 0603修改

- `packages/simulator/src/simulator/datagen/state_machine/pump_bottle_press.py`
  - FSM 的目標基準從 `bottle.data.root_pos_w` 改為可動按壓頭 `E_pump_1`
  - 新增 `_PUMP_HEAD_BODY_NAME = "E_pump_1"`
  - `setup()` 內新增 `self._pump_head_body_idx`
  - 新增 helper：
    - `_pump_head_pos_w()`
    - `_pump_head_quat_w()`
  - 所有 phase 的目標位置改為以 `pump head` 為基準，而不是整顆 bottle 的 articulation root
  - gripper 朝向也改為參考 `pump head` 的 quaternion
- `_PHASE_DURATIONS`
  - 由原本：
    - `(120, 60, 40, 40, 20, 30, 30)`
  - 調整為：
    - `(180, 120, 80, 50, 25, 40, 40)`
  - 目的：
    - 增加 `move_above / align / descend` 的時間
    - 避免還沒接近瓶子就提早進入 `press`

### pump_bottle_press.py 狀態機步驟

1. `move_above`
   - 先移到 `pump head` 上方的安全高度
   - 避免一開始低空橫移撞到瓶子
2. `align`
   - 在較低高度做最後一次對位
   - 目標是把 end-effector 對到按壓頭正上方
3. `descend`
   - 從對位高度下降到按壓起始高度
   - 這一段還不正式下壓，只是接近接觸面
4. `press`
   - 往下壓到目標按壓高度
   - 這是主要的按壓動作
5. `hold`
   - 在按壓位置保持一小段時間
   - 讓 joint 有機會穩定達到 success threshold
6. `release_up`
   - 往上回到較安全的高度
   - 相當於放開按壓頭
7. `retreat`
   - 再抬高並離開瓶子
   - 避免停在物體上方影響下一回合
