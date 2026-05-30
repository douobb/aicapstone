# LeRobot Rollout (Policy Evaluation)

Run a trained policy in the Isaac Lab simulator to evaluate robot performance.

## Prerequisites

1. **Linux machine with Nvidia GPU** — verify with `nvidia-smi`. Isaac Lab requires a Linux host with an Nvidia driver.
2. **Docker installed** — the simulator runs inside a container.
3. **Repository cloned** — if you haven't already:
   ```bash
   git clone https://github.com/HCIS-Lab/aicapstone.git
   cd aicapstone
   ```

## Step 1: Launch Isaac Lab Container

From the repository root, pick the target matching your GlowsAI GPU:

```bash
make launch-isaaclab-glowsai-4090   # RTX 4090
make launch-isaaclab-glowsai-l40s   # L40S
```

This builds and starts the Isaac Lab Docker container. On success, your terminal drops into the container shell.

All remaining steps run **inside the container**.

## Step 2: Download Your Trained Policy

Download the pretrained model from Hugging Face Hub:

```bash
export HF_USER=<your-huggingface-username>
hf download ${HF_USER}/<repo_id> --local-dir <download_path>
```

Replace:
- `<repo_id>` — the policy repository name you used during training (e.g., `my_policy`)
- `<download_path>` — where to save on disk (e.g., `checkpoints/my_policy`)

If you tagged a specific version during upload, add `--revision <tag>` (e.g., `--revision v1`).

For how to upload a trained policy, see [LeRobot Training — After Training](lerobot_training.md#after-training).

## Step 3: Run Rollout

```bash
python scripts/rollout.py \
    --task=<eval_task_file> \
    --policy_type=lerobot-<policy_name> \
    --policy_checkpoint_path=<download_path> \
    --policy_action_horizon=1 \
    --device=cuda \
    --enable_cameras \
    --eval_rounds=<num_rounds> \
    --episode_length_s=20
```

### Flag Reference

| Flag | Description |
|------|-------------|
| `--task` | Path to an evaluation task file under `eval/` (see table below). |
| `--policy_type` | Policy type prefixed with `lerobot-`. For diffusion policy use `lerobot-diffusion`. For ACT use `lerobot-act`. |
| `--policy_checkpoint_path` | Path to the downloaded policy directory (same as `<download_path>` from Step 2). |
| `--policy_action_horizon` | Number of action steps consumed per inference call. |
| `--device=cuda` | Run inference on GPU. |
| `--enable_cameras` | Enable camera rendering for visual observations. |
| `--eval_rounds` | Number of evaluation episodes to roll out. |
| `--episode_length_s` | Episode length in seconds. |

### Available Eval Task Files

| Task | Eval file |
|------|-----------|
| Cup stacking | `eval/cup_stacking_eval.py` |
| Cutlery arrangement | `eval/cutlery_arrangement_eval.py` |
| Toy blocks collection | `eval/toy_blocks_collection_eval` |

### Example

```bash
python scripts/rollout.py \
    --task=eval/cup_stacking_eval.py \
    --policy_type=lerobot-diffusion \
    --policy_checkpoint_path=checkpoints/my_policy \
    --policy_action_horizon=1 \
    --device=cuda \
    --enable_cameras \
    --eval_rounds=50 \
    --episode_length_s=20
```
