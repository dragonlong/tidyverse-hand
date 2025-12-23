# TidyVerse

**TidyVerse** is a research project on long-horizon household manipulation with dexterous mobile robots.  
The goal is to enable robots with multi-finger hands and a mobile base to perform room-scale tidying tasks—such as arranging shoes and folding clothes—by learning from unstructured human videos.

<p align="center">
  <img src="static/assets/example1.jpg" width="80%">
</p>

## Overview

TidyVerse focuses on three key challenges:
- **Long-horizon tasks** involving multiple objects and sequential subgoals
- **Dexterous manipulation** with multi-finger robot hands
- **Human video supervision** without paired human–robot demonstrations

We introduce **TidyMimic++**, an intent-level imitation framework that distills object-centric task structure from human videos and grounds it in robot execution.

## Tasks

Example tasks in TidyVerse include:
- Shoe arrangement and alignment
- Cloth flattening and folding
- Room-scale cleanup with navigation and manipulation

## Progress & Roadmap

### ✅ Done: pi-0.5 fine-tuning + serving (LIBERO)
- **Logs**: [W&B run group](https://wandb.ai/lxiaol9/openpi?nw=nwuserlxiaol9)

```shell
# (in your openpi checkout)
OPENPI_DIR=/path/to/openpi
cd "$OPENPI_DIR"

# 1) compute mean/std used for normalization
uv run scripts/compute_norm_stats.py --config-name pi05_libero

# 2) launch training
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
  uv run scripts/train.py pi05_libero --exp-name=libero_ft --overwrite

# 3) build a serving image
docker build -t openpi_server -f scripts/docker/serve_policy.Dockerfile .

# 4) run a dev container (adjust volumes as needed)
export OPENPI_DATA_HOME="${OPENPI_DATA_HOME:-$HOME/.cache/openpi}"
mkdir -p "$OPENPI_DATA_HOME"

docker run -d \
  --name openpi_dev_session \
  --gpus all \
  --network host \
  --shm-size=24G \
  -v "$PWD":/app \
  -v "${OPENPI_DATA_HOME}":/openpi_assets \
  -e OPENPI_DATA_HOME=/openpi_assets \
  openpi_server \
  tail -f /dev/null

# 5) run a simple client
docker exec -it openpi_dev_session /bin/bash
uv run python examples/simple_client/main.py --host localhost --port 8000
```

### 🔧 In progress: pi-0.5 SFT with adaptive hand grasping
- **Step 1 (data pipeline)**: rosbag/MCAP → LeRobot dataset (sample: [`littledragon/evan_house_split_1g_lerobot_v3`](https://huggingface.co/datasets/littledragon/evan_house_split_1g_lerobot_v3))

```shell
# TODO: fill in with the exact conversion command for your data source.
# Example placeholders:
# python tools/convert_mcap_to_lerobot.py --input /path/to/*.mcap --output /path/to/dataset_root
# lerobot-dataset-viz --repo-id <name> --root /path/to/dataset_root --episode-index 0 --video-backend pyav
```

### 🧪 Planned: co-training with human egocentric data (VITRA) + EgoDex

### 🧭 Planned: end-to-end spatial grounding + sequential task execution

## License

For research use only.
