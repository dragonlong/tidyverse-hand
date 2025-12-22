### Visualize local dataset
```shell
python /Users/xiaoli/projects/code/lerobot/src/lerobot/scripts/lerobot_dataset_viz.py \
  --repo-id evan_house_split_1g_lerobot_v3 \
  --root /Users/xiaoli/projects/code/tidyverse/data/evan_house_split_1g_lerobot_v3 \
  --video-backend pyav \
  --episode-index 0 \
  --num-workers 0

python /Users/xiaoli/projects/code/tidyverse/tools/push_lerobot_dataset_to_hf.py \
  --dataset-root /Users/xiaoli/projects/code/tidyverse/data/evan_house_split_1g_lerobot_v3 \
  --repo-id littledragon/evan_house_split_1g_lerobot_v3 \
  --private 0 \
  --upload-large-folder 1
```