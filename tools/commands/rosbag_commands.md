# ROS2 Rosbag Commands Reference

This document describes how to interact with ROS2 rosbag data (db3 and mcap formats).

## Prerequisites

1. Activate the tidybot2 conda environment:
```bash
mamba activate tidybot2
```

2. Source the ROS2 workspace to access custom messages:
```bash
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Or as a one-liner:
```bash
mamba activate tidybot2 && cd ~/tetheria/aero-open-ros2 && source /opt/ros/humble/setup.bash && source install/setup.bash
```

## Inspecting Rosbag Data

### Get bag info (topics, message counts, duration)
```bash
ros2 bag info /path/to/rosbag_folder
# Example:
ros2 bag info ~/tetheria/aero-open-ros2/rosbag2_2026_01_16-17_09_38
```

### Play back a rosbag
```bash
ros2 bag play /path/to/rosbag_folder
# Play at different rate:
ros2 bag play /path/to/rosbag_folder --rate 0.5  # half speed
```

### Echo specific topics during playback
```bash
# In one terminal:
ros2 bag play /path/to/rosbag_folder

# In another terminal:
ros2 topic echo /right/joint_control
ros2 topic echo /right/actuator_states
ros2 topic echo /spacemouse/cmd_vel
ros2 topic echo /manus_glove_0
```

## Recording Rosbag Data

### Record all topics
```bash
ros2 bag record -a
```

### Record specific topics
```bash
ros2 bag record /spacemouse/cmd_vel /right/joint_control /right/actuator_states /manus_glove_0 /camera_0/color /joint_states
```

### Record to specific output directory
```bash
ros2 bag record -o ~/data/my_recording /topic1 /topic2
```

### Record with mcap format (recommended)
```bash
ros2 bag record -s mcap /topic1 /topic2
```

## Converting db3 to mcap

### Method 1: Using rosbags library (Python)
```bash
# Make sure tidybot2 is activated
mamba activate tidybot2

# Convert single bag
python ~/tetheria/tidyverse-hand/tools/convert_db3_to_mcap.py \
    --input /path/to/rosbag_folder \
    --output /path/to/output.mcap
```

### Method 2: Using ros2 bag convert (requires ros2 bag installed with mcap support)
```bash
# Note: This requires the ros2bag mcap plugin
ros2 bag convert -i /path/to/rosbag_folder -o /path/to/output_folder -s mcap
```

## Converting to LeRobot Format

```bash
# Make sure tidybot2 is activated and ROS2 workspace is sourced
mamba activate tidybot2
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

python ~/tetheria/tidyverse-hand/tools/convert_rosbag_to_lerobot.py \
    --input-dir /path/to/rosbag_folder_or_mcap \
    --output-root ~/data/lerobot_dataset \
    --repo-id "local/my_dataset" \
    --fps 30 \
    --use-video \
    --debug-topics
```

Or use the convenience script:
```bash
~/tetheria/tidyverse-hand/tools/commands/03_convert_to_lerobot.sh \
    /path/to/rosbags_dir \
    /path/to/output_dataset \
    "local/my_dataset"
```

## Available Topics in Typical Recording

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/spacemouse/cmd_vel` | `geometry_msgs/msg/Twist` | SpaceMouse control commands |
| `/manus_glove_0` | `manus_ros2_msgs/msg/ManusGlove` | Human hand pose from Manus glove |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot arm joint states |
| `/right/actuator_states` | `aero_hand_open_msgs/msg/ActuatorStates` | Robot hand actuator feedback |
| `/right/joint_control` | `aero_hand_open_msgs/msg/JointControl` | Robot hand joint control commands |
| `/camera_0/color` | `sensor_msgs/msg/Image` | Camera image |
| `/camera_0/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsics |

## Custom Message Definitions

### JointControl (16 joints)
```
0 - thumb_cmc_abd      8  - middle_pip
1 - thumb_cmc_flex     9  - middle_dip
2 - thumb_mcp          10 - ring_mcp_flex
3 - thumb_ip           11 - ring_pip
4 - index_mcp_flex     12 - ring_dip
5 - index_pip          13 - pinky_mcp_flex
6 - index_dip          14 - pinky_pip
7 - middle_mcp_flex    15 - pinky_dip
```

### ActuatorStates (7 actuators)
```
0 - thumb_cmc_abd_act
1 - thumb_cmc_flex_act
2 - thumb_tendon_act
3 - index_tendon_act
4 - middle_tendon_act
5 - ring_tendon_act
6 - pinky_tendon_act
```
