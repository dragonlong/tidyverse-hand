# Dragonbot Control Architecture

**Robot:** TetherIA Aero Hand (7-DOF tendon-driven, 16 finger joints) + YAM Arm (6-DOF)
**ROS workspace:** `aero-open-ros2`

---

## Converged Teleoperation Mode

The finalised control setup uses three input devices — one per subsystem:

| Subsystem | Input Device | Notes |
|---|---|---|
| **Hand** (16 DOF) | Meta Quest 3 | Hand landmark retargeting, Wi-Fi/LAN UDP |
| **Arm** (6 DOF) | Gello replica arm | Dynamixel joint mirroring, USB-serial |
| **Base** (x, y, θ) | 3Dconnexion SpaceMouse | USB HID, 100 Hz Twist |

```
LOCAL PC                                    MOBILE PC (with hardware)
─────────────────────────────               ─────────────────────────────────────
 [Meta Quest 3]  ──UDP──►  04_quest3_hand   /right/joint_control ──►  05_aero_hand
                              (bridge +                                  │
                              retargeting)                               │ Serial 921600
                                                                        ▼
                                                                   [Aero Hand]

 [Gello Arm]  ──USB──►  02_gello_arm        /joint_states        ──►  03_yam_arm
                          (publisher)                                    │
                                                                        │ CAN
                                                                        ▼
                                                                   [YAM Arm]

 [SpaceMouse]  ──USB──►  01_spacemouse
                          (Twist 100Hz)     spacemouse/cmd_vel ──►  [Mobile Base]
```

---

## Node Graph

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  LOCAL PC                                                                    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  HAND PIPELINE                                                      │    │
│  │                                                                     │    │
│  │  [Quest 3 VR headset]                                               │    │
│  │       │ UDP (21-point hand landmarks + wrist pose, ~70 Hz)          │    │
│  │       ▼                                                             │    │
│  │  hand_tracking_bridge          (hand-tracking-sdk-ros2)            │    │
│  │       │                                                             │    │
│  │       ├─► /hands/right/landmarks  [PoseArray, 21 pts]              │    │
│  │       ├─► /hands/left/landmarks   [PoseArray, 21 pts]              │    │
│  │       └─► /hands/right/wrist_pose [PoseStamped, 6-DoF]            │    │
│  │                   │                                                 │    │
│  │                   ▼                                                 │    │
│  │  quest3_retargeting            (aero_hand_open_teleop)             │    │
│  │       │  21-pt landmarks → 16-DOF angles                           │    │
│  │       │  EMA smoothing α=0.7                                        │    │
│  │       │                                                             │    │
│  │       └─► /right/joint_control  [JointControl, 16-DOF]  ──────────┼──► │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  ARM PIPELINE                                                       │    │
│  │                                                                     │    │
│  │  [Gello Arm — 6-DOF Dynamixel]                                      │    │
│  │       │ USB-serial                                                  │    │
│  │       ▼                                                             │    │
│  │  gello_publisher               (gello_controller)                  │    │
│  │       │  Joint offsets + signs applied                              │    │
│  │       │  Optional footswitch enable/disable                         │    │
│  │       │  30 Hz                                                      │    │
│  │       │                                                             │    │
│  │       └─► /joint_states  [JointState, 6-DOF]  ─────────────────────┼──► │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  BASE PIPELINE                                                      │    │
│  │                                                                     │    │
│  │  [3Dconnexion SpaceMouse]                                           │    │
│  │       │ USB HID                                                     │    │
│  │       ▼                                                             │    │
│  │  spacemouse_twist_publisher    (aero_hand_open_teleop)             │    │
│  │       │  Deadzone + low-pass filter                                 │    │
│  │       │  100 Hz                                                     │    │
│  │       │                                                             │    │
│  │       └─► /spacemouse/cmd_vel  [Twist]  ────────────────────────────┼──► │
│  └─────────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────────┘
                    │ ROS 2 DDS (shared ROS_DOMAIN_ID over LAN)
                    ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  MOBILE PC (hardware attached)                                               │
│                                                                              │
│  /right/joint_control ──►  aero_hand_node  (aero_hand_open)                │
│                                 │  Serial UART @ 921600 baud                │
│                                 ├─► [Aero Hand Right]  16-DOF tendon hand   │
│                                 └─► /right/actuator_states  ◄ feedback 100Hz│
│                                                                              │
│  /joint_states  ──►  yam_gello_controller  (gello_controller)              │
│                                 │  CAN bus (can0)                           │
│                                 └─► [YAM Arm]  6-DOF                        │
│                                                                              │
│  /spacemouse/cmd_vel  ──►  [Mobile Base Controller]                         │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Topic Reference

| Topic | Type | Publisher | Subscriber | Rate |
|---|---|---|---|---|
| `/hands/right/landmarks` | `geometry_msgs/PoseArray` | `hand_tracking_bridge` | `quest3_retargeting` | ~70 Hz |
| `/hands/right/wrist_pose` | `geometry_msgs/PoseStamped` | `hand_tracking_bridge` | *(experimental arm)* | ~70 Hz |
| `/right/joint_control` | `aero_hand_open_msgs/JointControl` | `quest3_retargeting` | `aero_hand_node` | ~30 Hz |
| `/right/actuator_states` | `aero_hand_open_msgs/ActuatorStates` | `aero_hand_node` | monitoring | 100 Hz |
| `/joint_states` | `sensor_msgs/JointState` | `gello_publisher` | `yam_gello_controller` | 30 Hz |
| `/spacemouse/cmd_vel` | `geometry_msgs/Twist` | `spacemouse_twist_publisher` | base controller | 100 Hz |

---

## Launch Instructions

### Prerequisites

```bash
# Build the ROS workspace (once after cloning / adding packages)
cd /home/dragonx/tetheria/aero-open-ros2
colcon build --symlink-install
source install/setup.bash

# Python deps (if not already installed)
pip install pyspacemouse dynamixel-sdk evdev pyudev
```

### Local PC — teleop publishers

```bash
bash ~/tetheria/tidyverse-hand/scripts/teleop/launch_all.sh
```

Opens a tmux session with 3 panes:

| Pane | Script | Publishes |
|---|---|---|
| 0 | `teleop/01_spacemouse.sh` | `/spacemouse/cmd_vel` (100 Hz) |
| 1 | `teleop/02_gello_arm.sh` | `/joint_states` (30 Hz) |
| 2 | `teleop/03_quest3_hand.sh` | `/right/joint_control` (~30 Hz) |

### Edge Device — hardware control nodes

```bash
# Set Aero Hand serial port
export RIGHT_PORT=/dev/serial/by-id/usb-FTDI_USB_Serial_Cable_XXXXXXXX-if00-port0

bash ~/tetheria/tidyverse-hand/scripts/hardware/launch_all.sh
```

Opens a tmux session with 2 panes:

| Pane | Script | Subscribes → Hardware |
|---|---|---|
| 0 | `hardware/01_yam_arm.sh` | `/joint_states` → YAM arm (CAN) |
| 1 | `hardware/02_aero_hand.sh` | `/right/joint_control` → Aero Hand (serial) |

> **Note:** Both machines must share the same `ROS_DOMAIN_ID` over LAN.

### Verify everything is running

```bash
bash ~/tetheria/tidyverse-hand/scripts/utils/verify_topics.sh
```

### Stop all

```bash
bash ~/tetheria/tidyverse-hand/scripts/utils/kill_teleop.sh
```

---

## Simulation / Debug (no hardware)

Two MuJoCo simulators are available, both run FK-only (no physics step):

### Hand-only viewer

Visualises `/right/joint_control` with the Aero Hand floating in space —
fast to launch, useful for checking retargeting quality.

```bash
# Terminal 1 — Quest 3 bridge + retargeting
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# Terminal 2 — MuJoCo Aero Hand only
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_mujoco_hand.sh
```

### Combined YAM arm + hand viewer

Visualises `/joint_states` (arm) **and** `/right/joint_control` (hand) in one
window.  The Aero Hand is attached to `link_6` of the YAM arm — no mobile base.

```bash
# Terminal 1 — Gello arm publisher
bash ~/tetheria/tidyverse-hand/scripts/teleop/02_gello_arm.sh

# Terminal 2 — Quest 3 hand retargeting
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# Terminal 3 — combined MuJoCo viewer
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_yam_with_hand.sh
```

Scene XML: `models/yam_with_hand/scene_yam_with_hand.xml`
ROS node : `aero_hand_open_teleop mujoco_yam_hand_viewer`

### Full Dragonbot viewer (base + arm + hand)

The complete robot: TidyBot mobile base + YAM arm + Aero Hand.  Base velocity
from SpaceMouse is integrated live into `(joint_x, joint_y, joint_th)`.

```bash
# Terminal 1 — SpaceMouse base velocity
bash ~/tetheria/tidyverse-hand/scripts/teleop/01_spacemouse.sh

# Terminal 2 — Gello arm
bash ~/tetheria/tidyverse-hand/scripts/teleop/02_gello_arm.sh

# Terminal 3 — Quest 3 hand
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# Terminal 4 — full Dragonbot MuJoCo viewer
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_dragonbot.sh
```

Scene XML: `models/dragonbot/scene.xml` (includes `dragonbot.xml`)
ROS node : `aero_hand_open_teleop mujoco_dragonbot_viewer`

For the full staged bring-up procedure (simulator → hardware), see
[`docs/teleop_testing_guide.md`](teleop_testing_guide.md).

---

## Experimental: Quest 3 Wrist → YAM Arm

Uses Quest 3 wrist pose (instead of Gello) for arm control via inverse kinematics.
**Not validated for data collection** — workspace limits and IK stability are still being tested.

```
[Quest 3 wrist pose]
    │  /hands/right/wrist_pose  [PoseStamped]
    ▼
quest3_yam_teleop  (yam_teleop)
    │  pyroki IK solver
    │  EMA smoothing, workspace clamping
    ▼
/joint_states  →  yam_gello_controller  →  YAM Arm
```

```bash
# Replaces teleop/02_gello_arm.sh — do NOT run both simultaneously
# Still requires hardware/01_yam_arm.sh on the mobile PC
bash ~/tetheria/tidyverse-hand/scripts/teleop/experimental_quest3_arm.sh
```

---

## GR00T Policy Server (Autonomous Mode)

Replaces the human operator entirely. A trained GR00T N1.6 model runs on a GPU
machine and sends actions to the robot over ZeroMQ.

```
┌──────────────────────────────────────────────────────────────────┐
│  GPU Machine                                                     │
│                                                                  │
│  GR00T Policy Server  (run_gr00t_server.py)                     │
│  ├─ Input:  cameras (head, wrist, base) + arm/hand state        │
│  └─ Output: arm_joint_targets (16-step) + hand_joint_targets    │
│                                                                  │
│  ZeroMQ TCP  tcp://0.0.0.0:5555                                 │
└──────────────────────────────┬───────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│  Robot Client (user-written, runs on mobile PC)                  │
│                                                                  │
│  Captures:  3 cameras + /right/actuator_states + /joint_states  │
│  Publishes: /joint_states → YAM Arm                             │
│             /right/joint_control → Aero Hand                    │
└──────────────────────────────────────────────────────────────────┘
```

**Launch policy server:**
```bash
cd /path/to/Isaac-GR00T
source .venv/bin/activate

python gr00t/eval/run_gr00t_server.py \
  --model-path /path/to/checkpoint \
  --modality-config-path examples/tidyverse-hand/dragonbot_config.py \
  --embodiment-tag NEW_EMBODIMENT \
  --device cuda:0 \
  --host 0.0.0.0 \
  --port 5555
```

See `third_party/Isaac-GR00T/dragonbot_finetune.md` for full finetuning and deployment instructions.

---

## Scripts Reference

All scripts live in `~/tetheria/tidyverse-hand/scripts/`.

```
scripts/
  teleop/                        ← run on LOCAL PC (input publishers)
    01_spacemouse.sh               SpaceMouse → /spacemouse/cmd_vel
    02_gello_arm.sh                Gello → /joint_states
    03_quest3_hand.sh              Quest 3 → /right/joint_control
    launch_all.sh                  tmux launcher (all 3 above)
    experimental_quest3_arm.sh     Quest 3 wrist → YAM arm IK (experimental)

  hardware/                      ← run on EDGE DEVICE (hardware control)
    01_yam_arm.sh                  /joint_states → YAM arm (CAN)
    02_aero_hand.sh                /right/joint_control → Aero Hand (serial)
    launch_all.sh                  tmux launcher (both above)

  utils/
    verify_topics.sh               Check all topics are publishing
    kill_teleop.sh                 Kill teleop tmux session
    sim_mujoco_hand.sh             MuJoCo Aero Hand only (hand-only, no hardware)
    sim_yam_with_hand.sh           MuJoCo YAM arm + Aero Hand (no base)
    sim_dragonbot.sh               MuJoCo full Dragonbot (base + arm + hand)
```

---

## ROS Package Reference

| Package | Role |
|---|---|
| `aero_hand_open` | Hardware driver — serial bridge to Aero Hand SDK |
| `aero_hand_open_msgs` | Custom messages: `JointControl`, `ActuatorStates`, `HandMocap` |
| `aero_hand_open_teleop` | `quest3_retargeting`, `spacemouse_twist_publisher`, MuJoCo viewer |
| `aero_hand_open_description` | URDF models, RViz visualization |
| `hand-tracking-sdk-ros2` | Quest 3 UDP/TCP hand-landmark bridge |
| `yam_teleop` | Quest 3 wrist pose → YAM arm IK (pyroki) — experimental |
| `gello_controller` | Gello publisher + YAM arm CAN controller |
| `robot_arms` | Generic arm abstraction (YAM, Piper) via i2rt SDK |
| `manus_glove_pkg` | Manus data-glove driver (not in converged mode) |
| `dex_retargeting_ros` | Advanced IK retargeting via dex_retargeting library |
