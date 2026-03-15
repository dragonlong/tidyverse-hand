# Quest 3 Teleop Stack — Testing & Validation Guide

Walks through simulator-only verification and safety-aware hardware bring-up of the
Quest 3 → Aero Hand + Gello arm teleoperation stack.

---

## Architecture Recap

```
Local PC (this machine)
  Quest 3 app (UDP :9000)
    └─ hand_tracking_bridge
         └─ /hands/right/landmarks  ──► quest3_retargeting
                                          └─► /right/joint_control ──► [Mobile PC] aero_hand_node → Aero Hand

  Gello device (USB/serial)
    └─ scripts/teleop/02_gello_arm.sh
         └─► /joint_states ──────────────────────────────────► [Mobile PC] yam_gello_controller → YAM arm
```

Both PCs must share the same `ROS_DOMAIN_ID`.

Arm teleop uses the **Gello kinesthetic controller** — the operator physically
guides a replica arm and the joint angles are mirrored to the YAM arm in real
time.  Quest 3 hand pose drives **only the Aero Hand fingers**.

---

## STAGE 1 — Simulator-Only Testing (no hardware)

Validates the full signal path from Quest 3 → retargeting before touching any robot.

### 1.1 Build

```bash
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  hand_tracking_sdk_ros2 \
  aero_hand_open_teleop
source install/setup.bash
```

### 1.2 Install Python deps (system pip, strip conda first)

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge3 | tr '\n' ':')
pip install mujoco scipy viser
```

### 1.3 Verify Quest 3 UDP stream

Start the Quest 3 hand tracking app on the headset, pointed at this machine's IP, port 9000.

```bash
# Terminal 1: bridge + retargeting (Quest 3 → /right/joint_control)
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# Terminal 2: check landmark topic rate
ros2 topic hz /hands/right/landmarks
```

Expected: `/hands/right/wrist_pose`, `/hands/right/landmarks` publishing at 60–70 Hz.

### 1.4 Aero Hand — MuJoCo finger visualization

```bash
# Terminal 1: bridge + retargeting only
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# Terminal 2: MuJoCo window
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_mujoco_hand.sh
```

**What to check:**
- MuJoCo window opens showing the Aero right hand in rest pose
- Moving fingers drives the model in real-time (no lag >100 ms)
- Terminal prints incoming Hz: expect ~60 Hz
- Thumb opposition, all four finger curl/extend look anatomically correct
- No joint flipping or discontinuities during fast motion

```bash
# Spot-check the 16 values live:
ros2 topic echo /right/joint_control
```

All 16 values should be in radians, within [0, π/2] for fingers and their respective limits for thumb.

### 1.5 Gello Arm — offline joint echo

Verify the Gello publisher is reading the device without connecting to the YAM arm.

```bash
bash ~/tetheria/tidyverse-hand/scripts/teleop/02_gello_arm.sh   # publishes /joint_states from Gello device
```

```bash
ros2 topic hz /joint_states          # expect ~60 Hz
ros2 topic echo /joint_states        # 6 joint values, move Gello arm to confirm
```

**What to check:**
- All 6 joints track Gello physical position
- No large discontinuities when slowly moving each joint
- Hz stays stable at ~60 Hz

### 1.6 Full combined sim check — YAM arm + hand in one window

Run all three simultaneously.  The combined viewer shows the YAM arm moving
with the Aero Hand attached at link_6, driven by both data streams at once.

```bash
# T1: Gello publisher
bash ~/tetheria/tidyverse-hand/scripts/teleop/02_gello_arm.sh

# T2: Quest 3 bridge + finger retargeting
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh

# T3: combined MuJoCo viewer (arm + hand)
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_yam_with_hand.sh
```

**What to check:**
- YAM arm joints mirror Gello arm position in 3-D
- Aero Hand on end of arm mirrors Quest 3 finger poses
- Both update in real-time with no drift or flipping
- Terminal prints `arm X.X Hz  hand X.X Hz` — both should be 30–60 Hz

> **Full Dragonbot (with base):** `bash scripts/utils/sim_dragonbot.sh` — adds
> the TidyBot mobile base; also subscribe to `/spacemouse/cmd_vel` (Terminal 0:
> `bash scripts/teleop/01_spacemouse.sh`) to drive the base live.
>
> **Hand-only alternative:** `bash scripts/utils/sim_mujoco_hand.sh` — shows
> only the floating Aero Hand, fastest to launch for retargeting-only checks.

**Pass criteria for Stage 1:**
- [ ] `/right/joint_control` publishing 30+ Hz, 16 values, anatomically sensible
- [ ] `/joint_states` publishing 30+ Hz, 6 values, from Gello device
- [ ] Combined MuJoCo window shows arm + hand with <100 ms latency
- [ ] Both streams stable for >60 s of continuous motion

---

## STAGE 2 — Hardware Bring-up: Aero Hand (fingers only)

### Prerequisites

- Aero Hand connected via USB serial to mobile PC
- Know the serial port: `ls /dev/ttyUSB* /dev/ttyACM*` on mobile PC

### 2.1 Mobile PC — start hand driver only

```bash
source /opt/ros/humble/setup.bash
cd ~/tetheria/aero-open-ros2 && source install/setup.bash
ros2 run aero_hand_open aero_hand_node --ros-args \
  -p right_port:=/dev/ttyUSB0 \        # ← adjust port
  -p feedback_frequency:=30.0
```

Or via the unified launcher:

```bash
export RIGHT_PORT=/dev/serial/by-id/usb-FTDI_USB_Serial_Cable_XXXXXXXX-if00-port0
bash ~/tetheria/tidyverse-hand/scripts/hardware/02_aero_hand.sh
```

Wait for: *"Aero Hand node started"* with no serial errors.

### 2.2 Safety check before sending motion

From mobile PC, verify the hand is in rest (open) position and tendons are not under unusual tension.

### 2.3 Local PC — send commands

```bash
bash ~/tetheria/tidyverse-hand/scripts/teleop/03_quest3_hand.sh
```

**Incremental motion test:**
1. Hold hand flat and open — fingers should stay extended
2. Slowly curl index finger — watch index MCP/PIP/DIP actuate
3. Make a full fist — all fingers curl
4. Extend thumb out — CMC abduction actuates
5. Thumb opposition to index — CMC flex + MCP

**What to check on mobile PC:**
```bash
ros2 topic echo /right/actuator_states
```
- `actuations` values (degrees) moving in sync with hand motion
- `actuator_currents` (mA): normal operating range <500 mA per actuator; stop if >800 mA sustained
- `actuator_temperatures` (°C): stop if any actuator >60°C

**Stop immediately if:**
- Loud grinding or clicking from tendons
- Any actuator current spike >1000 mA
- Fingers moving in wrong direction vs. hand pose

### 2.4 Validate mapping quality

Run MuJoCo viewer alongside hardware to compare sim vs. real:

```bash
bash ~/tetheria/tidyverse-hand/scripts/utils/sim_mujoco_hand.sh   # local PC, separate terminal
```

The MuJoCo model and the physical hand should move identically. Discrepancies indicate calibration drift in `normalize_default_mediapipe.yaml` — adjust `peak`/`valley` values for the affected thumb joints.

---

## STAGE 3 — Hardware Bring-up: YAM Arm via Gello (arm only, no hand)

### Prerequisites

- CAN bus adapter connected to mobile PC
- YAM arm powered, clear of obstacles, ~50 cm free space in all directions
- Gello device connected to local PC via USB/serial
- Know the CAN interface number (`can_num`, default 0)

### 3.1 Mobile PC — start arm controller only

```bash
bash ~/tetheria/tidyverse-hand/scripts/hardware/01_yam_arm.sh
```

Or manually:
```bash
# Strip conda; use system Python for ROS2 + i2rt CAN
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge3 | tr '\n' ':')
unset CONDA_PREFIX CONDA_DEFAULT_ENV

source /opt/ros/humble/setup.bash
cd ~/tetheria/aero-open-ros2 && source install/setup.bash
python3 src/gello_controller/gello_controller/yam_gello_controller.py
```

**The arm immediately moves to zero-joint home on startup.** Stand clear and watch.

Wait for: *"Initializing Robot Arm for right side"* + arm settles at home pose.

### 3.2 Verify arm is at home

Home pose is zero on all 6 joints.

```bash
ros2 topic echo /yam/joint_states   # all ~0.0 initially
```

### 3.3 Local PC — start Gello publisher

```bash
bash ~/tetheria/tidyverse-hand/scripts/teleop/02_gello_arm.sh
```

### 3.4 Incremental motion test

Hold the Gello arm at home (zero-joint) position, then slowly move each joint:

1. **Joint 1 (base rotation)**: YAM base rotates
2. **Joint 2 (shoulder)**: YAM shoulder moves
3. **Gradually expand** range joint by joint
4. Confirm no runaway motion — Gello joint → YAM joint mapping should be 1:1

```bash
ros2 topic hz /joint_states          # expect ~60 Hz from local PC
```

**Watch on mobile PC:**
YAM arm joints should mirror Gello position. Any jerky motion indicates CAN timing issue — check cable connections.

**Stop immediately if:**
- Arm moves to unexpected position far from Gello
- Joint velocity appears uncontrolled
- Any mechanical resistance / motor stall sound

---

## STAGE 4 — Full Combined Hardware Teleop

Only proceed after Stages 2 and 3 each pass independently.

### 4.1 Mobile PC (both drivers)

```bash
export RIGHT_PORT=/dev/serial/by-id/usb-FTDI_USB_Serial_Cable_XXXXXXXX-if00-port0
bash ~/tetheria/tidyverse-hand/scripts/hardware/launch_all.sh
```

Opens tmux session — pane 0: YAM arm (homes immediately), pane 1: Aero Hand.

Wait for arm to finish homing.

### 4.2 Local PC — full launch

```bash
bash ~/tetheria/tidyverse-hand/scripts/teleop/launch_all.sh
```

Opens tmux session — pane 0: SpaceMouse, pane 1: Gello arm, pane 2: Quest 3 hand.

### 4.3 Combined validation checklist

- [ ] Fingers: curl/extend mirrors Quest 3 hand pose in both MuJoCo and physical hand
- [ ] Arm: Gello joint motion drives YAM arm with correct 1:1 mapping
- [ ] Both systems respond simultaneously with no observable cross-interference
- [ ] Actuator currents nominal (<500 mA sustained) on Aero Hand
- [ ] `/right/joint_control` at 60 Hz, `/joint_states` at 60 Hz
- [ ] No arm runaway; smooth joint tracking

---

## Quick-Reference: Topic Health

```bash
ros2 topic hz /hands/right/landmarks     # 60–70 Hz  (Quest 3 bridge)
ros2 topic hz /right/joint_control       # ~60 Hz    (finger retargeting)
ros2 topic hz /joint_states              # ~60 Hz    (Gello publisher)
ros2 topic echo /right/actuator_states   # currents, temps (hardware only)
```

## Quick-Reference: Safety Stops

| Condition | Action |
|---|---|
| Actuator current >800 mA sustained | `Ctrl-C` aero_hand_node |
| Actuator temperature >60°C | `Ctrl-C` aero_hand_node, allow cool-down |
| Arm moves unexpectedly | `Ctrl-C` yam_gello_controller (arm will hold last position) |
| Any mechanical noise | `Ctrl-C` all nodes immediately; inspect hardware |

## Quick-Reference: Tuning Parameters

| Parameter | Default | Effect | How to override |
|---|---|---|---|
| `ema_alpha` (retargeting) | 0.7 | Finger smoothing (higher=less smooth) | `-p ema_alpha:=0.5` |
| `normalize_default_mediapipe` | — | Thumb valley/peak calibration | Edit config YAML directly |
| Gello `can_num` | 0 | CAN interface for YAM arm | Edit `yam_gello_controller.py` |

## Verify & Stop

```bash
# Check all topics are healthy
bash ~/tetheria/tidyverse-hand/scripts/utils/verify_topics.sh

# Tear down all teleop sessions
bash ~/tetheria/tidyverse-hand/scripts/utils/kill_teleop.sh
```
