#!/usr/bin/env python3
"""
Convert ROS2 rosbag (db3 or mcap) episodes to LeRobotDataset v3.

This script supports:
- ROS2 rosbag2 directories (sqlite3/db3 format with metadata.yaml)
- MCAP files directly
- Custom messages for Aero hand: JointControl, ActuatorStates, ManusGlove

Key features:
- Handles custom message types by parsing .msg definitions
- Synchronizes all topics to fixed FPS
- Supports images (raw or compressed)
- Outputs to LeRobot v3 format

Usage:
    # Source ROS2 workspace first for custom msgs:
    cd ~/tetheria/aero-open-ros2
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # Run conversion:
    python convert_rosbag_to_lerobot.py \\
        --input-dir /path/to/rosbags_folder \\
        --output-root ~/data/lerobot_dataset \\
        --repo-id "local/hand_teleop" \\
        --fps 30 \\
        --use-video \\
        --task "teleop demonstration"

Requirements:
    pip install rosbags lerobot pillow numpy
"""

from __future__ import annotations

import argparse
import bisect
import io
import math
import shutil
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from PIL import Image

# rosbags for reading
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# LeRobot
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Optional rerun support
try:
    import rerun as rr
except ImportError:
    rr = None


# -----------------------------------------------------------------------------
# Custom Message Definitions (embedded for standalone use)
# -----------------------------------------------------------------------------

# These match the .msg files in aero_hand_open_msgs and manus_ros2_msgs
CUSTOM_MSG_DEFS = {
    "aero_hand_open_msgs/msg/JointControl": """
std_msgs/Header header
float32[] target_positions
""",
    "aero_hand_open_msgs/msg/ActuatorStates": """
std_msgs/Header header
string side
float32[] actuations
float32[] actuator_speeds
float32[] actuator_currents
float32[] actuator_temperatures
""",
    "aero_hand_open_msgs/msg/ActuatorControl": """
std_msgs/Header header
string side
float32[] target_actuations
""",
    "aero_hand_open_msgs/msg/HandMocap": """
std_msgs/Header header
string side
geometry_msgs/Pose[] keypoints
""",
    "manus_ros2_msgs/msg/ManusRawNode": """
int32 node_id
int32 parent_node_id
string joint_type
string chain_type
geometry_msgs/Pose pose
""",
    "manus_ros2_msgs/msg/ManusErgonomics": """
string type
float32 value
""",
    "manus_ros2_msgs/msg/ManusGlove": """
int32 glove_id
string side
int32 raw_node_count
manus_ros2_msgs/ManusRawNode[] raw_nodes
int32 ergonomics_count
manus_ros2_msgs/ManusErgonomics[] ergonomics
geometry_msgs/Quaternion raw_sensor_orientation
int32 raw_sensor_count
geometry_msgs/Pose[] raw_sensor
""",
}

# Joint indices for JointControl (16 joints)
JOINT_CONTROL_NAMES = [
    "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_pip", "pinky_dip",
]


def create_typestore_with_custom_msgs():
    """Create a typestore with ROS2 Humble types and custom message definitions."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # Register custom messages (order matters for dependencies)
    # First register base types that others depend on
    for msgtype in ["manus_ros2_msgs/msg/ManusRawNode", "manus_ros2_msgs/msg/ManusErgonomics"]:
        if msgtype in CUSTOM_MSG_DEFS:
            try:
                types = get_types_from_msg(CUSTOM_MSG_DEFS[msgtype].strip(), msgtype)
                typestore.register(types)
            except Exception as e:
                print(f"Warning: Could not register {msgtype}: {e}")
    
    # Then register ManusGlove which depends on the above
    for msgtype, msgdef in CUSTOM_MSG_DEFS.items():
        if msgtype.startswith("manus_ros2_msgs") and msgtype not in ["manus_ros2_msgs/msg/ManusRawNode", "manus_ros2_msgs/msg/ManusErgonomics"]:
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception as e:
                print(f"Warning: Could not register {msgtype}: {e}")
    
    # Finally register aero_hand_open_msgs
    for msgtype, msgdef in CUSTOM_MSG_DEFS.items():
        if msgtype.startswith("aero_hand_open_msgs"):
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception as e:
                print(f"Warning: Could not register {msgtype}: {e}")
    
    return typestore

# Actuator indices for ActuatorStates (7 actuators)
ACTUATOR_NAMES = [
    "thumb_cmc_abd_act", "thumb_cmc_flex_act", "thumb_tendon_act",
    "index_tendon_act", "middle_tendon_act", "ring_tendon_act", "pinky_tendon_act",
]


# -----------------------------------------------------------------------------
# Helper Classes
# -----------------------------------------------------------------------------

@dataclass
class TimedMsg:
    """Message with timestamp."""
    t: float  # seconds since epoch
    msg: Any


@dataclass
class TopicConfig:
    """Configuration for a topic to read."""
    name: str
    aliases: List[str] = field(default_factory=list)
    required: bool = False


# -----------------------------------------------------------------------------
# Utility Functions
# -----------------------------------------------------------------------------

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle (z-axis rotation)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def try_decode_image_ros(msg: Any) -> Optional[np.ndarray]:
    """
    Decode ROS image message to RGB numpy array [H, W, 3].
    
    Supports:
    - sensor_msgs/msg/Image: rgb8, bgr8, mono8, rgba8, bgra8
    - sensor_msgs/msg/CompressedImage: JPEG/PNG
    """
    # CompressedImage: has 'format' and 'data' fields
    if hasattr(msg, "format") and hasattr(msg, "data"):
        data = msg.data
        if isinstance(data, (bytes, bytearray, memoryview)):
            try:
                im = Image.open(io.BytesIO(bytes(data))).convert("RGB")
                return np.asarray(im, dtype=np.uint8)
            except Exception:
                return None

    # Raw Image: height, width, encoding, data, step
    if not all(hasattr(msg, attr) for attr in ["height", "width", "encoding", "data"]):
        return None

    H, W = int(msg.height), int(msg.width)
    enc = str(msg.encoding).lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "rgb8":
        if buf.size != H * W * 3:
            return None
        return buf.reshape(H, W, 3).copy()

    if enc == "bgr8":
        if buf.size != H * W * 3:
            return None
        return buf.reshape(H, W, 3)[:, :, ::-1].copy()

    if enc == "mono8":
        if buf.size != H * W:
            return None
        g = buf.reshape(H, W)
        return np.stack([g, g, g], axis=-1)

    if enc in ("rgba8", "rgb8;jpeg"):
        if enc == "rgb8;jpeg":
            # JPEG compressed in data field
            try:
                im = Image.open(io.BytesIO(bytes(msg.data))).convert("RGB")
                return np.asarray(im, dtype=np.uint8)
            except Exception:
                return None
        if buf.size != H * W * 4:
            return None
        return buf.reshape(H, W, 4)[:, :, :3].copy()

    if enc == "bgra8":
        if buf.size != H * W * 4:
            return None
        bgra = buf.reshape(H, W, 4)
        return bgra[:, :, :3][:, :, ::-1].copy()

    return None


def _times(stream: List[TimedMsg]) -> List[float]:
    return [x.t for x in stream]


def latest_before(stream: List[TimedMsg], t: float) -> Optional[Any]:
    """Binary search: return latest msg with time <= t."""
    if not stream:
        return None
    ts = _times(stream)
    idx = bisect.bisect_right(ts, t) - 1
    if idx < 0:
        return None
    return stream[idx].msg


def messages_in_range(stream: List[TimedMsg], t_start: float, t_end: float) -> List[Any]:
    """Get all messages in time range [t_start, t_end)."""
    if not stream:
        return []
    ts = _times(stream)
    idx_start = bisect.bisect_left(ts, t_start)
    idx_end = bisect.bisect_left(ts, t_end)
    return [stream[i].msg for i in range(idx_start, idx_end)]


def average_twist_in_range(stream: List[TimedMsg], t_start: float, t_end: float) -> Tuple[float, float, float, float, float, float]:
    """Average all Twist messages in a time range (for delta velocity commands)."""
    msgs = messages_in_range(stream, t_start, t_end)
    if not msgs:
        # Fall back to latest before t_end
        msg = latest_before(stream, t_end)
        if msg is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        try:
            return (
                float(msg.linear.x), float(msg.linear.y), float(msg.linear.z),
                float(msg.angular.x), float(msg.angular.y), float(msg.angular.z),
            )
        except Exception:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # Average all messages in the range
    vx = vy = vz = wx = wy = wz = 0.0
    count = 0
    for msg in msgs:
        try:
            vx += float(msg.linear.x)
            vy += float(msg.linear.y)
            vz += float(msg.linear.z)
            wx += float(msg.angular.x)
            wy += float(msg.angular.y)
            wz += float(msg.angular.z)
            count += 1
        except Exception:
            continue
    
    if count == 0:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return (vx / count, vy / count, vz / count, wx / count, wy / count, wz / count)


def sum_twist_in_range(stream: List[TimedMsg], t_start: float, t_end: float) -> Tuple[float, float, float, float, float, float]:
    """Sum all Twist messages in a time range (for accumulating delta commands)."""
    msgs = messages_in_range(stream, t_start, t_end)
    if not msgs:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    vx = vy = vz = wx = wy = wz = 0.0
    for msg in msgs:
        try:
            vx += float(msg.linear.x)
            vy += float(msg.linear.y)
            vz += float(msg.linear.z)
            wx += float(msg.angular.x)
            wy += float(msg.angular.y)
            wz += float(msg.angular.z)
        except Exception:
            continue
    
    return (vx, vy, vz, wx, wy, wz)


def register_custom_types(typestore):
    """Register custom message types with the typestore."""
    for msgtype, msgdef in CUSTOM_MSG_DEFS.items():
        try:
            types = get_types_from_msg(msgdef.strip(), msgtype)
            typestore.register(types)
        except Exception as e:
            print(f"Warning: Could not register {msgtype}: {e}")


# -----------------------------------------------------------------------------
# Reading Rosbag Data
# -----------------------------------------------------------------------------

def read_rosbag_topics(
    path: Path,
    topics: Sequence[str],
    debug_topics: bool = False,
) -> Dict[str, List[TimedMsg]]:
    """
    Read messages for the given topics from a rosbag2 directory or mcap file.
    
    Args:
        path: Path to rosbag2 directory (with metadata.yaml) or .mcap file
        topics: List of topic names to read
        debug_topics: If True, print available topics and counts
        
    Returns:
        Dictionary mapping topic name to list of TimedMsg
    """
    collected: Dict[str, List[TimedMsg]] = {tp: [] for tp in topics}
    
    # Build alias map for flexible topic matching
    alias_map: Dict[str, str] = {}
    for tp in topics:
        base = tp.lstrip("/")
        alias_map[tp] = tp
        alias_map[base] = tp
        alias_map[f"/{base}"] = tp
    
    available_topics: List[str] = []
    typestore = create_typestore_with_custom_msgs()
    
    try:
        with AnyReader([path], default_typestore=typestore) as reader:
            available_topics = list(reader.topics.keys())
            
            for conn, t_ns, raw in reader.messages():
                target = alias_map.get(conn.topic)
                if target is None:
                    continue
                try:
                    msg = reader.deserialize(raw, conn.msgtype)
                    collected[target].append(TimedMsg(t=float(t_ns) * 1e-9, msg=msg))
                except Exception as e:
                    if debug_topics:
                        print(f"  Deserialize error on {conn.topic}: {e}")
                    continue
                    
    except Exception as e:
        print(f"Error reading {path}: {e}")
        return collected
    
    # Sort by timestamp
    for tp in collected:
        collected[tp].sort(key=lambda x: x.t)
    
    if debug_topics or all(len(v) == 0 for v in collected.values()):
        print(f"  Available topics: {sorted(available_topics)}")
        counts = {k: len(v) for k, v in collected.items()}
        print(f"  Collected counts: {counts}")
    
    return collected


# -----------------------------------------------------------------------------
# Feature Extraction from Messages
# -----------------------------------------------------------------------------

def extract_twist(msg: Any) -> Tuple[float, float, float, float, float, float]:
    """Extract linear and angular velocities from Twist message."""
    if msg is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    try:
        return (
            float(msg.linear.x), float(msg.linear.y), float(msg.linear.z),
            float(msg.angular.x), float(msg.angular.y), float(msg.angular.z),
        )
    except Exception:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def extract_joint_state(msg: Any, num_joints: int = 7) -> np.ndarray:
    """Extract joint positions from JointState message."""
    result = np.zeros(num_joints, dtype=np.float32)
    if msg is None:
        return result
    try:
        pos = np.asarray(msg.position, dtype=np.float32).flatten()
        n = min(len(pos), num_joints)
        result[:n] = pos[:n]
    except Exception:
        pass
    return result


def extract_joint_control(msg: Any, num_joints: int = 16) -> np.ndarray:
    """Extract target positions from JointControl message."""
    result = np.zeros(num_joints, dtype=np.float32)
    if msg is None:
        return result
    try:
        pos = np.asarray(msg.target_positions, dtype=np.float32).flatten()
        n = min(len(pos), num_joints)
        result[:n] = pos[:n]
    except Exception:
        pass
    return result


def extract_actuator_states(msg: Any, num_actuators: int = 7) -> Tuple[np.ndarray, np.ndarray]:
    """Extract actuations and speeds from ActuatorStates message."""
    actuations = np.zeros(num_actuators, dtype=np.float32)
    speeds = np.zeros(num_actuators, dtype=np.float32)
    if msg is None:
        return actuations, speeds
    try:
        if hasattr(msg, "actuations"):
            act = np.asarray(msg.actuations, dtype=np.float32).flatten()
            n = min(len(act), num_actuators)
            actuations[:n] = act[:n]
        if hasattr(msg, "actuator_speeds"):
            spd = np.asarray(msg.actuator_speeds, dtype=np.float32).flatten()
            n = min(len(spd), num_actuators)
            speeds[:n] = spd[:n]
    except Exception:
        pass
    return actuations, speeds


def extract_manus_glove_ergonomics(msg: Any, num_values: int = 20) -> np.ndarray:
    """Extract ergonomics values from ManusGlove message."""
    result = np.zeros(num_values, dtype=np.float32)
    if msg is None:
        return result
    try:
        if hasattr(msg, "ergonomics"):
            for i, ergo in enumerate(msg.ergonomics):
                if i >= num_values:
                    break
                result[i] = float(ergo.value)
    except Exception:
        pass
    return result


# -----------------------------------------------------------------------------
# Episode Segmentation
# -----------------------------------------------------------------------------

@dataclass
class EpisodeBoundary:
    """Represents episode start/end times within a rosbag."""
    start_time: float
    end_time: float
    label: str = ""  # Optional label for the episode


def segment_episodes_by_time_gap(
    collected: Dict[str, List[TimedMsg]],
    gap_threshold: float = 2.0,
) -> List[EpisodeBoundary]:
    """
    Segment episodes based on time gaps in the data.
    
    If there's a gap > gap_threshold seconds between consecutive messages
    across all topics, it's considered an episode boundary.
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        gap_threshold: Minimum gap (seconds) to consider as episode boundary
        
    Returns:
        List of EpisodeBoundary objects
    """
    # Collect all timestamps across all topics
    all_times = []
    for stream in collected.values():
        all_times.extend([m.t for m in stream])
    
    if not all_times:
        return []
    
    all_times = sorted(set(all_times))
    
    if len(all_times) < 2:
        return [EpisodeBoundary(all_times[0], all_times[0])]
    
    # Find gaps
    episodes = []
    ep_start = all_times[0]
    
    for i in range(1, len(all_times)):
        gap = all_times[i] - all_times[i-1]
        if gap > gap_threshold:
            # End current episode, start new one
            episodes.append(EpisodeBoundary(ep_start, all_times[i-1]))
            ep_start = all_times[i]
    
    # Add final episode
    episodes.append(EpisodeBoundary(ep_start, all_times[-1]))
    
    return episodes


def segment_episodes_by_joint_home_position(
    collected: Dict[str, List[TimedMsg]],
    topic_joint_states: str,
    home_position: np.ndarray = None,
    threshold: float = 0.1,
    min_episode_duration: float = 3.0,
) -> List[EpisodeBoundary]:
    """
    Segment episodes based on arm returning to home position.
    
    When the arm joints are close to home_position for a brief period,
    it's considered an episode boundary.
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        topic_joint_states: Topic name for joint states
        home_position: Home joint positions (if None, uses first position)
        threshold: Joint position threshold to consider "at home"
        min_episode_duration: Minimum duration for an episode
        
    Returns:
        List of EpisodeBoundary objects
    """
    joint_stream = collected.get(topic_joint_states, [])
    if not joint_stream:
        return []
    
    # Use first position as home if not specified
    if home_position is None:
        first_msg = joint_stream[0].msg
        if hasattr(first_msg, "position"):
            home_position = np.array(first_msg.position, dtype=np.float32)
        else:
            return []
    
    episodes = []
    ep_start = joint_stream[0].t
    at_home_start = None
    
    for tm in joint_stream:
        if hasattr(tm.msg, "position"):
            pos = np.array(tm.msg.position, dtype=np.float32)
            dist = np.linalg.norm(pos - home_position[:len(pos)])
            
            if dist < threshold:
                if at_home_start is None:
                    at_home_start = tm.t
            else:
                if at_home_start is not None:
                    # Was at home, now moved away
                    if tm.t - ep_start > min_episode_duration:
                        episodes.append(EpisodeBoundary(ep_start, at_home_start))
                    ep_start = tm.t
                    at_home_start = None
    
    # Add final episode
    if joint_stream[-1].t - ep_start > min_episode_duration:
        episodes.append(EpisodeBoundary(ep_start, joint_stream[-1].t))
    
    return episodes


def segment_episodes_by_audio(
    collected: Dict[str, List[TimedMsg]],
    topic_audio: str = "/audio",
) -> List[EpisodeBoundary]:
    """
    [PLACEHOLDER] Segment episodes based on audio commands.
    
    This function is a placeholder for future implementation.
    It would use speech recognition to detect commands like:
    - "start" / "begin" -> episode start
    - "stop" / "done" / "end" -> episode end
    - "reset" / "cancel" -> discard current episode
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        topic_audio: Audio topic name
        
    Returns:
        List of EpisodeBoundary objects
        
    TODO:
        - Integrate speech recognition (whisper, vosk, etc.)
        - Define command vocabulary
        - Handle noisy environments
    """
    print("  [Audio segmentation not yet implemented - using single episode]")
    
    # Placeholder: return empty to fall back to single episode
    return []


def segment_episodes_by_gripper_state(
    collected: Dict[str, List[TimedMsg]],
    topic_hand_control: str,
    open_threshold: float = 0.8,  # Gripper "open" position threshold
    min_episode_duration: float = 3.0,
) -> List[EpisodeBoundary]:
    """
    Segment episodes based on gripper/hand opening (task completion pattern).
    
    Many manipulation tasks end with the gripper opening to release an object.
    This function detects when the hand opens wide as episode boundaries.
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        topic_hand_control: Hand control topic
        open_threshold: Threshold for "open" state (normalized)
        min_episode_duration: Minimum episode duration
        
    Returns:
        List of EpisodeBoundary objects
    """
    hand_stream = collected.get(topic_hand_control, [])
    if not hand_stream:
        return []
    
    episodes = []
    ep_start = hand_stream[0].t
    was_open = False
    
    for tm in hand_stream:
        if hasattr(tm.msg, "target_positions"):
            positions = np.array(tm.msg.target_positions, dtype=np.float32)
            # Check if hand is "open" (average position above threshold)
            # This is task-specific and may need tuning
            avg_pos = np.mean(np.abs(positions))
            is_open = avg_pos > open_threshold
            
            if is_open and not was_open:
                # Hand just opened - potential episode end
                if tm.t - ep_start > min_episode_duration:
                    episodes.append(EpisodeBoundary(ep_start, tm.t))
                    ep_start = tm.t
            
            was_open = is_open
    
    # Add final episode
    if hand_stream[-1].t - ep_start > min_episode_duration:
        episodes.append(EpisodeBoundary(ep_start, hand_stream[-1].t))
    
    return episodes


def detect_episode_boundaries(
    collected: Dict[str, List[TimedMsg]],
    method: str = "none",
    config: Dict[str, Any] = None,
) -> List[EpisodeBoundary]:
    """
    Detect episode boundaries within a rosbag using the specified method.
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        method: Segmentation method:
            - "none": Treat entire bag as single episode (default)
            - "time_gap": Segment by time gaps
            - "joint_home": Segment by arm returning to home position
            - "gripper": Segment by gripper opening
            - "audio": Segment by audio commands [placeholder]
        config: Method-specific configuration
        
    Returns:
        List of EpisodeBoundary objects
    """
    if config is None:
        config = {}
    
    if method == "none":
        # Single episode spanning entire data
        all_times = []
        for stream in collected.values():
            all_times.extend([m.t for m in stream])
        if not all_times:
            return []
        return [EpisodeBoundary(min(all_times), max(all_times))]
    
    elif method == "time_gap":
        gap_threshold = config.get("gap_threshold", 2.0)
        return segment_episodes_by_time_gap(collected, gap_threshold)
    
    elif method == "joint_home":
        topic = config.get("topic_joint_states", "/joint_states")
        threshold = config.get("threshold", 0.1)
        min_duration = config.get("min_episode_duration", 3.0)
        return segment_episodes_by_joint_home_position(
            collected, topic, threshold=threshold, min_episode_duration=min_duration
        )
    
    elif method == "gripper":
        topic = config.get("topic_hand_control", "/right/joint_control")
        threshold = config.get("open_threshold", 0.8)
        min_duration = config.get("min_episode_duration", 3.0)
        return segment_episodes_by_gripper_state(
            collected, topic, threshold, min_duration
        )
    
    elif method == "audio":
        topic = config.get("topic_audio", "/audio")
        return segment_episodes_by_audio(collected, topic)
    
    else:
        print(f"  Warning: Unknown segmentation method '{method}', using 'none'")
        return detect_episode_boundaries(collected, "none", config)


# -----------------------------------------------------------------------------
# Build Fixed-FPS Frames
# -----------------------------------------------------------------------------

def build_fixed_fps_frames(
    collected: Dict[str, List[TimedMsg]],
    fps: int,
    config: Dict[str, Any],
    time_range: Optional[Tuple[float, float]] = None,
) -> List[Dict[str, Any]]:
    """
    Resample all signals to fixed FPS frames.
    
    Args:
        collected: Dictionary of topic -> list of TimedMsg
        fps: Target frame rate
        config: Configuration with topic names and dimensions
            - sync_mode: "sample" (zero-order hold), "average" (mean in interval), 
                         or "sum" (accumulate deltas in interval)
        time_range: Optional (t0, t1) tuple to limit the time range for this episode
        
    Returns:
        List of frame dictionaries ready for LeRobot
    
    Synchronization modes:
        - "sample": Take the latest message before each frame (zero-order hold)
                    Best for: position targets, absolute values
        - "average": Average all messages in the frame interval
                    Best for: velocity commands where you want smoothing
        - "sum": Sum all messages in the frame interval  
                    Best for: delta commands where each contributes to motion
    """
    # Determine time span
    if time_range is not None:
        t0, t1 = time_range
    else:
        all_ts: List[float] = []
        for st in collected.values():
            all_ts.extend([x.t for x in st])
        if not all_ts:
            return []
        t0, t1 = min(all_ts), max(all_ts)
    
    if t1 <= t0:
        return []
    
    dt = 1.0 / float(fps)
    times = np.arange(t0, t1 + 1e-9, dt, dtype=np.float64)
    
    frames: List[Dict[str, Any]] = []
    task_value = config.get("task", "")
    
    # Image settings
    use_video = config.get("use_video", False)
    H, W = config.get("image_hw", (480, 640))
    
    # Camera configuration: dict of {name: topic}
    # e.g., {"head": "/camera_0/color", "wrist": "/camera_4/color", "base": "/logitech_base/color"}
    camera_topics = config.get("camera_topics", {})
    
    # Check if we have images for each camera
    if use_video:
        for cam_name, topic in camera_topics.items():
            img_stream = collected.get(topic, [])
            if len(img_stream) == 0:
                print(f"  Warning: No images found on {topic} for camera '{cam_name}'")
    
    # Track last image for each camera (for carry-forward)
    last_pil_imgs: Dict[str, Optional[Image.Image]] = {name: None for name in camera_topics}
    
    # Synchronization mode for velocity commands
    sync_mode = config.get("sync_mode", "sample")  # "sample", "average", or "sum"
    
    for i, ti in enumerate(times):
        frame: Dict[str, Any] = {"task": task_value}
        
        # Time range for this frame (used for average/sum modes)
        t_start = times[i - 1] if i > 0 else ti - dt
        t_end = ti
        
        # --- SpaceMouse cmd_vel (Twist) ---
        topic_cmd = config.get("topic_cmd_vel", "/spacemouse/cmd_vel")
        cmd_stream = collected.get(topic_cmd, [])
        
        if sync_mode == "average":
            vx, vy, vz, wx, wy, wz = average_twist_in_range(cmd_stream, t_start, t_end)
        elif sync_mode == "sum":
            vx, vy, vz, wx, wy, wz = sum_twist_in_range(cmd_stream, t_start, t_end)
        else:  # "sample" mode (default)
            cmd_msg = latest_before(cmd_stream, float(ti))
            vx, vy, vz, wx, wy, wz = extract_twist(cmd_msg)
        
        # --- Arm joint states ---
        topic_arm = config.get("topic_joint_states", "/joint_states")
        arm_joints_dim = config.get("arm_joint_dim", 7)
        arm_msg = latest_before(collected.get(topic_arm, []), float(ti))
        arm_joints = extract_joint_state(arm_msg, arm_joints_dim)
        
        # --- Hand joint control (target) ---
        topic_hand_ctrl = config.get("topic_hand_control", "/right/joint_control")
        hand_joints_dim = config.get("hand_joint_dim", 16)
        hand_ctrl_msg = latest_before(collected.get(topic_hand_ctrl, []), float(ti))
        hand_target = extract_joint_control(hand_ctrl_msg, hand_joints_dim)
        
        # --- Hand actuator states (actual) ---
        topic_hand_state = config.get("topic_actuator_states", "/right/actuator_states")
        num_actuators = config.get("num_actuators", 7)
        actuator_msg = latest_before(collected.get(topic_hand_state, []), float(ti))
        actuations, actuator_speeds = extract_actuator_states(actuator_msg, num_actuators)
        
        # --- Manus glove (human hand pose) ---
        topic_manus = config.get("topic_manus", "/manus_glove_0")
        manus_dim = config.get("manus_dim", 20)
        manus_msg = latest_before(collected.get(topic_manus, []), float(ti))
        manus_ergo = extract_manus_glove_ergonomics(manus_msg, manus_dim)
        
        # Build state vector:
        # [arm_joints(7), hand_actuations(7), manus_ergo(20)] = 34
        # Note: No base state as we control in local frame
        state = np.concatenate([
            arm_joints,  # 7: current arm joint positions
            actuations,  # 7: current hand actuator positions  
            manus_ergo,  # 20: human hand tracking from manus glove
        ], axis=0)
        
        # Build action vector:
        # [base_cmd(3), arm_joints(7), hand_target(16)] = 26
        # - base_cmd: spacemouse velocity (vx, vy, wz) for local frame control
        # - arm_joints: gello-style direct mapping (joint states = target positions)
        # - hand_target: hand joint control targets
        action = np.concatenate([
            np.array([vx, vy, wz], dtype=np.float32),  # 3: base velocity (local frame)
            arm_joints,  # 7: arm joint targets (gello-style)
            hand_target,  # 16: hand joint targets
        ], axis=0)
        
        frame["observation.state"] = state
        frame["action"] = action
        
        # --- Images (multiple cameras) ---
        if use_video and camera_topics:
            for cam_name, topic in camera_topics.items():
                pil_img: Optional[Image.Image] = None
                img_msg = latest_before(collected.get(topic, []), float(ti))
                if img_msg is not None:
                    rgb = try_decode_image_ros(img_msg)
                    if rgb is not None:
                        # Resize if needed
                        if rgb.shape[0] != H or rgb.shape[1] != W:
                            rgb = np.asarray(
                                Image.fromarray(rgb).resize((W, H), resample=Image.BILINEAR),
                                dtype=np.uint8,
                            )
                        pil_img = Image.fromarray(rgb)
                
                # Carry forward last image or use black
                if pil_img is None:
                    pil_img = last_pil_imgs.get(cam_name)
                if pil_img is None:
                    pil_img = Image.fromarray(np.zeros((H, W, 3), dtype=np.uint8))
                
                last_pil_imgs[cam_name] = pil_img
                frame[f"observation.images.{cam_name}"] = pil_img
        
        frames.append(frame)
    
    return frames


# -----------------------------------------------------------------------------
# LeRobot Dataset Creation
# -----------------------------------------------------------------------------

def make_features(
    state_dim: int,
    action_dim: int,
    use_video: bool,
    image_hw: Tuple[int, int],
    camera_names: List[str] = None,
) -> Dict[str, Any]:
    """Create LeRobot features dictionary.
    
    Args:
        state_dim: Dimension of state vector
        action_dim: Dimension of action vector
        use_video: Whether to include video features
        image_hw: (height, width) of images
        camera_names: List of camera names (e.g., ["head", "wrist", "base"])
    """
    H, W = image_hw
    feats: Dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (state_dim,),
        },
        "action": {
            "dtype": "float32",
            "shape": (action_dim,),
        },
    }
    if use_video and camera_names:
        for cam_name in camera_names:
            feats[f"observation.images.{cam_name}"] = {
                "dtype": "video",
                "shape": (H, W, 3),
            }
    return feats


def find_rosbags(input_dir: Path) -> List[Path]:
    """Find all rosbag directories and mcap files in the input directory."""
    bags = []
    
    # Check if input_dir is itself a rosbag
    if (input_dir / "metadata.yaml").exists():
        bags.append(input_dir)
    elif input_dir.suffix == ".mcap":
        bags.append(input_dir)
    else:
        # Search for rosbag directories
        for item in sorted(input_dir.iterdir()):
            if item.is_dir() and (item / "metadata.yaml").exists():
                bags.append(item)
            elif item.suffix == ".mcap":
                bags.append(item)
    
    return bags


def convert_rosbags_to_lerobot(
    input_dir: Path,
    output_root: Path,
    repo_id: str,
    robot_type: str,
    fps: int,
    task: str,
    use_video: bool,
    image_hw: Tuple[int, int],
    overwrite: bool,
    debug_topics: bool,
    # Topic configuration
    topic_cmd_vel: str,
    topic_joint_states: str,
    topic_hand_control: str,
    topic_actuator_states: str,
    topic_manus: str,
    # Camera topics: dict of {name: topic}
    camera_topics: Dict[str, str] = None,
    # Dimension configuration
    arm_joint_dim: int = 7,
    hand_joint_dim: int = 16,
    num_actuators: int = 7,
    manus_dim: int = 20,
    # Synchronization
    sync_mode: str = "sample",
    # Episode segmentation
    episode_segmentation: str = "none",
    episode_gap_threshold: float = 2.0,
    episode_min_duration: float = 3.0,
) -> Path:
    """
    Convert rosbag recordings to LeRobot dataset.
    
    State vector: [arm_joints(7), hand_actuations(7), manus_ergo(20)] = 34
    Action vector: [base_cmd(3), arm_joints(7), hand_target(16)] = 26
    
    Cameras (optional):
        - head: /camera_0/color (front view)
        - wrist: /camera_4/color (wrist camera)
        - base: /logitech_base/color (base camera)
    """
    if output_root.exists():
        if overwrite:
            shutil.rmtree(output_root)
        else:
            raise FileExistsError(f"Output exists: {output_root} (use --overwrite)")
    
    if camera_topics is None:
        camera_topics = {}
    
    # Calculate dimensions
    # state: arm(7) + actuations(7) + manus(20) = 34
    state_dim = arm_joint_dim + num_actuators + manus_dim
    # action: base_cmd(3) + arm(7) + hand_target(16) = 26
    action_dim = 3 + arm_joint_dim + hand_joint_dim
    
    camera_names = list(camera_topics.keys()) if use_video else []
    
    features = make_features(
        state_dim=state_dim,
        action_dim=action_dim,
        use_video=use_video,
        image_hw=image_hw,
        camera_names=camera_names,
    )
    
    print(f"Creating LeRobot dataset:")
    print(f"  State dim: {state_dim} = arm({arm_joint_dim}) + actuators({num_actuators}) + manus({manus_dim})")
    print(f"  Action dim: {action_dim} = base_cmd(3) + arm({arm_joint_dim}) + hand({hand_joint_dim})")
    print(f"  Cameras: {list(camera_topics.keys()) if camera_topics else 'none'}")
    print(f"  Features: {list(features.keys())}")
    
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_root,
        fps=fps,
        robot_type=robot_type,
        features=features,
        use_videos=use_video,
    )
    
    # Topics to read
    topics = [
        topic_cmd_vel,
        topic_joint_states,
        topic_hand_control,
        topic_actuator_states,
        topic_manus,
    ]
    # Add camera topics
    if use_video and camera_topics:
        topics.extend(camera_topics.values())
    
    # Configuration for frame building
    config = {
        "task": task,
        "use_video": use_video,
        "camera_topics": camera_topics,  # dict of {name: topic}
        "image_hw": image_hw,
        "topic_cmd_vel": topic_cmd_vel,
        "topic_joint_states": topic_joint_states,
        "topic_hand_control": topic_hand_control,
        "topic_actuator_states": topic_actuator_states,
        "topic_manus": topic_manus,
        "arm_joint_dim": arm_joint_dim,
        "hand_joint_dim": hand_joint_dim,
        "num_actuators": num_actuators,
        "manus_dim": manus_dim,
        "sync_mode": sync_mode,
        "episode_segmentation": episode_segmentation,
        "gap_threshold": episode_gap_threshold,
        "min_episode_duration": episode_min_duration,
    }
    
    print(f"  Sync mode: {sync_mode}")
    
    # Episode segmentation info
    episode_method = episode_segmentation
    print(f"  Episode segmentation: {episode_method}")
    
    # Find and process bags
    bags = find_rosbags(input_dir)
    if not bags:
        raise SystemExit(f"No rosbags found in: {input_dir}")
    
    print(f"Found {len(bags)} rosbag(s)")
    
    total_episodes = 0
    
    for bag_idx, bag_path in enumerate(bags):
        print(f"\n[Bag {bag_idx+1}/{len(bags)}] {bag_path.name}")
        
        collected = read_rosbag_topics(bag_path, topics, debug_topics=debug_topics)
        
        # Detect episode boundaries within this bag
        episode_boundaries = detect_episode_boundaries(collected, episode_method, config)
        
        if not episode_boundaries:
            print("  -> No episodes detected, skipping")
            continue
        
        print(f"  -> Detected {len(episode_boundaries)} episode(s)")
        
        for ep_idx, boundary in enumerate(episode_boundaries):
            duration = boundary.end_time - boundary.start_time
            print(f"    Episode {ep_idx+1}: {duration:.1f}s [{boundary.start_time:.1f} - {boundary.end_time:.1f}]")
            
            frames = build_fixed_fps_frames(
                collected, fps, config,
                time_range=(boundary.start_time, boundary.end_time)
            )
            
            if not frames:
                print(f"      -> No frames, skipping")
                continue
            
            # Write frames
            for fr in frames:
                ds.add_frame(fr)
            
            ds.save_episode()
            total_episodes += 1
            print(f"      -> Wrote {len(frames)} frames")
    
    ds.finalize()
    print(f"\nDone. Dataset saved to: {output_root}")
    print(f"Total episodes: {total_episodes}")
    return output_root


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS2 rosbag recordings to LeRobot v3 dataset"
    )
    
    # Input/Output
    parser.add_argument(
        "--input-dir", type=Path, required=True,
        help="Directory containing rosbag2 folders or .mcap files"
    )
    parser.add_argument(
        "--output-root", type=Path, required=True,
        help="Output LeRobot dataset directory"
    )
    parser.add_argument(
        "--repo-id", type=str, required=True,
        help='Dataset repo ID (e.g., "local/hand_teleop")'
    )
    parser.add_argument(
        "--robot-type", type=str, default="aero_hand",
        help="Robot type identifier"
    )
    
    # Frame settings
    parser.add_argument("--fps", type=int, default=30, help="Target FPS")
    parser.add_argument(
        "--task", type=str, default="hand teleop demonstration",
        help="Task description"
    )
    
    # Video settings
    parser.add_argument("--use-video", action="store_true", help="Include video frames")
    parser.add_argument("--image-h", type=int, default=480, help="Image height")
    parser.add_argument("--image-w", type=int, default=640, help="Image width")
    
    # Topic configuration
    parser.add_argument(
        "--topic-cmd-vel", type=str, default="/spacemouse/cmd_vel",
        help="SpaceMouse velocity command topic"
    )
    parser.add_argument(
        "--topic-joint-states", type=str, default="/joint_states",
        help="Arm joint states topic"
    )
    parser.add_argument(
        "--topic-hand-control", type=str, default="/right/joint_control",
        help="Hand joint control target topic"
    )
    parser.add_argument(
        "--topic-actuator-states", type=str, default="/right/actuator_states",
        help="Hand actuator states topic"
    )
    parser.add_argument(
        "--topic-manus", type=str, default="/manus_glove_0",
        help="Manus glove topic"
    )
    
    # Camera topics (multiple cameras supported)
    parser.add_argument(
        "--camera-head", type=str, default="/camera_0/color",
        help="Head camera topic (front view)"
    )
    parser.add_argument(
        "--camera-wrist", type=str, default=None,
        help="Wrist camera topic (optional, e.g., /camera_4/color)"
    )
    parser.add_argument(
        "--camera-base", type=str, default=None,
        help="Base camera topic (optional, e.g., /logitech_base/color)"
    )
    
    # Dimension configuration
    parser.add_argument(
        "--arm-joint-dim", type=int, default=7,
        help="Number of arm joints"
    )
    parser.add_argument(
        "--hand-joint-dim", type=int, default=16,
        help="Number of hand joints (JointControl)"
    )
    parser.add_argument(
        "--num-actuators", type=int, default=7,
        help="Number of hand actuators"
    )
    parser.add_argument(
        "--manus-dim", type=int, default=20,
        help="Number of Manus ergonomics values"
    )
    
    # Synchronization options
    parser.add_argument(
        "--sync-mode", type=str, default="sample",
        choices=["sample", "average", "sum"],
        help="""Synchronization mode for velocity commands:
            'sample': Take latest value before each frame (zero-order hold) - default
            'average': Average all values in frame interval (smoothing)
            'sum': Sum all values in frame interval (for delta commands)"""
    )
    
    # Episode segmentation options
    parser.add_argument(
        "--episode-segmentation", type=str, default="none",
        choices=["none", "time_gap", "joint_home", "gripper", "audio"],
        help="""Method to segment episodes within each rosbag:
            'none': Treat entire bag as single episode (default)
            'time_gap': Segment by time gaps (pauses > threshold)
            'joint_home': Segment when arm returns to home position
            'gripper': Segment when hand/gripper opens (task completion)
            'audio': Segment by audio commands [placeholder - not yet implemented]"""
    )
    parser.add_argument(
        "--episode-gap-threshold", type=float, default=2.0,
        help="Time gap threshold (seconds) for time_gap segmentation"
    )
    parser.add_argument(
        "--episode-min-duration", type=float, default=3.0,
        help="Minimum episode duration (seconds)"
    )
    
    # Other options
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output")
    parser.add_argument("--debug-topics", action="store_true", help="Print topic debug info")
    
    args = parser.parse_args()
    
    # Build camera topics dict
    camera_topics = {}
    if args.use_video:
        if args.camera_head:
            camera_topics["head"] = args.camera_head
        if args.camera_wrist:
            camera_topics["wrist"] = args.camera_wrist
        if args.camera_base:
            camera_topics["base"] = args.camera_base
    
    convert_rosbags_to_lerobot(
        input_dir=args.input_dir,
        output_root=args.output_root,
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        fps=args.fps,
        task=args.task,
        use_video=args.use_video,
        image_hw=(args.image_h, args.image_w),
        overwrite=args.overwrite,
        debug_topics=args.debug_topics,
        topic_cmd_vel=args.topic_cmd_vel,
        topic_joint_states=args.topic_joint_states,
        topic_hand_control=args.topic_hand_control,
        topic_actuator_states=args.topic_actuator_states,
        topic_manus=args.topic_manus,
        camera_topics=camera_topics,
        arm_joint_dim=args.arm_joint_dim,
        hand_joint_dim=args.hand_joint_dim,
        num_actuators=args.num_actuators,
        manus_dim=args.manus_dim,
        sync_mode=args.sync_mode,
        episode_segmentation=args.episode_segmentation,
        episode_gap_threshold=args.episode_gap_threshold,
        episode_min_duration=args.episode_min_duration,
    )


if __name__ == "__main__":
    main()
