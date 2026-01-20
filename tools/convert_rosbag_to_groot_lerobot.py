#!/usr/bin/env python3
"""
Convert ROS2 rosbag to GR00T LeRobot v2 format.

This script converts rosbag recordings to the LeRobot v2 format required for
fine-tuning GR00T N1 models.

Output structure:
    ├─meta/
    │ ├─episodes.jsonl
    │ ├─modality.json    (GR00T specific)
    │ ├─info.json
    │ └─tasks.jsonl
    ├─videos/chunk-000/
    │ └─observation.images.<view_name>/
    │   └─episode_XXXXXX.mp4
    └─data/chunk-000/
      └─episode_XXXXXX.parquet

Usage:
    # mamba activate tidybot2 (not needed here)
    cd ~/tetheria/aero-open-ros2
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    python3 convert_rosbag_to_groot_lerobot.py \\
        --input-dir ~/tetheria/tidyverse-hand/data/rosbag2_xxx \\
        --output-root ~/tetheria/tidyverse-hand/data/lerobot_v2/dataset_name \\
        --fps 30 \\
        --use-video \\
        --task "hand teleop demonstration"
"""

from __future__ import annotations

import argparse
import bisect
import io
import json
import math
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

# rosbags for reading
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


# -----------------------------------------------------------------------------
# Custom Message Definitions
# -----------------------------------------------------------------------------

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


def create_typestore_with_custom_msgs():
    """Create a typestore with ROS2 Humble types and custom message definitions."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # Register in dependency order
    for msgtype in ["manus_ros2_msgs/msg/ManusRawNode", "manus_ros2_msgs/msg/ManusErgonomics"]:
        if msgtype in CUSTOM_MSG_DEFS:
            try:
                types = get_types_from_msg(CUSTOM_MSG_DEFS[msgtype].strip(), msgtype)
                typestore.register(types)
            except Exception:
                pass
    
    for msgtype, msgdef in CUSTOM_MSG_DEFS.items():
        if "ManusGlove" in msgtype:
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception:
                pass
    
    for msgtype, msgdef in CUSTOM_MSG_DEFS.items():
        if msgtype.startswith("aero_hand_open_msgs"):
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception:
                pass
    
    return typestore


# -----------------------------------------------------------------------------
# Data Structures
# -----------------------------------------------------------------------------

@dataclass
class TimedMsg:
    t: float
    msg: Any


@dataclass
class ModalityConfig:
    """Configuration for state/action modalities."""
    state_fields: Dict[str, Tuple[int, int]]  # {field_name: (start, end)}
    action_fields: Dict[str, Tuple[int, int]]
    video_fields: Dict[str, str]  # {new_key: original_key}
    annotation_fields: List[str]


# -----------------------------------------------------------------------------
# Message Extraction
# -----------------------------------------------------------------------------

def _times(stream: List[TimedMsg]) -> List[float]:
    return [x.t for x in stream]


def latest_before(stream: List[TimedMsg], t: float) -> Optional[Any]:
    if not stream:
        return None
    ts = _times(stream)
    idx = bisect.bisect_right(ts, t) - 1
    if idx < 0:
        return None
    return stream[idx].msg


def extract_twist(msg: Any) -> Tuple[float, float, float, float, float, float]:
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


def extract_actuator_states(msg: Any, num_actuators: int = 7) -> np.ndarray:
    result = np.zeros(num_actuators, dtype=np.float32)
    if msg is None:
        return result
    try:
        if hasattr(msg, "actuations"):
            act = np.asarray(msg.actuations, dtype=np.float32).flatten()
            n = min(len(act), num_actuators)
            result[:n] = act[:n]
    except Exception:
        pass
    return result


def extract_manus_ergonomics(msg: Any, num_values: int = 20) -> np.ndarray:
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


def try_decode_image_ros(msg: Any) -> Optional[np.ndarray]:
    """Decode ROS image to RGB numpy array [H, W, 3]."""
    if hasattr(msg, "format") and hasattr(msg, "data"):
        data = msg.data
        if isinstance(data, (bytes, bytearray, memoryview)):
            try:
                im = Image.open(io.BytesIO(bytes(data))).convert("RGB")
                return np.asarray(im, dtype=np.uint8)
            except Exception:
                return None

    if not all(hasattr(msg, attr) for attr in ["height", "width", "encoding", "data"]):
        return None

    H, W = int(msg.height), int(msg.width)
    enc = str(msg.encoding).lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "rgb8" and buf.size == H * W * 3:
        return buf.reshape(H, W, 3).copy()
    if enc == "bgr8" and buf.size == H * W * 3:
        return buf.reshape(H, W, 3)[:, :, ::-1].copy()
    if enc == "mono8" and buf.size == H * W:
        g = buf.reshape(H, W)
        return np.stack([g, g, g], axis=-1)

    return None


# -----------------------------------------------------------------------------
# Episode Segmentation
# -----------------------------------------------------------------------------

@dataclass
class EpisodeBoundary:
    """Represents episode start/end times within a rosbag."""
    start_time: float
    end_time: float
    label: str = ""


def segment_episodes_by_time_gap(
    collected: Dict[str, List[TimedMsg]],
    gap_threshold: float = 2.0,
) -> List[EpisodeBoundary]:
    """
    Segment episodes based on time gaps in the data.
    If there's a gap > gap_threshold seconds, it's an episode boundary.
    """
    all_times = []
    for stream in collected.values():
        all_times.extend([m.t for m in stream])
    
    if not all_times:
        return []
    
    all_times = sorted(set(all_times))
    
    if len(all_times) < 2:
        return [EpisodeBoundary(all_times[0], all_times[0])]
    
    episodes = []
    ep_start = all_times[0]
    
    for i in range(1, len(all_times)):
        gap = all_times[i] - all_times[i-1]
        if gap > gap_threshold:
            episodes.append(EpisodeBoundary(ep_start, all_times[i-1]))
            ep_start = all_times[i]
    
    episodes.append(EpisodeBoundary(ep_start, all_times[-1]))
    return episodes


def segment_episodes_by_joint_home(
    collected: Dict[str, List[TimedMsg]],
    topic_joint_states: str,
    home_position: np.ndarray = None,
    threshold: float = 0.1,
    min_episode_duration: float = 3.0,
) -> List[EpisodeBoundary]:
    """
    Segment episodes based on arm returning to home position.
    """
    joint_stream = collected.get(topic_joint_states, [])
    if not joint_stream:
        return []
    
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
                    if tm.t - ep_start > min_episode_duration:
                        episodes.append(EpisodeBoundary(ep_start, at_home_start))
                    ep_start = tm.t
                    at_home_start = None
    
    if joint_stream[-1].t - ep_start > min_episode_duration:
        episodes.append(EpisodeBoundary(ep_start, joint_stream[-1].t))
    
    return episodes


def segment_episodes_by_audio(
    collected: Dict[str, List[TimedMsg]],
    topic_audio: str = "/audio",
) -> List[EpisodeBoundary]:
    """
    [PLACEHOLDER] Segment episodes based on audio commands.
    
    TODO: Integrate speech recognition (whisper, vosk) to detect:
    - "start" / "begin" -> episode start
    - "stop" / "done" / "end" -> episode end
    """
    print("  [Audio segmentation not yet implemented - using single episode]")
    return []


def detect_episode_boundaries(
    collected: Dict[str, List[TimedMsg]],
    method: str = "none",
    config: Dict[str, Any] = None,
) -> List[EpisodeBoundary]:
    """
    Detect episode boundaries within a rosbag.
    
    Args:
        method: "none", "time_gap", "joint_home", "audio"
        config: Method-specific configuration
    """
    if config is None:
        config = {}
    
    if method == "none":
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
        return segment_episodes_by_joint_home(
            collected, topic, threshold=threshold, min_episode_duration=min_duration
        )
    
    elif method == "audio":
        return segment_episodes_by_audio(collected)
    
    else:
        print(f"  Warning: Unknown segmentation method '{method}', using 'none'")
        return detect_episode_boundaries(collected, "none", config)


# -----------------------------------------------------------------------------
# Rosbag Reading
# -----------------------------------------------------------------------------

def read_rosbag_topics(
    path: Path,
    topics: Sequence[str],
    debug_topics: bool = False,
) -> Dict[str, List[TimedMsg]]:
    collected: Dict[str, List[TimedMsg]] = {tp: [] for tp in topics}
    
    alias_map: Dict[str, str] = {}
    for tp in topics:
        base = tp.lstrip("/")
        alias_map[tp] = tp
        alias_map[base] = tp
        alias_map[f"/{base}"] = tp
    
    typestore = create_typestore_with_custom_msgs()
    available_topics: List[str] = []
    
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
                except Exception:
                    continue
    except Exception as e:
        print(f"Error reading {path}: {e}")
        return collected
    
    for tp in collected:
        collected[tp].sort(key=lambda x: x.t)
    
    if debug_topics or all(len(v) == 0 for v in collected.values()):
        print(f"  Available topics: {sorted(available_topics)}")
        counts = {k: len(v) for k, v in collected.items()}
        print(f"  Collected counts: {counts}")
    
    return collected


# -----------------------------------------------------------------------------
# Frame Building
# -----------------------------------------------------------------------------

def build_frames_for_episode(
    collected: Dict[str, List[TimedMsg]],
    fps: int,
    config: Dict[str, Any],
    time_range: Optional[Tuple[float, float]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, np.ndarray]]]:
    """
    Build frames for one episode.
    
    Returns:
        (data_frames, image_frames)
        - data_frames: list of dicts with state, action, timestamp
        - image_frames: list of dicts with {camera_name: rgb_array}
    """
    if time_range is not None:
        t0, t1 = time_range
    else:
        all_ts = []
        for st in collected.values():
            all_ts.extend([x.t for x in st])
        if not all_ts:
            return [], []
        t0, t1 = min(all_ts), max(all_ts)
    
    if t1 <= t0:
        return [], []
    
    dt = 1.0 / float(fps)
    times = np.arange(t0, t1 + 1e-9, dt, dtype=np.float64)
    
    data_frames = []
    image_frames = []
    
    # Config
    use_video = config.get("use_video", False)
    camera_topics = config.get("camera_topics", {})
    H, W = config.get("image_hw", (480, 640))
    
    arm_joint_dim = config.get("arm_joint_dim", 7)
    hand_joint_dim = config.get("hand_joint_dim", 16)
    num_actuators = config.get("num_actuators", 7)
    manus_dim = config.get("manus_dim", 20)
    
    last_imgs: Dict[str, Optional[np.ndarray]] = {name: None for name in camera_topics}
    
    for i, ti in enumerate(times):
        timestamp = float(ti - t0)  # Relative timestamp from episode start
        
        # Extract data from topics
        topic_cmd = config.get("topic_cmd_vel", "/spacemouse/cmd_vel")
        cmd_msg = latest_before(collected.get(topic_cmd, []), float(ti))
        vx, vy, vz, wx, wy, wz = extract_twist(cmd_msg)
        
        topic_arm = config.get("topic_joint_states", "/joint_states")
        arm_msg = latest_before(collected.get(topic_arm, []), float(ti))
        arm_joints = extract_joint_state(arm_msg, arm_joint_dim)
        
        topic_hand_ctrl = config.get("topic_hand_control", "/right/joint_control")
        hand_ctrl_msg = latest_before(collected.get(topic_hand_ctrl, []), float(ti))
        hand_target = extract_joint_control(hand_ctrl_msg, hand_joint_dim)
        
        topic_hand_state = config.get("topic_actuator_states", "/right/actuator_states")
        actuator_msg = latest_before(collected.get(topic_hand_state, []), float(ti))
        actuations = extract_actuator_states(actuator_msg, num_actuators)
        
        topic_manus = config.get("topic_manus", "/manus_glove_0")
        manus_msg = latest_before(collected.get(topic_manus, []), float(ti))
        manus_ergo = extract_manus_ergonomics(manus_msg, manus_dim)
        
        # Build state: [arm_joints(7), hand_actuations(7), manus_ergo(20)] = 34
        state = np.concatenate([
            arm_joints,
            actuations,
            manus_ergo,
        ], axis=0).astype(np.float32)
        
        # Build action: [base_cmd(3), arm_joints(7), hand_target(16)] = 26
        action = np.concatenate([
            np.array([vx, vy, wz], dtype=np.float32),
            arm_joints,
            hand_target,
        ], axis=0).astype(np.float32)
        
        data_frames.append({
            "observation.state": state.tolist(),
            "action": action.tolist(),
            "timestamp": timestamp,
        })
        
        # Images
        if use_video and camera_topics:
            frame_imgs = {}
            for cam_name, topic in camera_topics.items():
                img_msg = latest_before(collected.get(topic, []), float(ti))
                rgb = None
                if img_msg is not None:
                    rgb = try_decode_image_ros(img_msg)
                    if rgb is not None and (rgb.shape[0] != H or rgb.shape[1] != W):
                        rgb = np.asarray(
                            Image.fromarray(rgb).resize((W, H), resample=Image.BILINEAR),
                            dtype=np.uint8,
                        )
                
                if rgb is None:
                    rgb = last_imgs.get(cam_name)
                if rgb is None:
                    rgb = np.zeros((H, W, 3), dtype=np.uint8)
                
                last_imgs[cam_name] = rgb
                frame_imgs[cam_name] = rgb
            
            image_frames.append(frame_imgs)
    
    return data_frames, image_frames


# -----------------------------------------------------------------------------
# Video Encoding
# -----------------------------------------------------------------------------

def encode_video_ffmpeg(
    frames: List[np.ndarray],
    output_path: Path,
    fps: int,
) -> bool:
    """Encode frames to MP4 using ffmpeg."""
    if not frames:
        return False
    
    H, W = frames[0].shape[:2]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Use ffmpeg to encode
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo",
        "-vcodec", "rawvideo",
        "-s", f"{W}x{H}",
        "-pix_fmt", "rgb24",
        "-r", str(fps),
        "-i", "-",
        "-c:v", "libx264",
        "-preset", "fast",
        "-crf", "23",
        "-pix_fmt", "yuv420p",
        str(output_path)
    ]
    
    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        
        for frame in frames:
            proc.stdin.write(frame.tobytes())
        
        proc.stdin.close()
        proc.wait()
        
        return proc.returncode == 0
    except Exception as e:
        print(f"  Video encoding error: {e}")
        return False


# -----------------------------------------------------------------------------
# Meta File Generation
# -----------------------------------------------------------------------------

def create_modality_json(
    arm_joint_dim: int,
    hand_joint_dim: int,
    num_actuators: int,
    manus_dim: int,
    camera_names: List[str],
) -> Dict[str, Any]:
    """Create the modality.json configuration for GR00T."""
    
    # State fields: [arm_joints(7), hand_actuations(7), manus_ergo(20)] = 34
    state_idx = 0
    state_fields = {}
    
    state_fields["arm_joint_positions"] = {"start": state_idx, "end": state_idx + arm_joint_dim}
    state_idx += arm_joint_dim
    
    state_fields["hand_actuator_positions"] = {"start": state_idx, "end": state_idx + num_actuators}
    state_idx += num_actuators
    
    state_fields["manus_glove_ergonomics"] = {"start": state_idx, "end": state_idx + manus_dim}
    state_idx += manus_dim
    
    # Action fields: [base_cmd(3), arm_joints(7), hand_target(16)] = 26
    action_idx = 0
    action_fields = {}
    
    action_fields["base_velocity"] = {"start": action_idx, "end": action_idx + 3}
    action_idx += 3
    
    action_fields["arm_joint_targets"] = {"start": action_idx, "end": action_idx + arm_joint_dim}
    action_idx += arm_joint_dim
    
    action_fields["hand_joint_targets"] = {"start": action_idx, "end": action_idx + hand_joint_dim}
    action_idx += hand_joint_dim
    
    # Video fields
    video_fields = {}
    for cam_name in camera_names:
        video_fields[f"observation.images.{cam_name}"] = {
            "original_key": f"observation.images.{cam_name}"
        }
    
    # Annotation fields
    annotation_fields = {
        "annotation.human.action.task_description": {},
        "annotation.human.validity": {},
    }
    
    return {
        "state": state_fields,
        "action": action_fields,
        "video": video_fields,
        "annotation": annotation_fields,
    }


def create_info_json(
    repo_id: str,
    fps: int,
    total_episodes: int,
    total_frames: int,
    state_dim: int,
    action_dim: int,
    robot_type: str,
) -> Dict[str, Any]:
    """Create the info.json metadata."""
    return {
        "codebase_version": "v2.0",
        "robot_type": robot_type,
        "fps": fps,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "data_path": "data/chunk-000",
        "video_path": "videos/chunk-000",
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [state_dim],
            },
            "action": {
                "dtype": "float32",
                "shape": [action_dim],
            },
        },
        "repo_id": repo_id,
    }


# -----------------------------------------------------------------------------
# Main Conversion
# -----------------------------------------------------------------------------

def convert_rosbags_to_groot_lerobot(
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
    camera_topics: Dict[str, str],
    # Dimension configuration
    arm_joint_dim: int,
    hand_joint_dim: int,
    num_actuators: int,
    manus_dim: int,
    # Episode segmentation
    episode_segmentation: str = "none",
    episode_gap_threshold: float = 2.0,
    episode_min_duration: float = 3.0,
) -> Path:
    """Convert rosbag recordings to GR00T LeRobot v2 format."""
    
    if output_root.exists():
        if overwrite:
            shutil.rmtree(output_root)
        else:
            raise FileExistsError(f"Output exists: {output_root} (use --overwrite)")
    
    # Create directory structure
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "meta").mkdir()
    (output_root / "data" / "chunk-000").mkdir(parents=True)
    if use_video:
        (output_root / "videos" / "chunk-000").mkdir(parents=True)
        for cam_name in camera_topics:
            (output_root / "videos" / "chunk-000" / f"observation.images.{cam_name}").mkdir()
    
    # Calculate dimensions
    state_dim = arm_joint_dim + num_actuators + manus_dim  # 34
    action_dim = 3 + arm_joint_dim + hand_joint_dim  # 26
    
    print(f"Creating GR00T LeRobot v2 dataset:")
    print(f"  State dim: {state_dim} = arm({arm_joint_dim}) + actuators({num_actuators}) + manus({manus_dim})")
    print(f"  Action dim: {action_dim} = base_cmd(3) + arm({arm_joint_dim}) + hand({hand_joint_dim})")
    print(f"  Cameras: {list(camera_topics.keys()) if camera_topics else 'none'}")
    
    # Topics to read
    topics = [
        topic_cmd_vel,
        topic_joint_states,
        topic_hand_control,
        topic_actuator_states,
        topic_manus,
    ]
    if use_video and camera_topics:
        topics.extend(camera_topics.values())
    
    # Config for frame building
    config = {
        "use_video": use_video,
        "camera_topics": camera_topics,
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
        "episode_segmentation": episode_segmentation,
        "gap_threshold": episode_gap_threshold,
        "min_episode_duration": episode_min_duration,
    }
    
    # Find rosbags
    bags = []
    if (input_dir / "metadata.yaml").exists():
        bags.append(input_dir)
    else:
        for item in sorted(input_dir.iterdir()):
            if item.is_dir() and (item / "metadata.yaml").exists():
                bags.append(item)
    
    if not bags:
        raise SystemExit(f"No rosbags found in: {input_dir}")
    
    print(f"Found {len(bags)} rosbag(s)")
    
    # Episode segmentation config
    episode_method = config.get("episode_segmentation", "none")
    print(f"Episode segmentation: {episode_method}")
    
    # Process bags
    episodes_info = []
    tasks_info = [
        {"task_index": 0, "task": task},
        {"task_index": 1, "task": "valid"},
    ]
    
    global_index = 0
    episode_index = 0
    
    for bag_idx, bag_path in enumerate(bags):
        print(f"\n[Bag {bag_idx+1}/{len(bags)}] {bag_path.name}")
        
        collected = read_rosbag_topics(bag_path, topics, debug_topics=debug_topics)
        
        # Detect episode boundaries within this bag
        seg_config = {
            "gap_threshold": config.get("gap_threshold", 2.0),
            "min_episode_duration": config.get("min_episode_duration", 3.0),
            "topic_joint_states": topic_joint_states,
        }
        episode_boundaries = detect_episode_boundaries(collected, episode_method, seg_config)
        
        if not episode_boundaries:
            print("  -> No episodes detected, skipping")
            continue
        
        print(f"  -> Detected {len(episode_boundaries)} episode(s) in this bag")
        
        for ep_in_bag, boundary in enumerate(episode_boundaries):
            duration = boundary.end_time - boundary.start_time
            print(f"    Episode {ep_in_bag+1}: {duration:.1f}s")
            
            data_frames, image_frames = build_frames_for_episode(
                collected, fps, config,
                time_range=(boundary.start_time, boundary.end_time)
            )
            
            if not data_frames:
                print(f"      -> No frames, skipping")
                continue
            
            episode_length = len(data_frames)
            print(f"      -> Processing {episode_length} frames")
            
            # Build parquet data
            parquet_data = {
                "observation.state": [],
                "action": [],
                "timestamp": [],
                "annotation.human.action.task_description": [],
                "annotation.human.validity": [],
                "task_index": [],
                "episode_index": [],
                "index": [],
                "next.reward": [],
                "next.done": [],
            }
            
            for i, frame in enumerate(data_frames):
                parquet_data["observation.state"].append(frame["observation.state"])
                parquet_data["action"].append(frame["action"])
                parquet_data["timestamp"].append(frame["timestamp"])
                parquet_data["annotation.human.action.task_description"].append(0)  # task index
                parquet_data["annotation.human.validity"].append(1)  # valid index
                parquet_data["task_index"].append(0)
                parquet_data["episode_index"].append(episode_index)
                parquet_data["index"].append(global_index)
                parquet_data["next.reward"].append(0.0)
                parquet_data["next.done"].append(i == episode_length - 1)
                global_index += 1
            
            # Write parquet file
            parquet_path = output_root / "data" / "chunk-000" / f"episode_{episode_index:06d}.parquet"
            
            table = pa.table({
                "observation.state": pa.array(parquet_data["observation.state"], type=pa.list_(pa.float32())),
                "action": pa.array(parquet_data["action"], type=pa.list_(pa.float32())),
                "timestamp": pa.array(parquet_data["timestamp"], type=pa.float64()),
                "annotation.human.action.task_description": pa.array(parquet_data["annotation.human.action.task_description"], type=pa.int64()),
                "annotation.human.validity": pa.array(parquet_data["annotation.human.validity"], type=pa.int64()),
                "task_index": pa.array(parquet_data["task_index"], type=pa.int64()),
                "episode_index": pa.array(parquet_data["episode_index"], type=pa.int64()),
                "index": pa.array(parquet_data["index"], type=pa.int64()),
                "next.reward": pa.array(parquet_data["next.reward"], type=pa.float64()),
                "next.done": pa.array(parquet_data["next.done"], type=pa.bool_()),
            })
            
            pq.write_table(table, parquet_path)
            print(f"      -> Wrote parquet: {parquet_path.name}")
            
            # Write video files
            if use_video and image_frames:
                for cam_name in camera_topics:
                    video_frames = [f[cam_name] for f in image_frames]
                    video_path = (output_root / "videos" / "chunk-000" / 
                                 f"observation.images.{cam_name}" / f"episode_{episode_index:06d}.mp4")
                    
                    if encode_video_ffmpeg(video_frames, video_path, fps):
                        print(f"      -> Wrote video: {video_path.name}")
                    else:
                        print(f"      -> Video encoding failed for {cam_name}")
            
            # Track episode info
            episodes_info.append({
                "episode_index": episode_index,
                "tasks": [task],
                "length": episode_length,
            })
            
            episode_index += 1
    
    # Write meta files
    print("\nWriting meta files...")
    
    # episodes.jsonl
    with open(output_root / "meta" / "episodes.jsonl", "w") as f:
        for ep in episodes_info:
            f.write(json.dumps(ep) + "\n")
    
    # tasks.jsonl
    with open(output_root / "meta" / "tasks.jsonl", "w") as f:
        for task_info in tasks_info:
            f.write(json.dumps(task_info) + "\n")
    
    # modality.json
    modality = create_modality_json(
        arm_joint_dim, hand_joint_dim, num_actuators, manus_dim,
        list(camera_topics.keys()) if use_video else []
    )
    with open(output_root / "meta" / "modality.json", "w") as f:
        json.dump(modality, f, indent=2)
    
    # info.json
    info = create_info_json(
        repo_id=repo_id,
        fps=fps,
        total_episodes=episode_index,
        total_frames=global_index,
        state_dim=state_dim,
        action_dim=action_dim,
        robot_type=robot_type,
    )
    with open(output_root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)
    
    print(f"\nDone! Dataset saved to: {output_root}")
    print(f"Total episodes: {episode_index}")
    print(f"Total frames: {global_index}")
    
    return output_root


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS2 rosbag to GR00T LeRobot v2 format"
    )
    
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--repo-id", type=str, required=True)
    parser.add_argument("--robot-type", type=str, default="aero_hand")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--task", type=str, default="hand teleop demonstration")
    
    parser.add_argument("--use-video", action="store_true")
    parser.add_argument("--image-h", type=int, default=480)
    parser.add_argument("--image-w", type=int, default=640)
    
    parser.add_argument("--topic-cmd-vel", type=str, default="/spacemouse/cmd_vel")
    parser.add_argument("--topic-joint-states", type=str, default="/joint_states")
    parser.add_argument("--topic-hand-control", type=str, default="/right/joint_control")
    parser.add_argument("--topic-actuator-states", type=str, default="/right/actuator_states")
    parser.add_argument("--topic-manus", type=str, default="/manus_glove_0")
    
    parser.add_argument("--camera-head", type=str, default="/camera_0/color")
    parser.add_argument("--camera-wrist", type=str, default=None)
    parser.add_argument("--camera-base", type=str, default=None)
    
    parser.add_argument("--arm-joint-dim", type=int, default=7)
    parser.add_argument("--hand-joint-dim", type=int, default=16)
    parser.add_argument("--num-actuators", type=int, default=7)
    parser.add_argument("--manus-dim", type=int, default=20)
    
    # Episode segmentation
    parser.add_argument(
        "--episode-segmentation", type=str, default="none",
        choices=["none", "time_gap", "joint_home", "audio"],
        help="Method to segment multiple episodes within each rosbag"
    )
    parser.add_argument("--episode-gap-threshold", type=float, default=2.0,
                       help="Time gap threshold (seconds) for time_gap segmentation")
    parser.add_argument("--episode-min-duration", type=float, default=3.0,
                       help="Minimum episode duration (seconds)")
    
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--debug-topics", action="store_true")
    
    args = parser.parse_args()
    
    # Build camera topics
    camera_topics = {}
    if args.use_video:
        if args.camera_head:
            camera_topics["head"] = args.camera_head
        if args.camera_wrist:
            camera_topics["wrist"] = args.camera_wrist
        if args.camera_base:
            camera_topics["base"] = args.camera_base
    
    convert_rosbags_to_groot_lerobot(
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
        episode_segmentation=args.episode_segmentation,
        episode_gap_threshold=args.episode_gap_threshold,
        episode_min_duration=args.episode_min_duration,
    )


if __name__ == "__main__":
    main()
