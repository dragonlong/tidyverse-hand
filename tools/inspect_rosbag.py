#!/usr/bin/env python3
"""
Inspect and visualize ROS2 rosbag data.

This tool helps you:
- List all topics and message counts
- Sample messages from specific topics
- Visualize time series data
- Extract and save images

Usage:
    # Source ROS2 workspace first:
    cd ~/tetheria/aero-open-ros2
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # Run inspection:
    python inspect_rosbag.py --input /path/to/rosbag_folder
    python inspect_rosbag.py --input /path/to/rosbag_folder --topic /right/joint_control --sample 5
    python inspect_rosbag.py --input /path/to/rosbag_folder --plot-joints
    python inspect_rosbag.py --input /path/to/rosbag_folder --extract-images --output-dir ./images
"""

from __future__ import annotations

import argparse
import io
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# rosbags for reading
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def create_typestore_with_custom_msgs():
    """Create a typestore with ROS2 Humble types and custom message definitions."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # Custom message definitions for aero_hand_open_msgs
    custom_msgs = {
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
        # Manus glove messages
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
    
    # Register custom messages (order matters for dependencies)
    for msgtype in ["manus_ros2_msgs/msg/ManusRawNode", "manus_ros2_msgs/msg/ManusErgonomics"]:
        try:
            types = get_types_from_msg(custom_msgs[msgtype].strip(), msgtype)
            typestore.register(types)
        except Exception as e:
            print(f"Warning: Could not register {msgtype}: {e}")
    
    for msgtype, msgdef in custom_msgs.items():
        if msgtype.startswith("manus_ros2_msgs/msg/Manus") and msgtype not in ["manus_ros2_msgs/msg/ManusRawNode", "manus_ros2_msgs/msg/ManusErgonomics"]:
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception as e:
                print(f"Warning: Could not register {msgtype}: {e}")
    
    for msgtype, msgdef in custom_msgs.items():
        if msgtype.startswith("aero_hand_open_msgs"):
            try:
                types = get_types_from_msg(msgdef.strip(), msgtype)
                typestore.register(types)
            except Exception as e:
                print(f"Warning: Could not register {msgtype}: {e}")
    
    return typestore


@dataclass
class TopicInfo:
    """Information about a topic."""
    name: str
    msgtype: str
    count: int
    first_time: Optional[float] = None
    last_time: Optional[float] = None


def get_topic_info(bag_path: Path) -> Dict[str, TopicInfo]:
    """Get information about all topics in the bag."""
    topics: Dict[str, TopicInfo] = {}
    typestore = create_typestore_with_custom_msgs()
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        # Get topic list and counts from metadata
        for topic_name, topic_info in reader.topics.items():
            topics[topic_name] = TopicInfo(
                name=topic_name,
                msgtype=topic_info.msgtype,
                count=topic_info.msgcount,
            )
        
        # Get first and last timestamps for each topic
        first_times: Dict[str, float] = {}
        last_times: Dict[str, float] = {}
        
        for conn, t_ns, _ in reader.messages():
            t = float(t_ns) * 1e-9
            if conn.topic not in first_times:
                first_times[conn.topic] = t
            last_times[conn.topic] = t
        
        for name, info in topics.items():
            info.first_time = first_times.get(name)
            info.last_time = last_times.get(name)
    
    return topics


def sample_messages(bag_path: Path, topic: str, n: int = 5, typestore=None) -> List[Any]:
    """Sample n messages from a topic."""
    messages = []
    count = 0
    
    if typestore is None:
        typestore = create_typestore_with_custom_msgs()
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for conn, t_ns, raw in reader.messages():
            if conn.topic == topic or conn.topic == topic.lstrip("/") or conn.topic == "/" + topic.lstrip("/"):
                try:
                    msg = reader.deserialize(raw, conn.msgtype)
                    messages.append({
                        "time": float(t_ns) * 1e-9,
                        "data": msg_to_dict(msg),
                    })
                    count += 1
                    if count >= n:
                        break
                except Exception as e:
                    messages.append({
                        "time": float(t_ns) * 1e-9,
                        "error": str(e),
                    })
                    count += 1
                    if count >= n:
                        break
    
    return messages


def msg_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert a ROS message to a dictionary."""
    result = {}
    
    # Get all attributes that don't start with underscore
    for attr in dir(msg):
        if attr.startswith("_"):
            continue
        try:
            value = getattr(msg, attr)
            if callable(value):
                continue
            
            # Handle numpy arrays
            if isinstance(value, np.ndarray):
                result[attr] = value.tolist()
            # Handle nested messages
            elif hasattr(value, "__slots__"):
                result[attr] = msg_to_dict(value)
            # Handle lists of messages
            elif isinstance(value, (list, tuple)):
                if len(value) > 0 and hasattr(value[0], "__slots__"):
                    result[attr] = [msg_to_dict(v) for v in value[:5]]  # Limit nested lists
                    if len(value) > 5:
                        result[attr].append(f"... ({len(value)} total)")
                else:
                    result[attr] = list(value)[:20]  # Limit array display
                    if len(value) > 20:
                        result[attr].append(f"... ({len(value)} total)")
            else:
                result[attr] = value
        except Exception:
            continue
    
    return result


def plot_joint_data(bag_path: Path, output_path: Optional[Path] = None):
    """Plot joint control and actuator data over time."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed. Run: pip install matplotlib")
        return
    
    # Collect data
    joint_control_times = []
    joint_control_data = []
    actuator_times = []
    actuator_data = []
    typestore = create_typestore_with_custom_msgs()
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for conn, t_ns, raw in reader.messages():
            t = float(t_ns) * 1e-9
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                
                if "joint_control" in conn.topic:
                    if hasattr(msg, "target_positions"):
                        joint_control_times.append(t)
                        joint_control_data.append(np.array(msg.target_positions, dtype=np.float32))
                
                elif "actuator_states" in conn.topic:
                    if hasattr(msg, "actuations"):
                        actuator_times.append(t)
                        actuator_data.append(np.array(msg.actuations, dtype=np.float32))
            except Exception:
                continue
    
    if not joint_control_times and not actuator_times:
        print("No joint data found")
        return
    
    # Create plots
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # Plot joint control targets
    if joint_control_times:
        t0 = joint_control_times[0]
        times_rel = [t - t0 for t in joint_control_times]
        data = np.array(joint_control_data)
        
        ax = axes[0]
        for i in range(min(data.shape[1], 16)):
            ax.plot(times_rel, data[:, i], label=f"j{i}", alpha=0.7)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title("Hand Joint Control Targets")
        ax.legend(loc="upper right", ncol=4, fontsize=8)
        ax.grid(True, alpha=0.3)
    
    # Plot actuator states
    if actuator_times:
        t0 = actuator_times[0]
        times_rel = [t - t0 for t in actuator_times]
        data = np.array(actuator_data)
        
        actuator_names = [
            "thumb_abd", "thumb_flex", "thumb_tendon",
            "index", "middle", "ring", "pinky"
        ]
        
        ax = axes[1]
        for i in range(min(data.shape[1], 7)):
            label = actuator_names[i] if i < len(actuator_names) else f"act{i}"
            ax.plot(times_rel, data[:, i], label=label, alpha=0.7)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Actuation (deg)")
        ax.set_title("Hand Actuator States")
        ax.legend(loc="upper right", ncol=4, fontsize=8)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Saved plot to: {output_path}")
    else:
        plt.show()


def extract_images(
    bag_path: Path,
    topic: str,
    output_dir: Path,
    max_images: int = 100,
    interval: int = 1,
):
    """Extract images from an image topic."""
    from PIL import Image
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    count = 0
    saved = 0
    
    typestore = create_typestore_with_custom_msgs()
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for conn, t_ns, raw in reader.messages():
            if topic not in conn.topic:
                continue
            
            if count % interval != 0:
                count += 1
                continue
            
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                
                # Try to decode image
                img = None
                
                # CompressedImage
                if hasattr(msg, "format") and hasattr(msg, "data"):
                    img = Image.open(io.BytesIO(bytes(msg.data))).convert("RGB")
                
                # Raw Image
                elif hasattr(msg, "height") and hasattr(msg, "width") and hasattr(msg, "data"):
                    H, W = int(msg.height), int(msg.width)
                    enc = str(msg.encoding).lower()
                    buf = np.frombuffer(msg.data, dtype=np.uint8)
                    
                    if enc == "rgb8" and buf.size == H * W * 3:
                        img = Image.fromarray(buf.reshape(H, W, 3))
                    elif enc == "bgr8" and buf.size == H * W * 3:
                        img = Image.fromarray(buf.reshape(H, W, 3)[:, :, ::-1])
                
                if img:
                    filename = f"frame_{saved:06d}.jpg"
                    img.save(output_dir / filename)
                    saved += 1
                    
                    if saved >= max_images:
                        break
                    
            except Exception as e:
                print(f"Error extracting frame {count}: {e}")
            
            count += 1
    
    print(f"Extracted {saved} images to {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Inspect ROS2 rosbag data")
    
    parser.add_argument(
        "--input", "-i", type=Path, required=True,
        help="Path to rosbag2 directory or .mcap file"
    )
    
    # Information commands
    parser.add_argument(
        "--list-topics", "-l", action="store_true",
        help="List all topics with counts and types"
    )
    
    parser.add_argument(
        "--topic", "-t", type=str,
        help="Topic to inspect"
    )
    parser.add_argument(
        "--sample", "-n", type=int, default=3,
        help="Number of messages to sample (default: 3)"
    )
    
    # Visualization
    parser.add_argument(
        "--plot-joints", action="store_true",
        help="Plot joint control and actuator data"
    )
    parser.add_argument(
        "--plot-output", type=Path,
        help="Output path for plot (shows interactively if not specified)"
    )
    
    # Image extraction
    parser.add_argument(
        "--extract-images", action="store_true",
        help="Extract images from camera topic"
    )
    parser.add_argument(
        "--image-topic", type=str, default="/camera_0/color",
        help="Image topic to extract from"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("./extracted_images"),
        help="Output directory for extracted images"
    )
    parser.add_argument(
        "--max-images", type=int, default=100,
        help="Maximum number of images to extract"
    )
    parser.add_argument(
        "--image-interval", type=int, default=1,
        help="Extract every Nth image"
    )
    
    args = parser.parse_args()
    
    # Default: list topics
    if not any([args.topic, args.plot_joints, args.extract_images]):
        args.list_topics = True
    
    # List topics
    if args.list_topics:
        print(f"\nRosbag: {args.input}")
        print("=" * 80)
        
        topics = get_topic_info(args.input)
        
        # Calculate duration
        all_times = []
        for info in topics.values():
            if info.first_time is not None:
                all_times.append(info.first_time)
            if info.last_time is not None:
                all_times.append(info.last_time)
        
        if all_times:
            duration = max(all_times) - min(all_times)
            print(f"Duration: {duration:.2f} seconds")
        
        print(f"\nTopics ({len(topics)}):")
        print("-" * 80)
        
        for name in sorted(topics.keys()):
            info = topics[name]
            freq_str = ""
            if info.first_time and info.last_time and info.count > 1:
                dur = info.last_time - info.first_time
                if dur > 0:
                    freq = info.count / dur
                    freq_str = f" ({freq:.1f} Hz)"
            print(f"  {info.count:>6} msgs  {name:<40} {info.msgtype}{freq_str}")
    
    # Sample messages from topic
    if args.topic:
        print(f"\nSampling {args.sample} messages from {args.topic}:")
        print("-" * 80)
        
        messages = sample_messages(args.input, args.topic, args.sample)
        
        for i, msg in enumerate(messages):
            print(f"\n[{i+1}] t={msg['time']:.6f}")
            if "error" in msg:
                print(f"  Error: {msg['error']}")
            else:
                # Pretty print the message data
                print(json.dumps(msg["data"], indent=2, default=str)[:2000])
    
    # Plot joint data
    if args.plot_joints:
        plot_joint_data(args.input, args.plot_output)
    
    # Extract images
    if args.extract_images:
        extract_images(
            args.input,
            args.image_topic,
            args.output_dir,
            args.max_images,
            args.image_interval,
        )


if __name__ == "__main__":
    main()
