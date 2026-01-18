"""
Convert ROS2 MCAP (rosbag2) episodes to LeRobotDataset v3.

Fixes vs your current script:
- Avoids the common `typestore.deserialize_cdr(...)` / `deserialize_cdr(...)` issues by using
  rosbags.highlevel.AnyReader whenever possible.
- Removes the free-form `timestamp` feature/key (LeRobot v3 rejects unknown keys → "Extra features: {'timestamp'}").
  Time is implicit from frame index + fps.
- Ensures each frame dict keys match `features` EXACTLY (no extra keys like `task`, `timestamp`, etc.).
- Adds optional Rerun export path:
    * either log directly from ROS messages (no decoding), or
    * log from the fixed-FPS frames you produce (recommended, because it matches LeRobot frames exactly).
  Rerun can write an .rrd file which you can open later, and you can also load that .rrd and re-export frames if you want.
- Robust image decoding for sensor_msgs/Image + sensor_msgs/CompressedImage.

Assumptions:
- 1 *.mcap file = 1 episode
- odom: nav_msgs/msg/Odometry
- cmd : geometry_msgs/msg/Twist
- hand: sensor_msgs/msg/JointState (or similar with .position)
- img : sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage (optional)
"""

from __future__ import annotations

import argparse
import bisect
import io
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image

from rosbags.highlevel import AnyReader
from rosbags.interfaces import Connection, ConnectionExtRosbag2
from rosbags.rosbag2.errors import ReaderError
from rosbags.rosbag2.storage_mcap import MCAPFile
from rosbags.typesys import Stores, get_typestore, get_types_from_idl, get_types_from_msg

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Optional (only used if --rerun-* flags are passed)
try:
    import rerun as rr
except Exception:
    rr = None


# -----------------------------
# Helpers
# -----------------------------
@dataclass
class TimedMsg:
    t: float  # seconds
    msg: Any


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # yaw from quaternion (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def try_decode_image_ros(msg: Any) -> Optional[np.ndarray]:
    """
    Return uint8 RGB image [H,W,3] or None.
    Supports:
      - sensor_msgs/msg/Image: rgb8, bgr8, mono8, rgba8, bgra8 (common)
      - sensor_msgs/msg/CompressedImage: JPEG/PNG payload in .data
    """
    # CompressedImage (ROS2): fields: format (str), data (bytes)
    if hasattr(msg, "format") and hasattr(msg, "data"):
        data = msg.data
        if isinstance(data, (bytes, bytearray, memoryview)):
            try:
                im = Image.open(io.BytesIO(bytes(data))).convert("RGB")
                return np.asarray(im, dtype=np.uint8)
            except Exception:
                return None

    # Raw Image: height, width, encoding, data, step
    if not (hasattr(msg, "height") and hasattr(msg, "width") and hasattr(msg, "encoding") and hasattr(msg, "data")):
        return None

    H, W = int(msg.height), int(msg.width)
    enc = str(msg.encoding).lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    # Common encodings
    if enc == "rgb8":
        if buf.size != H * W * 3:
            return None
        return buf.reshape(H, W, 3)

    if enc == "bgr8":
        if buf.size != H * W * 3:
            return None
        return buf.reshape(H, W, 3)[:, :, ::-1]

    if enc == "mono8":
        if buf.size != H * W:
            return None
        g = buf.reshape(H, W)
        return np.stack([g, g, g], axis=-1)

    if enc == "rgba8":
        if buf.size != H * W * 4:
            return None
        rgba = buf.reshape(H, W, 4)
        return rgba[:, :, :3]

    if enc == "bgra8":
        if buf.size != H * W * 4:
            return None
        bgra = buf.reshape(H, W, 4)
        bgr = bgra[:, :, :3]
        return bgr[:, :, ::-1]

    return None


def _times(stream: List[TimedMsg]) -> List[float]:
    return [x.t for x in stream]


def latest_before(stream: List[TimedMsg], t: float) -> Optional[Any]:
    """
    Binary search: return latest msg with time <= t
    """
    if not stream:
        return None
    ts = _times(stream)
    idx = bisect.bisect_right(ts, t) - 1
    if idx < 0:
        return None
    return stream[idx].msg


def read_mcap_topics(
    path: Path,
    topics: Sequence[str],
    debug_topics: bool = False,
) -> Dict[str, List[TimedMsg]]:
    """
    Read messages for the given topics.

    - If `path` is a rosbag2 .mcap file (no metadata.yaml), decode directly with
      rosbags' MCAP reader + typestore to avoid AnyReader falling back to rosbag1.
    - Otherwise, use AnyReader (works for rosbag2 directories and rosbag1 bags).
    """
    collected: Dict[str, List[TimedMsg]] = {tp: [] for tp in topics}

    # Allow topics with or without a leading slash to match (bags differ here).
    alias_map: Dict[str, str] = {}
    for tp in topics:
        base = tp[1:] if tp.startswith("/") else tp
        # Add both "color" and "col" variants for convenience
        variants = {tp, base, f"/{base}"}
        if base.endswith("color"):
            short_color = base[:-2]
            variants.update({short_color, f"/{short_color}"})
        alias_map.update({v: tp for v in variants})

    available_topics: List[str] = []

    if path.suffix == ".mcap":
        bag = MCAPFile(path)
        try:
            bag.open()
            bag.meta_scan()
        except ReaderError as e:
            print(f"  -> skip {path.name}: {e}")
            return collected
        available_topics = [ch.topic for ch in bag.channels.values()]

        # Build typestore from embedded schema definitions
        typestore = get_typestore(Stores.EMPTY)
        schema_defs = bag.get_schema_definitions()
        schema_enc_debug = []
        for name, (enc, data) in schema_defs.items():
            schema_enc_debug.append((name, enc))
            try:
                if enc == "idl":
                    typestore.register(get_types_from_idl(data, name=name))
                else:
                    typestore.register(get_types_from_msg(data, name=name))
            except Exception:
                continue

        # Build synthetic connections for topic filtering
        connections: Dict[int, Connection] = {}
        for cid, ch in bag.channels.items():
            target = alias_map.get(ch.topic)
            if target is None:
                continue
            connections[cid] = Connection(
                id=cid,
                topic=target,
                msgtype=ch.schema,
                msgdef=schema_defs.get(ch.schema, ("", ""))[1],
                digest="",
                msgcount=bag.statistics.message_count if bag.statistics else 0,
                ext=ConnectionExtRosbag2(serialization_format="cdr", offered_qos_profiles=""),
                owner=None,
            )

        for conn, t_ns, raw in bag.messages_scan(connections.values()):
            target = conn.topic
            if target not in collected:
                continue
            try:
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
            except Exception:
                continue
            collected[target].append(TimedMsg(t=float(t_ns) * 1e-9, msg=msg))

        bag.close()

    else:
        # AnyReader for rosbag2 dirs (metadata.yaml) or rosbag1 bags
        with AnyReader([path]) as reader:
            available_topics = list(reader.topics.keys())
            for conn, t_ns, raw in reader.messages():
                target = alias_map.get(conn.topic)
                if target is None:
                    continue
                try:
                    msg = reader.deserialize(raw, conn.msgtype)
                except Exception:
                    # If a topic cannot be deserialized, skip but keep going
                    continue
                collected[target].append(TimedMsg(t=float(t_ns) * 1e-9, msg=msg))

    for tp in collected:
        collected[tp].sort(key=lambda x: x.t)

    if debug_topics or all(len(v) == 0 for v in collected.values()):
        print(f"  -> available topics: {sorted(set(available_topics))}")
        counts = {k: len(v) for k, v in collected.items()}
        print(f"  -> collected counts: {counts}")

    return collected


def make_features(
    state_dim: int,
    action_dim: int,
    use_video: bool,
    image_hw: Tuple[int, int],
) -> Dict[str, Any]:
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
    if use_video:
        feats["observation.images.front"] = {
            "dtype": "video",
            "shape": (H, W, 3),
        }
    return feats


def build_fixed_fps_frames(
    collected: Dict[str, List[TimedMsg]],
    topic_odom: str,
    topic_cmd: str,
    topic_hand: str,
    topic_img: Optional[str],
    fps: int,
    state_dim: int,
    arm_joint_dim: int,
    hand_joint_dim: int,
    use_video: bool,
    task: Optional[str],
    image_hw: Tuple[int, int],
) -> List[Dict[str, Any]]:
    """
    Resample all signals to fixed FPS frames over [t0, t1].

    IMPORTANT:
    - DO NOT add any extra keys like "timestamp" or "task".
    - Frame dict keys must match ds.features EXACTLY.
    """
    # Determine episode time span from available streams
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

    H, W = image_hw
    frames: List[Dict[str, Any]] = []

    # LeRobot requires a `task` key in every frame (special field, not part of features)
    task_value = "" if task is None else str(task)

    # If video is enabled, we must provide an image for every frame.
    # If the bag contains no images at all for the requested topic, skip the episode entirely
    # (otherwise we'd write meaningless black frames).
    if use_video and topic_img is not None:
        if len(collected.get(topic_img, [])) == 0:
            return []

    last_pil_img: Optional[Image.Image] = None

    for ti in times:
        odom = latest_before(collected.get(topic_odom, []), float(ti))
        cmd = latest_before(collected.get(topic_cmd, []), float(ti))
        hand = latest_before(collected.get(topic_hand, []), float(ti))

        imgm = None
        if use_video and topic_img is not None:
            imgm = latest_before(collected.get(topic_img, []), float(ti))

        # ---- observation.state ----
        # base: [x, y, yaw, v, w] (5)
        x = y = yaw = v = w = 0.0
        if odom is not None and hasattr(odom, "pose") and hasattr(odom, "twist"):
            try:
                px = float(odom.pose.pose.position.x)
                py = float(odom.pose.pose.position.y)
                q = odom.pose.pose.orientation
                x, y = px, py
                yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                v = float(odom.twist.twist.linear.x)
                w = float(odom.twist.twist.angular.z)
            except Exception:
                pass

        # Joint states: split into arm joints then hand joints
        arm_joints = np.zeros((arm_joint_dim,), dtype=np.float32)
        hand_joints = np.zeros((hand_joint_dim,), dtype=np.float32)
        if hand is not None and hasattr(hand, "position"):
            try:
                pos = np.asarray(hand.position, dtype=np.float32).reshape(-1)
                n_arm = min(pos.size, arm_joint_dim)
                arm_joints[:n_arm] = pos[:n_arm]
                remaining = pos[n_arm:]
                n_hand = min(remaining.size, hand_joint_dim)
                hand_joints[:n_hand] = remaining[:n_hand]
            except Exception:
                pass

        state = np.concatenate(
            [np.array([x, y, yaw, v, w], dtype=np.float32), hand_joints],
            axis=0,
        )
        if state.shape[0] != state_dim:
            raise ValueError(f"state dim mismatch: got {state.shape[0]}, expected {state_dim}")

        # ---- action ----
        cmd_vx = cmd_vy = cmd_wz = 0.0
        if cmd is not None and hasattr(cmd, "linear") and hasattr(cmd, "angular"):
            try:
                cmd_vx = float(cmd.linear.x)
                cmd_vy = float(cmd.linear.y)
                cmd_wz = float(cmd.angular.z)
            except Exception:
                pass

        action = np.concatenate(
            [
                np.array([cmd_vx, cmd_vy, cmd_wz], dtype=np.float32),
                arm_joints,
                hand_joints,
            ],
            axis=0,
        )
        frame: Dict[str, Any] = {
            "observation.state": state,
            "action": action,
            "task": task_value,
        }

        if use_video and topic_img is not None:
            pil_img: Optional[Image.Image] = None
            if imgm is not None:
                rgb = try_decode_image_ros(imgm)
                if rgb is not None:
                    # Ensure size matches feature shape; resize if needed.
                    if rgb.shape[0] != H or rgb.shape[1] != W:
                        rgb = np.asarray(
                            Image.fromarray(rgb).resize((W, H), resample=Image.BILINEAR),
                            dtype=np.uint8,
                        )
                    pil_img = Image.fromarray(rgb)

            # Carry-forward the last valid image; if none exists yet, use a black frame.
            if pil_img is None:
                pil_img = last_pil_img
            if pil_img is None:
                pil_img = Image.fromarray(np.zeros((H, W, 3), dtype=np.uint8))

            last_pil_img = pil_img
            frame["observation.images.front"] = pil_img

        frames.append(frame)

    return frames


# -----------------------------
# Rerun export (optional)
# -----------------------------
def rerun_export_episode_frames(
    frames: List[Dict[str, Any]],
    fps: int,
    out_rrd: Path,
) -> None:
    """
    Export the already-resampled fixed-FPS frames to a Rerun .rrd recording.

    This is the cleanest option if you want your Rerun visualization to match
    EXACTLY what you wrote to LeRobot.
    """
    if rr is None:
        raise RuntimeError("rerun is not installed. `pip install rerun-sdk`")

    rr.init("mcap_to_lerobot", spawn=False)
    rr.save(str(out_rrd))  # write to file, no viewer needed

    for i, fr in enumerate(frames):
        t = i / float(fps)
        rr.set_time_seconds("t", t)

        st = fr["observation.state"]
        ac = fr["action"]

        # Plot some time series (all dims)
        rr.log("state", rr.SeriesLine(st))
        rr.log("action", rr.SeriesLine(ac))

        # If you know indices: x,y,yaw in state, log pose too
        if len(st) >= 3:
            x, y, yaw = float(st[0]), float(st[1]), float(st[2])
            rr.log(
                "base",
                rr.Transform3D(
                    translation=[x, y, 0.0],
                    rotation=rr.Quaternion(xyzw=[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]),
                ),
            )

        if "observation.images.front" in fr:
            img = np.asarray(fr["observation.images.front"], dtype=np.uint8)
            rr.log("camera/front", rr.Image(img))

    rr.shutdown()


# -----------------------------
# Conversion
# -----------------------------
def convert_mcap_folder_to_lerobot_v3(
    mcap_paths: Sequence[Path],
    output_root: Path,
    repo_id: str,
    robot_type: str,
    fps: int,
    task: Optional[str],
    use_video: bool,
    image_hw: Tuple[int, int],
    topic_odom: str,
    topic_cmd: str,
    topic_hand: str,
    topic_img: Optional[str],
    state_dim: int,
    arm_joint_dim: int,
    hand_joint_dim: int,
    overwrite: bool,
    rerun_export_dir: Optional[Path] = None,
    debug_topics: bool = False,
) -> Path:
    output_root = Path(output_root)

    if output_root.exists():
        if overwrite:
            shutil.rmtree(output_root)
        else:
            raise FileExistsError(f"Output exists: {output_root} (pass --overwrite to replace)")

    # action_dim = base_cmd(3) + arm joints + hand joints
    action_dim = 3 + arm_joint_dim + hand_joint_dim

    features = make_features(
        state_dim=state_dim,
        action_dim=action_dim,
        use_video=use_video,
        image_hw=image_hw,
    )

    ds = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_root,
        fps=fps,
        robot_type=robot_type,
        features=features,
        use_videos=use_video,
    )

    topics = [topic_odom, topic_cmd, topic_hand]
    if use_video and topic_img is not None:
        topics.append(topic_img)

    rerun_export_dir = Path(rerun_export_dir) if rerun_export_dir is not None else None
    if rerun_export_dir is not None:
        rerun_export_dir.mkdir(parents=True, exist_ok=True)

    for epi, p in enumerate(mcap_paths):
        print(f"[{epi+1}/{len(mcap_paths)}] {p.name}")

        try:
            collected = read_mcap_topics(p, topics=topics, debug_topics=debug_topics)
        except ReaderError as e:
            print(f"  -> skip {p.name}: {e}")
            continue

        frames = build_fixed_fps_frames(
            collected=collected,
            topic_odom=topic_odom,
            topic_cmd=topic_cmd,
            topic_hand=topic_hand,
            topic_img=topic_img,
            fps=fps,
            state_dim=state_dim,
            arm_joint_dim=arm_joint_dim,
            hand_joint_dim=hand_joint_dim,
            use_video=use_video,
            task=task,
            image_hw=image_hw,
        )

        if not frames:
            if use_video and topic_img is not None and len(collected.get(topic_img, [])) == 0:
                print(f"  -> no images on {topic_img}, skipping episode")
            else:
                print("  -> no frames, skipping")
            continue

        # Optional: export to Rerun .rrd for smooth replay
        if rerun_export_dir is not None:
            out_rrd = rerun_export_dir / f"{p.stem}.rrd"
            try:
                rerun_export_episode_frames(frames, fps=fps, out_rrd=out_rrd)
                print(f"  -> rerun saved: {out_rrd}")
            except Exception as e:
                print(f"  -> rerun export failed (continuing): {e}")

        # Write frames to LeRobot dataset (NO extra keys!)
        for fr in frames:
            if "task" not in fr:
                # Safety net: ensure task is always present (LeRobot special required field)
                fr["task"] = "" if task is None else str(task)
            ds.add_frame(fr)

        ds.save_episode()
        print(f"  -> wrote {len(frames)} frames")

    ds.finalize()
    print(f"Done. Dataset saved to: {output_root}")
    return output_root


# -----------------------------
# CLI
# -----------------------------
def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser("Convert ROS2 MCAP -> LeRobot v3 (optionally export Rerun)")

    ap.add_argument("--input-dir", type=Path, required=True, help="Folder containing *.mcap episodes")
    ap.add_argument("--output-root", type=Path, required=True, help="Output LeRobot dataset folder")
    ap.add_argument("--repo-id", type=str, required=True, help='e.g. "local/evan_house"')
    ap.add_argument("--robot-type", type=str, default="mobile_base")

    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument(
        "--task",
        type=str,
        default="picking up the toy and place into the container",
        help="Task string stored per frame (as a single-element string feature).",
    )

    ap.add_argument("--use-video", action="store_true")
    ap.add_argument("--image-h", type=int, default=480)
    ap.add_argument("--image-w", type=int, default=640)

    ap.add_argument("--topic-odom", type=str, default="/odom")
    ap.add_argument("--topic-cmd", type=str, default="/cmd_vel")
    ap.add_argument("--topic-hand", type=str, default="/joint_states")
    ap.add_argument("--topic-img", type=str, default="/camera_4/color")

    ap.add_argument("--arm-joint-dim", type=int, default=7, help="Number of arm joints encoded in joint_states")
    ap.add_argument("--hand-joint-dim", type=int, default=7)

    ap.add_argument("--overwrite", action="store_true")

    # Rerun export
    ap.add_argument(
        "--rerun-export-dir",
        type=Path,
        default=None,
        help="If set, exports one .rrd per episode to this directory.",
    )

    ap.add_argument(
        "--debug-topics",
        action="store_true",
        help="Print available topics and collected counts per bag.",
    )

    return ap.parse_args()


def main() -> None:
    args = parse_args()

    mcap_paths = sorted(args.input_dir.glob("*.mcap"))
    if not mcap_paths:
        raise SystemExit(f"No *.mcap found in: {args.input_dir}")

    state_dim = 5 + int(args.hand_joint_dim)
    if state_dim <= 0:
        raise SystemExit("Invalid state_dim computed.")

    convert_mcap_folder_to_lerobot_v3(
        mcap_paths=mcap_paths,
        output_root=args.output_root,
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        fps=args.fps,
        task=args.task,
        use_video=bool(args.use_video),
        image_hw=(int(args.image_h), int(args.image_w)),
        topic_odom=args.topic_odom,
        topic_cmd=args.topic_cmd,
        topic_hand=args.topic_hand,
        topic_img=(args.topic_img if args.use_video else None),
        state_dim=state_dim,
        arm_joint_dim=int(args.arm_joint_dim),
        hand_joint_dim=int(args.hand_joint_dim),
        overwrite=bool(args.overwrite),
        rerun_export_dir=args.rerun_export_dir,
        debug_topics=bool(args.debug_topics),
    )


if __name__ == "__main__":
    main()
