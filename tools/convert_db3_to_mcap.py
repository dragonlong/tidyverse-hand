#!/usr/bin/env python3
"""
Convert ROS2 rosbag (db3/sqlite3 format) to MCAP format.

This script handles both:
- rosbag2 directories (containing metadata.yaml + *.db3)
- direct *.db3 files

Usage:
    python convert_db3_to_mcap.py --input /path/to/rosbag_folder --output /path/to/output.mcap
    python convert_db3_to_mcap.py --input-dir /path/to/bags_folder --output-dir /path/to/mcap_folder

Requirements:
    pip install rosbags mcap
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import List, Optional

from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.rosbag2 import Writer as Rosbag2Writer


def convert_single_bag(input_path: Path, output_path: Path, overwrite: bool = False) -> bool:
    """
    Convert a single rosbag2 (db3) to mcap format.
    
    Args:
        input_path: Path to rosbag2 directory or .db3 file
        output_path: Path to output .mcap file
        overwrite: Whether to overwrite existing output
        
    Returns:
        True if conversion succeeded
    """
    # Handle output path
    if output_path.exists():
        if overwrite:
            if output_path.is_dir():
                shutil.rmtree(output_path)
            else:
                output_path.unlink()
        else:
            print(f"Output already exists: {output_path} (use --overwrite)")
            return False
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        # rosbags library reads from directories containing metadata.yaml
        # If input is a .db3 file directly, we need the parent directory
        if input_path.suffix == ".db3":
            # Check if there's a metadata.yaml in the same directory
            bag_dir = input_path.parent
            if not (bag_dir / "metadata.yaml").exists():
                print(f"Error: No metadata.yaml found for {input_path}")
                return False
            input_path = bag_dir
        
        with Rosbag2Reader(input_path) as reader:
            # Create mcap writer
            # The rosbags library's Writer can write to mcap with storage_id='mcap'
            # But the simpler approach is to use the convert utility
            
            # Actually, let's use a different approach - write messages one by one
            # Using mcap-ros2-support directly
            from mcap_ros2.writer import Writer as McapWriter
            
            with open(output_path, 'wb') as f:
                with McapWriter(f) as writer:
                    for connection, timestamp, rawdata in reader.messages():
                        writer.write_message(
                            topic=connection.topic,
                            schema=connection.msgtype,
                            message=rawdata,
                            log_time=timestamp,
                            publish_time=timestamp,
                        )
        
        print(f"Converted: {input_path} -> {output_path}")
        return True
        
    except ImportError:
        # Fall back to rosbags conversion
        print("mcap-ros2-support not available, using rosbags conversion...")
        return convert_with_rosbags(input_path, output_path)
    except Exception as e:
        print(f"Error converting {input_path}: {e}")
        return False


def convert_with_rosbags(input_path: Path, output_path: Path) -> bool:
    """
    Convert using rosbags library's built-in conversion.
    
    This creates a rosbag2 directory with mcap storage.
    """
    try:
        from rosbags.rosbag2 import Reader, Writer
        from rosbags.typesys import Stores, get_typestore
        
        # Output should be a directory for rosbag2 format
        output_dir = output_path.parent / (output_path.stem + "_mcap")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        with Reader(input_path) as reader:
            with Writer(output_dir) as writer:
                # Register all connections
                conn_map = {}
                for conn in reader.connections:
                    new_conn = writer.add_connection(
                        conn.topic,
                        conn.msgtype,
                        typestore=get_typestore(Stores.ROS2_HUMBLE),
                    )
                    conn_map[conn.id] = new_conn
                
                # Copy all messages
                for conn, timestamp, rawdata in reader.messages():
                    writer.write(conn_map[conn.id], timestamp, rawdata)
        
        print(f"Converted: {input_path} -> {output_dir}")
        return True
        
    except Exception as e:
        print(f"Error in rosbags conversion: {e}")
        return False


def find_rosbag_dirs(input_dir: Path) -> List[Path]:
    """
    Find all rosbag2 directories in the given path.
    
    A rosbag2 directory contains:
    - metadata.yaml
    - One or more .db3 files
    """
    bags = []
    
    # Check if input_dir itself is a rosbag
    if (input_dir / "metadata.yaml").exists():
        bags.append(input_dir)
    else:
        # Search subdirectories
        for subdir in input_dir.iterdir():
            if subdir.is_dir() and (subdir / "metadata.yaml").exists():
                bags.append(subdir)
    
    return sorted(bags)


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS2 rosbag (db3) to MCAP format"
    )
    
    # Single file conversion
    parser.add_argument(
        "--input", "-i",
        type=Path,
        help="Input rosbag2 directory (containing metadata.yaml) or .db3 file"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        help="Output .mcap file path"
    )
    
    # Batch conversion
    parser.add_argument(
        "--input-dir",
        type=Path,
        help="Input directory containing multiple rosbag2 folders"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory for .mcap files"
    )
    
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing output files"
    )
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.input and args.output:
        # Single conversion
        success = convert_single_bag(args.input, args.output, args.overwrite)
        exit(0 if success else 1)
        
    elif args.input_dir and args.output_dir:
        # Batch conversion
        bags = find_rosbag_dirs(args.input_dir)
        if not bags:
            print(f"No rosbag2 directories found in: {args.input_dir}")
            exit(1)
        
        print(f"Found {len(bags)} rosbag(s) to convert")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        
        success_count = 0
        for bag in bags:
            output_path = args.output_dir / f"{bag.name}.mcap"
            if convert_single_bag(bag, output_path, args.overwrite):
                success_count += 1
        
        print(f"Converted {success_count}/{len(bags)} bags")
        exit(0 if success_count == len(bags) else 1)
        
    else:
        parser.error("Provide either (--input, --output) or (--input-dir, --output-dir)")


if __name__ == "__main__":
    main()
