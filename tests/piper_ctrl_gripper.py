#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
import time
import argparse
from piper_sdk import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Piper gripper demo")
    parser.add_argument(
        "--fixed-range-0-05",
        dest="fixed_range_0_05",
        action="store_true",
        help="Always command gripper range 0.05 m (skips gradual sweep)",
    )
    args = parser.parse_args()

    piper = C_PiperInterface_V2("can_piper")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    piper.GripperCtrl(0,1000,0x02, 0)
    piper.GripperCtrl(0,1000,0x01, 0)
    range = 0
    count = 0
    
    while True:
        print(piper.GetArmGripperMsgs())
        count  = count + 1
        if args.fixed_range_0_05:
            range = 0.05 * 1000 * 1000
        else:
            # Gradually increase gripper range from 0 to 0.06 m as count goes
            # 0, 100, 200, ... , 500, then restart.
            if count % 100 == 0 and count <= 500:
                step = count // 100   # 0, 1, 2, ..., 5
                range_m = step * 0.01 # 0.00 m up to 0.05 m
                print(f"{count} -------- range = {range_m:.3f} m")
                range = range_m * 1000 * 1000
            if count > 500:
                count = 0
        range = round(range)
        start_time = time.time()
        piper.GripperCtrl(abs(range), 1000, 0x01, 0)
        elapsed = time.time() - start_time
        print(f"elapsed: {elapsed:.6f} s, range_cmd: {range}")
        time.sleep(0.005)
    
