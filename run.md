### Diagnosis
```shell 
pygame 2.5.2 (SDL 2.28.2, Python 3.10.14)
Hello from the pygame community. https://www.pygame.org/contribute.html
Press the "Start" button on the gamepad to start control
[phoenix] CANbus Successfully Started: can0

CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device.
        talon fx 1 ("") Status Signal is_pro_licensed
Motor supply voltage: 12.75 V
[phoenix] CANbus Connected: DragonDrive (can0, F831FBCC50374E5320202047132E10FF)

[phoenix] CANbus Network Up: DragonDrive (can0, F831FBCC50374E5320202047132E10FF)

CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device.
        talon fx 2 ("") Status Signal is_pro_licensed
CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device.
        talon fx 3 ("") Status Signal is_pro_licensed
CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device.
        talon fx 4 ("") Status Signal is_pro_licensed
CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device.
        talon fx 5 ("") Status Signal is_pro_licensed
Control started
Warning: Step time 13.043 ms in Vehicle control_loop
Warning: Step time 14.372 ms in Vehicle control_loop
[phoenix] Library initialization is complete.

[phoenix-diagnostics] Server 2025.4.2 (Jul 10 2025, 18:29:45) running on port: 1250

Robot enabled (local frame)
Warning: Step time 11.608 ms in Vehicle control_loop
```
### Initial connection
```shell
# Interface can0 is connected to USB port 3-3:1.1
# Interface can_piper is connected to USB port 3-4:1.0
alias pcan='(cd /home/dragonx/piper_sdk/piper_sdk && bash find_all_can_port.sh)'
alias base_act='(cd /home/dragonx/piper_sdk/piper_sdk && bash can_activate.sh can0 1000000 "3-3:1.1")'
alias arm_act='(cd /home/dragonx/piper_sdk/piper_sdk && bash can_activate.sh can_piper 1000000 "3-4:1.0")'
# must re-run below if re-plugged in
pcan 
base_act
arm_act

# 
cd 
# check base control
sbot

# arm basic check
python /home/dragonx/piper_sdk/piper_sdk/demo/V2/piper_ctrl_end_pose.py

python /home/dragonx/piper_sdk/piper_sdk/demo/V2/piper_ctrl_gripper.py

# python /home/dragonx/piper_sdk/piper_sdk/demo/V2/piper_ctrl_enable.py

# sequence execution
python /home/dragonx/tidybot2/tests/piper_ctrl_end_pose.py --play 66 --sequence-file /home/dragonx/tidybot2/tests/end_pose_sequence.txt

# server mode
python /home/dragonx/tidybot2/arm_server_piper.py --mode demo --demo-verbose
```


### Telo-op 
```shell
tb
python policies.py

```

### Auto start
```shell
sudo install -m 755 ~/.local/bin/piper_start.sh /usr/local/bin/piper_start.sh
sudo install -m 644 ~/piper-can.service /etc/systemd/system/piper-can.service
sudo systemctl daemon-reload
sudo systemctl enable --now piper-can.service
```