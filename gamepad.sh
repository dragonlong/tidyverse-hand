[Unit]
Description=Gamepad Teleop

[Service]
ExecStart=/bin/bash -c "/home/dragonx/miniforge3/envs/tidybot2/bin/python /home/dragonx/tidybot2/gamepad_teleop.py"
User=dragonx
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target