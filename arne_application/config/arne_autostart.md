# Realizing autostart on Ubuntu 20.04
The goal is to start everything for robot control automatically right on startup.
This gives non-technicians *power-on-an-go* functionality.

We use system services and [catmux](https://github.com/fmauch/catmux) for this task.
Here are the steps:

1.
   On the control pc, under `/etc/systemd/system/`, create an *arne_autostart.service* with the following content:
   ```bash
   [Unit]
   Description=Start catmux-session with ArNe project configuration
   After=network.target
   After=systemd-user-sessions.service
   After=network-online.target
   
   [Service]
   Type=forking
   User=arne
   Environment=PATH=/home/arne/.local/bin/:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
   ExecStart=/home/arne/autostart.bash
   ExecStop=/usr/bin/tmux -L catmux kill-session -t autostart
   
   [Install]
   WantedBy=multi-user.target
   ```
   
2.
   Enable the service with `systemctl enable arne_autostart.service`. The service will now be called automatically on startup.


3.
   Create an executable `autostart.bash` file in the user's home directory as specified in `ExecStart` from above with the following content:
   ```bash
   #!/bin/bash
   catmux_create_session /home/arne/src/robot_folders/checkout/arne/catkin_ws/src/ArNe/arne_application/config/arne_autostart.yaml -n autostart -d
   ```
   Make sure to point the catmux config file to the respective configuration from the *arne_application* package.
   The session's name will be *autostart* in this example.


## Useful commands
Managing system services:

```bash
sudo systemctl start arne_autostart.service
sudo systemctl stop arne_autostart.service
```

Managing *tmux*:
```bash
tmux -L catmux attach
tmux -L catmux list-sessions
tmux -L catmux kill-session -t autostart
```
