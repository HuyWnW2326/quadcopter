#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world",

    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600",

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)