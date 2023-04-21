import subprocess
import shlex
import time




command_1 = "ros2 launch gazebo_sim start_gazebo.launch.py"
command_2 = "ros2 launch prototype spawn.launch.xml"

commands = "/usr/bin/gnome-terminal --tab --title=gazebo -e 'bash -c \"" + command_1 + "; exec bash\"' --tab --title=rviz2 -e 'bash -c \"" + command_2 + "; exec bash\"'"

subprocess.run(shlex.split(commands))
