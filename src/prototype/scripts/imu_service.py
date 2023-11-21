import robot_upstart

j = robot_upstart.Job(name="imu_node")
j.symlink = True

j.add(package="prototype", filename="launch/imu_launch.py")

j=robot_upstart.Job(name="imu_node")

j.uninstall()