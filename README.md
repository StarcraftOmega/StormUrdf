##Articubot One: 6-Wheeled Edition

This guide provides the exact command sequence to build, visualize, and simulate your custom 6-wheeled robot in ROS 2 Humble.

1. Build the Workspace

Always run this after making changes to your .xacro files or adding new launch/config files.

# Navigate to your workspace root
cd ~/ros_2

# Build the specific package
colcon build --packages-select articubot_one

# Source the overlay
source install/setup.bash


#2. Visualization Mode (RViz)

Use this mode to verify your robot's appearance and joint transforms without running physics.

# Launch RSP, the Joint State GUI, and RViz with your saved config
ros2 launch articubot_one view_robot.launch.py


Note: If you haven't saved your config yet, open RViz manually (ros2 run rviz2 rviz2), add the RobotModel display, and save it to src/articubot_one/config/view_bot.rviz.

#3. Physics Simulation (Gazebo)

Use this mode to interact with the world and test the movement logic of the 6-wheel diff-drive plugin.

Terminal 1: Launch the Simulation

source ~/ros_2/install/setup.bash
ros2 launch articubot_one launch_sim.launch.py


Terminal 2: Drive the Robot

source ~/ros_2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard


Controls: Use i to go forward, k to stop, and j/l to turn.

4. Key Configuration Files

description/robot.urdf.xacro: The entry point that includes core and gazebo files.

description/robot_core.xacro: Contains the 6-wheel macro and chassis geometry.

description/gazebo_control.xacro: Configures the libgazebo_ros_diff_drive plugin for 6-joint control.

config/view_bot.rviz: Your saved RViz layout (Fixed frame: base_link).
