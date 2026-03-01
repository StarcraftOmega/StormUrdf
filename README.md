**1. Terminal 1: Launch Simulation**

cd ~/ros_2
colcon build --symlink-install --packages-select articubot_one
source install/setup.bash
ros2 launch articubot_one launch_sim.launch.py

**2. Terminal 2: Control**

cd ~/ros_2
source install/setup.bash
ros2 run articubot_one arrow_teleop.py

**3. Key Controls**

Up Arrow: Forward

Down Arrow: Backward

Left Arrow: Spin Left

Right Arrow: Spin Right

'q': Quit controller
