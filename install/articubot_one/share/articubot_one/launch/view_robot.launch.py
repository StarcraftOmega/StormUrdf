import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Include the existing robot_state_publisher launch
    pkg_path = get_package_share_directory('articubot_one')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path,'launch','rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. Add Joint State Publisher GUI
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. Add RViz with the config file
    rviz_config_path = os.path.join(pkg_path, 'config', 'view_bot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        jsp_gui,
        rviz
    ])