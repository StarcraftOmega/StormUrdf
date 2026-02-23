import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Include the Robot State Publisher (RSP)
    # We tell it to use sim time so it syncs with Gazebo's clock
    pkg_path = get_package_share_directory('articubot_one')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path,'launch','rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo launch file (provided by the gazebo_ros package)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. Run the spawner node from gazebo_ros
    # This takes the robot_description published by RSP and puts it in the Gazebo world
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])