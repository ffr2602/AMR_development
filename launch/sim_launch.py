import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'amrbot_dev' 

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp_launch.py')]),
        launch_arguments={'use_sim_time':'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','joy_launch.py')]), 
        launch_arguments={'use_sim_time':'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo_params_world = os.path.join(get_package_share_directory(package_name),'world','maze_map.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args':'--ros-args --params-file ' + gazebo_params_file,
            'world':gazebo_params_world}.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch/localization_launch.py')])
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch/navigation_launch.py')])
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','robot'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        gazebo,
        spawn_entity,
        localization_launch,
        navigation_launch
    ])
