import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'amrbot_dev' 

    use_sim_time = LaunchConfiguration('use_sim_time')
    viewbot = LaunchConfiguration('viewbot')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rviz_launch.py')]),
        launch_arguments={'view_bot':viewbot,'view_map':'false'}.items()
    )

    xacro_file = os.path.join(get_package_share_directory(package_name),'description','robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_xml = robot_description_config.toxml()
    params = {'robot_description':robot_xml,'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_robot_joint = Node(
        condition=IfCondition(viewbot),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=''),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description=''),
        DeclareLaunchArgument(
            'viewbot',
             default_value='false',
             description=''),

        node_robot_state_publisher,
        node_robot_joint,
        rviz_launch
    ])
