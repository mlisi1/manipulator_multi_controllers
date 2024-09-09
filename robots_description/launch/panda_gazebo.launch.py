import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution



def generate_launch_description():

    world_init_pos = ["0", "0", "0"]
    world_init_orient = ["0", "0", "0"]
    robot_name = "panda"

    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('robots_description'), 'launch', 'panda_description.launch.py']
    )

  


    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "--pause",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
        )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
    )
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_states_controller'],
        output='screen',
    )

    load_joint_trajectory_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_position_controller'],
        output='screen'
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[{'use_sim_time' : True}],
            arguments=['-d' + os.path.join(get_package_share_directory('robots_description'), 'config', 'panda.rviz')],
            )



    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "/robot_description",
            "-robot_namespace",
            "",
            "-x",
            world_init_pos[0],
            "-y",
            world_init_pos[1],
            "-z",
            world_init_pos[2],
            "-R",
            world_init_orient [0],
            "-P",
            world_init_orient[1],
            "-Y",
            world_init_orient[2],
        ],
    )


    joint_state_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_gui' : True,
                'rate' : 5,
                'use_sim_time' : True,
                }],
            )

    return LaunchDescription(
        [   
            # control_node,
            start_gazebo_client_cmd,
            start_gazebo_server_cmd,
            start_gazebo_spawner_cmd,
            launch_description,
            # load_joint_state_controller,
            # load_joint_trajectory_position_controller        
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            rviz_node,
            # joint_state_node

        ]
    )


