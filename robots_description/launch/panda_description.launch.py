
import os

import launch_ros
from launch_ros.actions import Node
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true")

    pkg_share = launch_ros.substitutions.FindPackageShare(package="robots_description").find("robots_description")
    default_model_path = os.path.join(pkg_share, "robots/franka_panda/panda/panda.urdf.xacro")

    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")
   
    xacro_content = xacro.process_file(default_model_path).toxml()
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        
        parameters=[
            {"robot_description": xacro_content},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {'use_sim_time': True}
            ],
        # remappings=(("/joint_states", "/reactor_controller/joint_state"),),
    )

    return LaunchDescription(
        [
            declare_description_path,
            declare_use_sim_time,
            robot_state_publisher_node,
        ]
    )