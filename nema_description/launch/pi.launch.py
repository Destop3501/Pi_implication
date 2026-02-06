import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    Nema_Description_pkg = get_package_share_directory("nema_description")
    urdffile = os.path.join(Nema_Description_pkg,'urdf', 'model.urdf.xacro')
    rvizfile = os.path.join(Nema_Description_pkg,'config','config.rviz')

    print(urdffile)

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Flag to enable use sim time'
    )

    model = DeclareLaunchArgument(name="model", default_value=urdffile,
                                    description="Absalute URDF file")

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )
    print(robot_description)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='both',
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
	parameters=[{'use_sim_time': True}]
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
	parameters=[{
        	'use_sim_time': True,
        	'send_service_discovery': False, # Stop the background "parameter" scanning
        	'capabilities': ['parameters', 'services', 'connectionGraph'], # Limit what it tries to do
    		'ignore_unresponsive_param_nodes': True,
	}]
    )

    return LaunchDescription([
        model,
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        foxglove_node
    ])
