import os

from xacro import process_file
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	# Launch description
	ld = LaunchDescription()

	# Package share path
	corc_path = get_package_share_directory("CORC")
	x2_description_path = get_package_share_directory("x2_description")

	# Package share paths
	exo_path = os.path.join(corc_path, "config", "x2_params.yaml")
	rviz_path = os.path.join(x2_description_path, "rviz", "x2.rviz")

	# Process XACRO
	urdf_xacro = process_file(
		os.path.join(x2_description_path, "urdf", "x2_fixed_base.urdf.xacro")
	)

	# Nodes
	exo_node = Node(
		package="CORC",
		executable="X2ROS2DemoMachine_APP",
		arguments=["-can", "can0"],
		name="x2",
		output="screen",
        parameters=[
			{ "exo_path": exo_path }
		]
	)
	robot_state_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		parameters=[
			{ "robot_description": urdf_xacro.toprettyxml(indent='	') }
		]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz_path],
		name="rviz2"
	)

	# Launch description actions
	ld.add_action(exo_node)
	ld.add_action(robot_state_node)
	ld.add_action(rviz_node)
	return ld
