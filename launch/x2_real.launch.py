import os

from xacro import process_file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()

	# Package share path
	x2_description_path = get_package_share_directory("x2_description")

	# Package share files
	rviz_config = os.path.join(x2_description_path, "rviz", "view_robot.rviz")
	urdf_file = process_file(
		os.path.join(x2_description_path, "urdf", "x2_fixed_base.urdf.xacro")
	)

	# Nodes
	robot_node = Node(
		package="CORC",
		executable="X2DemoMachineROS2_APP",
		arguments=["-can", "can0"],
		name="x2",
		output="screen"
	)
	robot_state_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		parameters=[
			{ "robot_description": urdf_file.toprettyxml(indent='	') }
		],
		remappings=[
			("joint_states", "x2/joint_states"),
			("robot_description", "x2/robot_description")
		]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		# arguments=["-d", rviz_config],
		name="rviz2"
	)

	ld.add_action(robot_node)
	ld.add_action(robot_state_node)
	ld.add_action(rviz_node)
	return ld
