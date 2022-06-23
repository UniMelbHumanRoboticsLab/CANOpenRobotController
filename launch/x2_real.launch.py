import os

from xacro import process_file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()

	# Package share path
	x2_description_path = get_package_share_directory("x2_description")
	rviz_path = os.path.join(
		x2_description_path,
		"rviz",
		"x2.rviz"
	)
	urdf_path = os.path.join(
		x2_description_path,
		"urdf",
		"x2_fixed_base.urdf.xacro"
	)

	# Package share files
	rviz_file = rviz_path
	urdf_file = process_file(urdf_path)

	# Nodes
	robot_node = Node(
		package="CORC",
		executable="X2DemoMachineROS2_APP",
		name="x2",
		output="screen",
		arguments=["-can", "can0"]
	)
	robot_state_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		output="screen",
		parameters=[
			{ "use_sim_time" : False },
			{ "robot_description" : urdf_file.toprettyxml(indent='	') }
		]
	)
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		arguments=["-d", rviz_file]
	)

	ld.add_action(robot_node)
	ld.add_action(robot_state_node)
	ld.add_action(rviz_node)
	return ld
