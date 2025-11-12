from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    urdf_path = "/home/dhruv-bansal/Documents/ros_arm/src/arm_teleop/urdf/simple_arm.urdf"

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/dhruv-bansal/Documents/ros_arm/src/arm_teleop/rviz/arm_final.rviz'],
            output='screen'
        ),
    ])

