from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    urdf_path = "/home/dhruv-bansal/Documents/ros_arm/src/arm_teleop/urdf/simple_arm.urdf"

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_gui': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/dhruv-bansal/Documents/ros_arm/src/arm_teleop/rviz/arm_display.rviz'],
            output='screen'
        ),
    ])
