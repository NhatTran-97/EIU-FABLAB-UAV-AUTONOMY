import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Lidar Driver Node
    lidar_node = Node(
        package='obstacle_avoidance',
        executable='stl_50b2_node',
        name='stl_50b2_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 230400,
            'frame_id': 'laser_link'
        }]
    )

    # Static Transform Publisher (Map -> Laser) for RViz2
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['--frame-id', 'map', '--child-frame-id', 'laser_link'],
    )

    return LaunchDescription([
        lidar_node,
        tf_node
    ])
