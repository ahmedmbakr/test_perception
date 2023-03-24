from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_driver',
            namespace='velodyne',
            executable='velodyne_driver_node',
            name='velodyne_driver',
            parameters=[
                {"device_ip":"192.168.1.201"},
                {"model":"VLP16"}
            ]
        ),
        Node(
            package='velodyne_pointcloud',
            namespace='velodyne',
            executable='velodyne_convert_node',
            name='vevlodyne_pointcloud',
            parameters=[
                {"calibration": "/opt/ros/galactic/share/velodyne_pointcloud/params/VLP16db.yaml"}
            ]
        ),
        Node(
            package='tf2_ros',
            namespace='velodyne',
            executable='static_transform_publisher',
            name='velodyne_transform',
            output='screen',
            arguments=["0", "0", ".5715", "2.1708", "0", "0", "velodyne", "kart"]
        )
    ])

