from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='duo_image_publisher',
            executable='camera_node',
            name='camera_1',
            parameters=[
                {'camera_id': '/dev/video3'},
                {'image_topic': '/camera1/image_raw'}
            ],
        ),
        Node(
            package='duo_image_publisher',
            executable='camera_node',
            name='camera_2',
            parameters=[
                {'camera_id': '/dev/video4'},
                {'image_topic': '/camera2/image_raw'}
            ],
        ),

        Node(
            package='duo_image_publisher',
            executable='camera_node',
            name='camera_3',
            parameters=[
                {'camera_id': '/dev/video6'},
                {'image_topic': '/camera3/image_raw'}
            ],
        ),
    ])
