from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_info_url': 'file:///home/barbie/linorobot2_ws/install/yellow_ball_detector/share/yellow_ball_detector/config/camera_calibration.yaml'

            }]
        ),
    ])