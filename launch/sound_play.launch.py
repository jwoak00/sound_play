from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sound_play_pkg',
            executable='sound_play',
            name='sound_play',
            output='screen',
            parameters=[{
                'topic': '/sound_id',
                'play_mode': 'single',            # 'single' or 'overlap'
                'file_ids':  [0, 1, 2, 3, 4, 5],
                'file_names': ['test_0.mp3', 'test_1.mp3', 'test_2.mp3', 'test_3.mp3', 'test_4.mp3', 'test_5.mp3'],
                'sounds_dir': '/home/ok/ros2_ws/src/sound_play_pkg/sound',
            }],
        )
    ])
