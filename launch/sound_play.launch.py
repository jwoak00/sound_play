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
                'play_mode': 'single',          # 'single' or 'overlap'
                'file_ids':  [0, 1, 2, 3, 4, 5],
                'file_names': ['idle.wav', 'start.mp3', 'stop.wav', 'warn.mp3', 'tmp.mp3', 'test_0.mp3'],       # 추가 후 재빌드
            }],
        )
    ])
