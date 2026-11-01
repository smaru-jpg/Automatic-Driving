from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # -------------------------------
    # 1. YDLIDAR 기본 런치 포함
    # -------------------------------
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        ]),
        launch_arguments={
            'port': '/dev/ttyUSB2',
            'baudrate': '512000',
            'frame_id': 'laser'
        }.items()
    )

    # -------------------------------
    # 2. Python 노드들 실행
    # -------------------------------
    lidar_array_node = Node(
        package='wooak',
        executable='lidar_array.py',
        name='lidar_array',
        output='screen'
    )

    lidar_drive_node = Node(
        package='wooak',
        executable='lidar_drive.py',
        name='lidar_drive',
        output='screen'
    )

    maze_drive_node = Node(
        package='wooak',
        executable='maze_drive.py',
        name='maze_drive',
        output='screen'
    )

    # -------------------------------
    # 3. LaunchDescription 반환
    # -------------------------------
    return LaunchDescription([
        ydlidar_launch,
        lidar_array_node,
        lidar_drive_node,
        maze_drive_node
    ])

