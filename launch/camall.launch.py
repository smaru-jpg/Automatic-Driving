from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ USB 카메라 노드
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/ttyusu_cam',
                'framerate': 30.0,              # ✅ 추가!
                'pixel_format': 'mjpeg2rgb',    # ✅ 추가!
                'image_width': 640,             # ✅ 추가!
                'image_height': 480,            # ✅ 추가!
                'io_method': 'mmap',            # ✅ 추가!
                'autoexposure': False,          # ✅ 추가!
                'exposure': 60,                 # ✅ 추가!
                'auto_white_balance': False     # ✅ 추가!
            }]
        ),

        # 2️⃣ 차선 인식 파이썬 노드 (직접 실행)
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/stopline_find.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/left_lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/right_lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),
        
        

        # 3️⃣ 이미지 뷰어
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            output='screen'
        ),
    ])

