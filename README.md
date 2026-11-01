# WOOAK Autonomous Driving

ROS 2 기반 자율주행 프로젝트입니다. 카메라 차선 인식과 YDLIDAR G6 라이다 데이터를 이용해 직진, 장애물 회피, 정지선, 미로 주행 등 다양한 주행 모드를 수행합니다.

## 주요 기능

- 카메라 영상 기반 차선 검출 및 좌우 차선 오차 계산
- 정지선 인식
- YDLIDAR G6 라이다 데이터 전처리 및 장애물 방향 판단
- 미로 주행, 장애물 회피, 주차 및 출차 모드 제어
- ROS 2 토픽을 통한 모터 조향/속도 명령 전달

## 시스템 구성

- ROS 2
- Python
- OpenCV
- cv_bridge
- USB 카메라
- YDLIDAR G6
- Dynamixel SDK

## 주요 노드

| 파일 | 역할 |
|---|---|
| `main_drive_node.py` | 전체 주행 상태 및 모드 관리 |
| `lane_detection_node.py` | 전방 차선 검출 및 차선 오차 발행 |
| `left_lane_detection_node.py` | 왼쪽 차선 검출 |
| `right_lane_detection_node.py` | 오른쪽 차선 검출 |
| `stopline_find.py` | 정지선 검출 |
| `lidar_array_node.py` | 라이다 스캔 데이터를 고정 길이 배열로 변환 |
| `lidar_drive.py` | 라이다 기반 주행 제어 |
| `maze_drive.py` | 미로 주행 제어 |
| `m2ct.py` | Dynamixel 모터 제어 인터페이스 |

## Launch 파일

- `launch/camall.launch.py`: USB 카메라와 차선/정지선 인식 노드 실행
- `launch/lidarall.launch.py`: YDLIDAR와 라이다 기반 주행 노드 실행

## 빌드 및 실행

ROS 2 워크스페이스의 `src` 폴더 아래에 이 패키지를 두고 빌드합니다.

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

카메라 기반 노드를 실행합니다.

```bash
ros2 launch wooak camall.launch.py
```

라이다 기반 노드를 실행합니다.

```bash
ros2 launch wooak lidarall.launch.py
```

## 하드웨어 설정

현재 코드에는 다음과 같은 개발 환경 종속 설정이 포함되어 있습니다.

- USB 카메라 장치 경로: `/dev/ttyusu_cam`
- 카메라 해상도: `640x480`, 30 FPS
- YDLIDAR 포트: `/dev/ttyUSB2`
- YDLIDAR baudrate: `512000`
- 일부 launch 명령의 절대 경로가 `/home/kkk/come_ws/src/wooak/wooak/...` 형식으로 작성되어 있음

실제 환경에 맞게 카메라 장치, 시리얼 포트, baudrate 및 ROS 2 경로를 수정해야 합니다.

## 펌웨어 소스

텍스트 파일에 합쳐져 있던 MCU 코드는 MCU별로 분리했습니다.

- `firmware/stm32_recording/main.c`: STM32 모터 기록/재생 프로그램
- `firmware/stm32_joystick/main.c`: STM32 조이스틱 기반 주행/카메라 제어 메뉴
- `firmware/c2000_f2807x/main.c`: TI C2000 짐벌, 서보 및 4축 모터 제어

펌웨어별 헤더, 핀 설정, ADC/DMA/I2C/UART 초기화, 링커 스크립트와 IDE 프로젝트 파일이 필요합니다.
