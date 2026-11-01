#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Float32
from dynamixel_sdk import *

# ===== 1. 하드웨어 설정 (사용자 로봇에 맞게 수정) =====
DEVICENAME = '/dev/ttyu2d2'
BAUDRATE   = 57600
ID_LEFT    = 8
ID_RIGHT   = 9
ID_STEER   = 7

# 제어 테이블 주소 (X-series 기준)
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_VELOCITY    = 104
ADDR_GOAL_POSITION    = 116

# 동작 모드
MODE_VELOCITY = 1
MODE_POSITION = 3
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 조향 파라미터 (사용자 로봇에 맞게 수정)
STEER_CENTER   = 800
STEER_MIN      = STEER_CENTER - 500
STEER_MAX      = STEER_CENTER + 500

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.get_logger().info('Motor Driver Node (Actuation) Started!')

        # ===== 2. 다이나믹셀 초기 설정 =====
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(2.0)
        self.setup_dynamixel()

        # ===== 3. Subscriber 설정 =====
        # main_drive_node가 발행하는 최종 명령을 구독합니다.
        self.angle_subscriber = self.create_subscription(
            Float32,
            '/motor_command/angle',
            self.angle_callback,
            10)
            
        self.speed_subscriber = self.create_subscription(
            Float32,
            '/motor_command/speed',
            self.speed_callback,
            10)
            
        # 모터에 전달할 목표 값을 저장할 변수
        self.target_position = STEER_CENTER
        self.target_speed = 0

    def angle_callback(self, msg):
        """수신된 angle 값으로 조향 모터 목표 위치(position)를 계산합니다."""
        # angle_gain: ROS angle 값(보통 -50 ~ 50)을 다이나믹셀 position 값으로 변환하는 비율.
        # 이 값은 테스트를 통해 직접 튜닝해야 합니다.
        angle_gain = 5.0
        self.target_position = (msg.data)
        self.apply_command()
        self.get_logger().info(f"현재 조향값: {self.target_position:.1f}")


    def speed_callback(self, msg):
        """수신된 speed 값으로 구동 모터 목표 속도를 설정합니다."""
        self.target_speed = msg.data
        self.apply_command()

    def apply_command(self):
        """저장된 목표 값으로 실제 모터를 움직입니다."""
        # 조향 모터 제어 (위치)
        position_clamped = max(STEER_MIN, min(STEER_MAX, self.target_position))
        self.write4(ID_STEER, ADDR_GOAL_POSITION, int(position_clamped))

        # 구동 모터 제어 (속도)
        self.write4(ID_LEFT,  ADDR_GOAL_VELOCITY, int(self.target_speed))
        self.write4(ID_RIGHT, ADDR_GOAL_VELOCITY, int(-self.target_speed)) # 오른쪽은 반대 방향

    def setup_dynamixel(self):
        """다이나믹셀 포트와 모터 모드를 설정합니다."""
        if not self.port_handler.openPort(): self.get_logger().error("포트 오픈 실패"); sys.exit(1)
        if not self.port_handler.setBaudRate(BAUDRATE): self.get_logger().error("보드레이트 설정 실패"); sys.exit(1)
        
        # 조향(위치모드), 구동(속도모드) 설정 및 토크 인가
        for motor_id, mode in [(ID_STEER, MODE_POSITION), (ID_LEFT, MODE_VELOCITY), (ID_RIGHT, MODE_VELOCITY)]:
            self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.write1(motor_id, ADDR_OPERATING_MODE, mode)
            self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info("Dynamixel setup complete.")

    def on_shutdown(self):
        """노드 종료 시 모터를 정지하고 포트를 닫습니다."""
        self.get_logger().info("Shutting down. Stopping motors.")
        self.target_speed = 0
        self.target_position = STEER_CENTER
        self.apply_command()
        self.port_handler.closePort()

    # 다이나믹셀 통신을 위한 저수준 함수들
    def write1(self, id, addr, val): self.packet_handler.write1ByteTxRx(self.port_handler, id, addr, val)
    def write4(self, id, addr, val): self.packet_handler.write4ByteTxRx(self.port_handler, id, addr, val)


def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    try:
        rclpy.spin(motor_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.on_shutdown()
        motor_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()