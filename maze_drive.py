#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32
import math

# ===== 파라미터 설정 =====
VEL_LIMIT = 200
STEER_CENTER = 800
STEER_STEP = 3.125   # 인덱스 차이당 조향 변화량
# =================================

class LidarDriveNode(Node):
    def __init__(self):
        super().__init__('lidar_drive_node')

        # LiDAR 배열 구독
        self.sub = self.create_subscription(Float32MultiArray, '/lidar_array', self.lidar_callback, 10)

        # 조향 및 미션 완료 신호 발행
        self.angle_pub = self.create_publisher(Float32, '/lidar_steer', 10)
        self.mission_done_pub = self.create_publisher(Int32, '/maze_done', 10)  # ✅ 추가

        # 중복 발행 방지 플래그
        self.mission_done_flag = False

        self.get_logger().info("✅ LidarDriveNode started (Publishing /lidar_steer & /maze_done)")

    def lidar_callback(self, msg):
        if not msg.data:
            return

        ranges = msg.data
        n = len(ranges)

        # ===== YDLIDAR G6 데이터 유효성 =====
        if n < 1800 or n > 1900:
            self.get_logger().warn(f"⚠️ 예상 인덱스(약 1860)와 다름: {n}")
            return

        # ===== 구역 설정 (0~1860 인덱스 기준) =====
        right_indices  = list(range(1450, 1610))
        center_indices = list(range(1610, 1860)) + list(range(0, 250))
        left_indices   = list(range(250, 410))

        # ===== 거리 유효성 검증 =====
        ranges_valid = [20.0 if (r <= 0.1 or r > 16.0 or math.isnan(r)) else r for r in ranges]

        # ===== 각 영역 최소값 =====
        right_min_val  = min(ranges_valid[i] for i in right_indices)
        center_min_val = min(ranges_valid[i] for i in center_indices)
        left_min_val   = min(ranges_valid[i] for i in left_indices)

        # ===== 최소값 인덱스 =====
        right_min_idx  = min(right_indices, key=lambda i: ranges_valid[i])
        left_min_idx   = min(left_indices, key=lambda i: ranges_valid[i])

        # ===== 기본 설정 =====
        steer_pos = STEER_CENTER
        v_left = v_right = 200
        state = "🟢 Straight"

        # ===== 전방 하위 영역 =====
        front_left_indices  = list(range(1610, 1760))
        front_right_indices = list(range(0, 250)) + list(range(1760, 1860))
        front_left_min  = min(ranges_valid[i] for i in front_left_indices)
        front_right_min = min(ranges_valid[i] for i in front_right_indices)

        # ===== 주행 제어 로직 =====
        if (right_min_val < 0.4 or center_min_val < 0.70 or left_min_val < 0.4):
            if center_min_val < 0.65:
                if front_left_min > front_right_min:
                    steer_pos = 300
                    state = f"⬅️ Hard Left"
                elif front_left_min < front_right_min:
                    steer_pos = 1300
                    state = f"➡️ Hard Right"

            elif left_min_val < right_min_val and left_min_val < 0.4:
                offset = (410 - left_min_idx) * STEER_STEP
                steer_pos = STEER_CENTER - offset
                state = f"↩️ Turn left"

            elif right_min_val < left_min_val and right_min_val < 0.4:
                offset = (right_min_idx - 1449) * STEER_STEP
                steer_pos = STEER_CENTER + offset
                state = f"↪️ Turn right"

            else:
                steer_pos = STEER_CENTER
                state = "🟢 Straight Drive"

        else:
            steer_pos = STEER_CENTER
            state = "🟢 Straight Drive"

        # ===== 조향 한계 제한 =====
        steer_pos = max(min(steer_pos, 1300), 300)

        v_right = -v_right

        # ===== 발행 =====
        self.angle_pub.publish(Float32(data=steer_pos * 1.0))

        # ===== 터미널 출력 =====
        self.get_logger().info(
            f"{state} | angle={steer_pos:.1f}° | "
            f"L={left_min_val:.2f} R={right_min_val:.2f} C={center_min_val:.2f}"
        )

        # ===== 추가: 미션 완료 신호 발행 =====
        if center_min_val > 1.5 and right_min_val > 1.0 and left_min_val >1.0 and not self.mission_done_flag:
            self.mission_done_flag = True
            self.mission_done_pub.publish(Int32(data=1))
            self.get_logger().info("🏁 Maze section clear! Published 1 on /maze_done")


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
