#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import math

class LidarObstacleCheckNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_check_node')
        self.sub = self.create_subscription(Float32MultiArray, '/lidar_array', self.lidar_callback, 10)
        self.pub = self.create_publisher(Int32, '/obstacle_direction1', 10)
        self.get_logger().info("✅ LidarObstacleCheckNode started (/obstacle_direction)")

    def lidar_callback(self, msg):
        if not msg.data:
            return

        ranges = msg.data
        n = len(ranges)
        if n < 1800:
            return

        # ===== 영역 설정 (YDLIDAR G6 기준 0~1860) =====
        right_indices  = list(range(100, 1400))
        center_indices = list(range(1610, 1860)) + list(range(0, 250))

        # ===== 거리 유효성 보정 =====
        ranges_valid = [20.0 if (r <= 0.1 or r > 16.0 or math.isnan(r)) else r for r in ranges]

        # ===== 최소 거리 계산 =====
        right_min  = min(ranges_valid[i] for i in right_indices)
        center_min = min(ranges_valid[i] for i in center_indices)

        # ===== 단순 판단 =====
        if center_min < 1.2 and center_min > 0.2: 
            result = 1  # 정면 장애물 → 왼쪽 차선으로
            state = f"🚧 Front obstacle1 ({center_min:.2f} m) → output=1"
        else:
            result = 0  # 오른쪽 장애물 → 오른쪽 차선으로
            state = f"🚧 No obstacle1 ({center_min:.2f} m) → output=0"


        # ===== 퍼블리시 =====
        self.pub.publish(Int32(data=result))
        self.get_logger().info(state)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
