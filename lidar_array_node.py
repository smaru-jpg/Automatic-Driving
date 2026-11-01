#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math, time


class LidarArrayNode(Node):
    def __init__(self):
        super().__init__('lidar_array_node')

        # QoS 설정 (YDLIDAR도 BestEffort로 송신함)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 구독: 원본 라이다 데이터 (/scan)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        # 발행: 배열 형태의 라이다 데이터 (/lidar_array)
        self.pub = self.create_publisher(Float32MultiArray, '/lidar_array', 10)

        # ✅ G6는 약 1860포인트
        self.num_bins = 1860
        self.last_log_time = 0.0  # 로그 주기 제어용 타이머

        self.get_logger().info(f"✅ LidarArrayNode started for YDLIDAR G6 ({self.num_bins} bins)")

    def scan_callback(self, msg: LaserScan):
        """라이다 /scan 데이터를 받아 1860개 배열로 변환"""
        n = len(msg.ranges)
        if n == 0:
            return

        # --- 리샘플링: 원래 데이터 개수를 1860개로 맞춤 ---
        resampled = []
        factor = n / self.num_bins
        for i in range(self.num_bins):
            src_idx = int(i * factor)
            if src_idx >= n:
                src_idx = n - 1
            val = msg.ranges[src_idx]
            if math.isinf(val) or math.isnan(val):
                val = msg.range_max
            resampled.append(val)

        # 배열 데이터 메시지 생성 및 발행
        arr_msg = Float32MultiArray()
        arr_msg.data = resampled
        self.pub.publish(arr_msg)

        # 전방(0도 기준) 거리 표시 — 1초에 한 번만 로그
        front = resampled[930]
        now = time.time()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"[G6] Front distance: {front:.2f} m")
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = LidarArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
