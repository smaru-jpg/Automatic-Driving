# 양쪽 차선 보고 주행하는 경우

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
# 다른 부분에서 저장된 델타값 사용할 수 있도록
from std_msgs.msg import Float32
# try
import os

# try
def cam_exposure(value=60):
    os.system('v4l2-ctl -d /dev/video0 -c auto_exposure=1')
    os.system('v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0')
    os.system(f'v4l2-ctl -d /dev/video0 -c exposure_time_absolute={value}')
    print(f"[INFO] Camera exposure set to manual ({value})")

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.get_logger().info('Lane Detection Node Started!')

        # try 카메라 노출 고정 (ROS 시작 시 한 번만 실행)
        cam_exposure(60) 
        self.get_logger().info('Camera exposure locked to manual mode (80)')
        
        # OpenCV Bridge 초기화
        self.bridge = CvBridge()
        
        # 구독자/발행자
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, '/lane_detection/processed_image', 10)
        
        ### 성민 수정 부분2 
        self.blur_pub = self.create_publisher(Image,'/debug/blur',10)
        self.edge_pub = self.create_publisher(Image,'/debug/edge',10)
        #self.blur_pub = self.create_publisher(Image,'/deveub/blur',10)
        
        ### 여기까지
        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Delta_pub = self.create_publisher(Float32, '/lane_detection/Delta_x', 10)
        
        # 파라미터
        self.WIDTH = 640
        self.HEIGHT = 480
        # x_left_top, x_right_top if문 밖에서 사용하기 위해
        self.prev_x_left_top = 100
        self.prev_x_right_top = 540
        
        # ROI 설정 (중앙 부분만)
        self.roi_center_width = 640  # 중앙 800픽셀만 사용
        self.roi_start_y = int(self.HEIGHT * 0.6)
        self.roi_end_y = int(self.HEIGHT * 0.8)
        self.roi_height = self.roi_end_y - self.roi_start_y
        self.L_ROW = int(self.roi_height * 0.25)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 이미지 크기 자동 감지
            actual_height, actual_width = cv_image.shape[:2]
            if actual_width != self.WIDTH or actual_height != self.HEIGHT:
                self.WIDTH = actual_width
                self.HEIGHT = actual_height
                self.roi_center_width = min(800, self.WIDTH)  # 화면보다 작게
                self.roi_start_y = int(self.HEIGHT * 0.6)
                self.roi_end_y = int(self.HEIGHT * 0.8)
                self.roi_height = self.roi_end_y - self.roi_start_y
                self.L_ROW = int(self.roi_height * 0.25)
                self.get_logger().info(f'Resolution: {self.WIDTH}x{self.HEIGHT}, ROI width: {self.roi_center_width}')
            
            # 차선 검출
            self.lane_detect(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def lane_detect(self, image):
        # 중앙 ROI 설정
        roi_start_x = (self.WIDTH - self.roi_center_width) // 2
        roi_end_x = roi_start_x + self.roi_center_width
        roi_img = image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x]

        gray_img = cv2.cvtColor(roi_img,cv2.COLOR_BGR2GRAY)
        gray_mask = cv2.adaptiveThreshold(gray_img, 255, 
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY, 45, -3)

        blur_mask_y = cv2.GaussianBlur(gray_mask,(5,5),0)
        edge_img_y = cv2.Canny(blur_mask_y, 100, 140)


        try:
            self.blur_pub.publish(self.bridge.cv2_to_imgmsg(blur_mask_y, "mono8"))
            self.edge_pub.publish(self.bridge.cv2_to_imgmsg(edge_img_y, "mono8"))
        except Exception as e:
            self.get_logger().error(f"Error publishing debug images: {e}")



        # Hough 변환 노란차선
        all_lines = cv2.HoughLinesP(edge_img_y, 1, math.pi/180, 30, 30, 15)
        if all_lines is None:
        # 차선 검출에 관계없이 ROI 박스/텍스트가 나타나도록함 
         self.visualize_lanes(image, roi_img, roi_start_x, [0.0, 0.0], [0.0, 0.0])
         return False

        #try
        margin = 1
        center_x  = self.roi_center_width // 2
        left_min,  left_max  = 0, center_x - margin
        right_min, right_max = center_x + margin, self.roi_center_width - 1
        #try
        # 선분 필터링 및 분류
        left_lines, right_lines = [], []
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = float(y2-y1) / float(x2-x1)
            if abs(slope) <= 0.4: #수평에 가까운선(기울기 작음)은 차선이 아닐 가능성 커서 제거
                continue

            left_region = (left_min <= x1 <= left_max)
            right_region = (right_min <= x2 <= right_max)

            #try 왼쪽과 오른쪽 차선 간의 최소 gap 지정
            gap = (abs(x2 -x1) or abs(x1 - x2))

            #if slope < 0 and left_region and gap >= 5 :
            #    left_lines.append(line[0])
            #if slope > 0 and right_region and gap >= 5:
            #    right_lines.append(line[0])

            if left_region and gap >= 5 :
                left_lines.append(line[0])
            if right_region and gap >= 5:
                right_lines.append(line[0])

        
        # 대표직선 계산
        m_left = self.calculate_slope(left_lines)
        m_right = self.calculate_slope(right_lines)
        
        # 차선 위치 계산 (ROI 좌표를 전체 이미지 좌표로 변환)

        
        # Roi의 상단과 차선의 교점에서 x좌표 구하기(self.L_ROW 대신 self.roi_start_y 대입)
        if m_left[0] != 0.0:
            x_left_top = int((0 - m_left[1]) / m_left[0]) + roi_start_x
        else:
            x_left_top = self.prev_x_left_top
        if m_right[0] != 0.0:
            x_right_top = int((0 - m_right[1]) / m_right[0]) + roi_start_x
        else:
            x_right_top= self.prev_x_right_top

        if m_left[0] != 0.0 and m_right[0] == 0.0:
            x_right_top = x_left_top + 380
        elif m_left[0] == 0.0 and m_right[0] != 0.0:
            x_left_top = x_right_top - 380
        elif m_left[0] == 0.0 and m_right[0] == 0.0:
            pass
            
            
        self.prev_x_left_top = x_left_top
        self.prev_x_right_top = x_right_top

        # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치
        x_midpoint = (x_left_top + x_right_top) // 2
        # 화면의 중앙값 = 카메라 위치
        View_Center = (roi_start_x + self.roi_center_width) //2 
        # delta_x
        Delta_x_raw = float(x_midpoint- View_Center)
        # delta_x 너무 작은 값은 0으로 처리
        Delta_x = 0.0 if abs(Delta_x_raw) < 10 else Delta_x_raw

       
        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Delta_pub.publish(Float32(data=Delta_x))
        # 노란색 네모(x_midpoint)
        cv2.rectangle(image, (x_midpoint-5,self.roi_start_y-5), (x_midpoint+5,self.roi_start_y+5), (0, 255, 255), 4)
        # 갈색 네모(View_Center)
        cv2.rectangle(image, (View_Center-5,self.roi_start_y-5), (View_Center+5,self.roi_start_y+5), (19, 69, 139), 4)
        # delta x 표시
        cv2.putText(image, f"Delta_x={int(Delta_x)}",
                    (roi_end_x - 150, self.roi_start_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                # x_left_top 표시 (빨간색 - 왼쪽 차선과 같은 색)
        cv2.circle(image, (x_left_top, self.roi_start_y), 8, (0, 0, 255), -1)
        cv2.putText(image, f"L:{x_left_top}",
                    (x_left_top - 30, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # x_right_top 표시 (초록색 - 오른쪽 차선과 같은 색)
        cv2.circle(image, (x_right_top, self.roi_start_y), 8, (0, 255, 0), -1)
        cv2.putText(image, f"R:{x_right_top}",
                    (x_right_top + 10, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)           
        
        # 시각화
        self.visualize_lanes(image, roi_img, roi_start_x, m_left, m_right)
        return True

    def calculate_slope(self, lines):
        if len(lines) == 0:
            return [0.0, 0.0]
        
        x_sum = y_sum = m_sum = 0.0
        for line in lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if x2 != x1:
                m_sum += float(y2-y1)/float(x2-x1)
        
        size = len(lines)
        x_avg = x_sum / (size*2)
        y_avg = y_sum / (size*2)
        m = m_sum / size
        b = y_avg - m * x_avg
        return [m, b]

    def visualize_lanes(self, image, roi_img, roi_start_x, m_left, m_right):
        line_draw_img = roi_img.copy()
        
        # 왼쪽 차선 (빨간색)
        if m_left[0] != 0.0:
            x1 = int((0.0 - m_left[1]) / m_left[0])
            x2 = int((self.roi_height - m_left[1]) / m_left[0])
            cv2.line(line_draw_img, (x1, 0), (x2, self.roi_height), (0, 0, 255), 3)
        
        # 오른쪽 차선 (초록색)
        if m_right[0] != 0.0:
            x1 = int((0.0 - m_right[1]) / m_right[0])
            x2 = int((self.roi_height - m_right[1]) / m_right[0])
            cv2.line(line_draw_img, (x1, 0), (x2, self.roi_height), (0, 255, 0), 3)

        # ROI 영역을 원본 이미지에 합성
        roi_end_x = roi_start_x + self.roi_center_width
        image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x] = line_draw_img
        
        # ROI 영역 테두리 표시 (녹색 사각형)
        cv2.rectangle(image, 
                     (roi_start_x, self.roi_start_y), 
                     (roi_end_x, self.roi_end_y), 
                     (0, 255, 0), 2)
        
        # ROI 정보 텍스트 표시
        roi_text = f"ROI: {self.roi_center_width}x{self.roi_height}"
        cv2.putText(image, roi_text, 
                   (roi_start_x, self.roi_start_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        self.publish_result_image(image)

    def publish_result_image(self, image):
        if self.processed_image_pub.get_subscription_count() >0:
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.processed_image_pub.publish(processed_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    
    try:
        rclpy.spin(lane_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
