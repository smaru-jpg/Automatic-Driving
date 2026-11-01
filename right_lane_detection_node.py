# 오른쪽 차선만 보고 주행하는 경우

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
# 카메라 노출값 조정 관련
import os

# 카메라 노출값 조정을 위한 세팅
def cam_exposure(value=60):
    os.system('v4l2-ctl -d /dev/video0 -c auto_exposure=3')
    os.system('v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0')
    os.system(f'v4l2-ctl -d /dev/video0 -c exposure_time_absolute={value}')
    print(f"[INFO] Camera exposure set to manual ({value})")

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('right_lane_detection_node')
        self.get_logger().info('right_Lane Detection Node Started!')

        # 카메라 노출 고정 (ROS 시작 시 한 번만 실행)
        cam_exposure(60) 
        self.get_logger().info('Camera exposure locked to manual mode (100)')
        
        # OpenCV Bridge 초기화
        self.bridge = CvBridge()
        
        # 구독자/발행자
        self.image_subscription = self.create_subscription(Image, '/image_raw',self.image_callback, 10)
        self.right_processed_image_pub = self.create_publisher(Image, '/right_lane_detection/processed_image', 10)

        # 성민 수정 부분2 
        self.blur_pub = self.create_publisher(Image,'/right/debug/blur',10)
        self.edge_pub = self.create_publisher(Image,'/right/debug/edge',10)
        #self.blur_pub = self.create_publisher(Image,'/deveub/blur',10)
        
        self.Right_Delta_pub = self.create_publisher(Float32, '/Right_lane_detection/Right_Delta_x', 10)
        
        # 파라미터
        self.WIDTH = 640
        self.HEIGHT = 480
        #x_left_top if문 밖에서 사용하기 위해
        self.prev_x_right_top = 540
        
        
        # ROI 설정 (중앙 부분만)
        self.roi_center_width = 200  # 중앙 800픽셀만 사용
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
        roi_start_x = self.WIDTH // 2
        roi_end_x = self.WIDTH-50   # 전체 가로의 절반까지만
        roi_img = image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x]
        
        
                # 화면의 중앙값 = 카메라 위치
        View_Center = (self.WIDTH) //2 
        
        #추가
        b, g, r= cv2.split(roi_img)
        yellow_enhanced = cv2.add(r,g)
        yellow_enhanced = cv2.subtract(yellow_enhanced, cv2.multiply(b,1.3))
        yellow_enhanced = np.clip(yellow_enhanced,0,255).astype(np.uint8)
        blur_mask_y = cv2.GaussianBlur(yellow_enhanced,(5,5),0)
        
        gray_mask = cv2.adaptiveThreshold(blur_mask_y, 255, 
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY, 55,16)
        edge_img_y = cv2.Canny(blur_mask_y, 100, 140)
        ##ksm추가 코드... 여기부터
        #self.gary_pub.publish(self.bridge.cv2_to_imgmsg(,"mono8")) ? gray가 없네
        self.blur_pub.publish(self.bridge.cv2_to_imgmsg(blur_mask_y,"mono8"))
        self.edge_pub.publish(self.bridge.cv2_to_imgmsg(edge_img_y,"mono8"))

        # Hough 변환 노란차선
        all_lines = cv2.HoughLinesP(edge_img_y, 1, math.pi/180, 30, 30, 15)
        if all_lines is None:
        # 차선 검출에 관계없이 ROI 박스/텍스트가 나타나도록함 
         self.visualize_lanes(image, roi_img, roi_start_x, [0.0, 0.0])
         return False

        # 선분 필터링 및 분류
        right_lines = []
        
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = float(y2-y1) / float(x2-x1)
            if abs(slope) <= 0.4: #수평에 가까운선(기울기 작음)은 차선이 아닐 가능성 커서 제거
                continue
            
            # ROI 영역 안에서 오른쪽 차선 분류  
            if (x1 < roi_end_x and x2 < roi_end_x):
                right_lines.append(line[0])

        # 대표직선 계산
        m_right = self.calculate_slope(right_lines)
        
        # Roi의 상단과 차선의 교점에서 x좌표 구하기
        if m_right[0] != 0.0:
            x_right_top = int((0 - m_right[1]) / m_right[0]) + roi_start_x
        else:
            x_right_top = self.prev_x_right_top
            
        self.prev_x_right_top = x_right_top
        
        x_midpoint = (x_right_top+x_right_top-420)//2 # x_left_top = x_right_top - 350으로 대체
        Right_Delta_raw = x_midpoint - View_Center
        Right_Delta_x = 0.0 if abs(Right_Delta_raw) < 10 else float(Right_Delta_raw)
        
        self.Right_Delta_pub.publish(Float32(data=Right_Delta_x))




        # 갈색 네모(View_Center)
        cv2.rectangle(image, (View_Center-5,self.roi_start_y-5), (View_Center+5,self.roi_start_y+5), (19, 69, 139), 4)

        # x_Right_top 표시 (빨간색 - 왼쪽 차선과 같은 색)
        cv2.circle(image, (x_right_top, self.roi_start_y), 8, (0, 0, 255), -1)
        
        #yello square
        cv2.rectangle(image, (x_midpoint-5,self.roi_start_y-5), (x_midpoint+5,self.roi_start_y+5), (0, 255, 255), 4)
        
        cv2.putText(image, f"L:{x_right_top}",
                    (x_right_top - 30, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
          
        cv2.putText(image, f"Right_Delta_x={int(Right_Delta_x)}",
                    (roi_end_x - 150, self.roi_start_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                   
       
        # 시각화
        self.visualize_lanes(image, roi_img, roi_start_x, m_right)
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

    def visualize_lanes(self, image, roi_img, roi_start_x, m_left):
        line_draw_img = roi_img.copy()
        
        # 오른쪽 차선 (빨간색)
        if m_left[0] != 0.0:
            x1 = int((0.0 - m_left[1]) / m_left[0])
            x2 = int((self.roi_height - m_left[1]) / m_left[0])
            cv2.line(line_draw_img, (x1, 0), (x2, self.roi_height), (0, 0, 255), 3)

        # ROI 영역을 원본 이미지에 합성
        roi_end_x = roi_start_x + roi_img.shape[1]
        image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x] = line_draw_img
        
        # ROI 영역 테두리 표시 (녹색 사각형)
        cv2.rectangle(image, 
                     (roi_start_x, self.roi_start_y), 
                     (roi_end_x, self.roi_end_y), 
                     (0, 255, 0), 2)
        
        # ROI 정보 텍스트 표시
        roi_text = f"ROI: {roi_img.shape[1]}x{self.roi_height}"
        cv2.putText(image, roi_text, 
                   (roi_start_x, self.roi_start_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        
        self.publish_result_image(image)

    def publish_result_image(self, image):
        if self.right_processed_image_pub.get_subscription_count() >0:
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.right_processed_image_pub.publish(processed_msg)
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
    
