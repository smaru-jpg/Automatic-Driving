#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
# 다른 부분에서 저장된 델타값 사용할 수 있도록
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import time


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('stopline_node')
        self.get_logger().info('stopline Node Started!')
        
        self.use_undistort = False
        self.camera_matrix = None
        self.dist_coeffs = None
        # OpenCV Bridge 초기화
        self.bridge = CvBridge()
        
        # 구독자/발행자
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        #self.stop_find_image_pub = self.create_publisher(Image, '/stop_image', 10)

        ### 성민 수정 부분2 
        #self.grayimg_pub = self.create_publisher(Image,'/stop/grayimg',10)
        #self.graymask_pub = self.create_publisher(Image,'/stop/graymask',10)
        self.blur_pub = self.create_publisher(Image,'/stop/blur',10)
        self.stop_detected_pub = self.create_publisher(Bool,'/stop_line_detected',10)
        


        # 파라미터
        self.WIDTH = 640
        self.HEIGHT = 480
        self.prev_x_left = 100
        self.prev_x_right = 540
        # x_left_top, x_right_top if문 밖에서 사용하기 위해
        self.prev_x_left_top = 100
        self.prev_x_right_top = 540
        
        # ROI 설정 (중앙 부분만)
        self.roi_center_width = 200  # 중앙 800픽셀만 사용
        self.roi_start_y = int(self.HEIGHT * 0.7) # 0.7
        self.roi_end_y = int(self.HEIGHT * 0.8)   # 0.8
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
                self.roi_start_y = int(self.HEIGHT * 0.6) # 0.7
                self.roi_end_y = int(self.HEIGHT * 0.8)   # 0.8
                self.roi_height = self.roi_end_y - self.roi_start_y
                self.L_ROW = int(self.roi_height * 0.25)
                self.get_logger().info(f'Resolution: {self.WIDTH}x{self.HEIGHT}, ROI width: {self.roi_center_width}')
            
            # 차선 검출
            self.lane_detect(cv_image)
            
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            
        

    def lane_detect(self, image):
        
        stop_detected = False
        # 중앙 ROI 설정
        roi_start_x = (self.WIDTH - self.roi_center_width) // 2
        roi_end_x = roi_start_x + self.roi_center_width
        roi_img = image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x]


        # Grayscale - threshhold     
        gray_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray_mask = cv2.adaptiveThreshold(gray_img, 255, 
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY, 35, -20) #-7
        blur_img = cv2.GaussianBlur(gray_mask, (3, 3), 0)
        
 

        # 흰색 정지선 검출
        
        #self.grayimg_pub.publish(self.bridge.cv2_to_imgmsg(gray_img,"mono8"))
        self.blur_pub.publish(self.bridge.cv2_to_imgmsg(blur_img,"mono8"))
        #self.graymask_pub.publish(self.bridge.cv2_to_imgmsg(gray_mask,"mono8"))



        # ROI에서 정지선 판단 범위
        band_stop =gray_mask

        # 행 픽셀 수에 따른 정지선 판단
        if band_stop.size > 0:
            row_counts = np.count_nonzero(band_stop, axis=1)   # 각 행의 흰 픽셀 수
            
            if row_counts.size > 0:
                best_idx = int(np.argmax(row_counts))
                coverage = row_counts[best_idx] / float(self.roi_center_width)
                start = max(0, best_idx)
                end = min(len(row_counts), best_idx+1)
                neighbors = row_counts[start:end]
                avg_coverage = np.mean(neighbors) / float(self.roi_center_width)

                if coverage > 0.40 and avg_coverage > 0.50:  # # 흰 픽셀의 정지선 판단 가로 퍼센티지
                    stop_detected = True
                    stop_y_img = self.roi_start_y + best_idx
                    #time.sleep(0.9)

                # 정지선 시각화
                    cv2.line(image,
                        (roi_start_x, stop_y_img),
                        (roi_start_x + self.roi_center_width, stop_y_img),
                        (0, 255, 255), 2)
                    cv2.putText(image, "STOP LINE",
                        (roi_start_x + 10, stop_y_img - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        msg = Bool()
        msg.data = stop_detected
        self.stop_detected_pub.publish(msg)
        
        #result_msg = self.bridge.cv2_to_imgmsg(image,"bgr8")
        #self.stop_find_image_pub.publish(result_msg)
        
    
        self.get_logger().info(f'stop line: {stop_detected}')
        


    def visualize_lanes(self, image, roi_img, roi_start_x, m_left, m_right):
        line_draw_img = roi_img.copy()
        
        
        # ROI 영역을 원본 이미지에 합성
        roi_end_x = roi_start_x + self.roi_center_width
        image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x] = line_draw_img
        
        # ROI 영역 테두리 표시 (녹색 사각형)
        cv2.rectangle(image, 
                     (roi_start_x, self.roi_start_y), 
                     (roi_end_x, self.roi_end_y), 
                     (100, 100, 0), 2)
        
        # ROI 정보 텍스트 표시
        roi_text = f"ROI: {self.roi_center_width}x{self.roi_height}"
        cv2.putText(image, roi_text, 
                   (roi_start_x, self.roi_start_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        self.publish_result_image(image)





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
    