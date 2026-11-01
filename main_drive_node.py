#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Float32MultiArray, Int32
import math
import time



# =================== 모드 상수 ===================
First_drive    = 1
Second_drive   = 2
OBSTACLE       = 3
CONE           = 4

PARKIN         = 5
PARKING        = 6
PARKOUT        = 7
MAZE           = 8
FINISH         = 9

class MainDriveNode(Node):
    def __init__(self, start_mode=1):
        super().__init__('main_drive_node')
        self.get_logger().info("✅ Main Drive Node Started!")

        # =================== 주행 관련 ===================
        self.delta_x = 0.0
        self.left_delta_x = 0.0
        self.right_delta_x = 0.0
        self.steer_gain = 5.5
        self.speed_default = 200.0
        self.mode = start_mode
        self.ignore_duration = 0 
        self.obstacle_flag = 0
        self.middle_object_flag=0
        self.obstacle_direction = 0
        self.obstacle_direction2 = 0
        self.obstacle_direction1 = 0

        self.parking_flag = 0
        self.parkout_flag = 0
        self.cone_done = 0
        self.maze_done = 0
        self.parkinflag=0




        # =================== 라이다 관련  ==================
        self.lidar_angle = 0.0
        self.lidar_speed = 0.0
        self.maze_steer = 0.0
    
        # =================== 퍼블리셔 ===================
        self.angle_pub = self.create_publisher(Float32, '/motor_command/angle', 10)
        self.speed_pub = self.create_publisher(Float32, '/motor_command/speed', 10)
        
        # =================== 구독자 ===================
        self.create_subscription(Float32, '/lane_detection/Delta_x', self.delta_callback, 10)
        self.create_subscription(Float32, '/Left_lane_detection/Left_Delta_x', self.left_delta_callback, 10)
        self.create_subscription(Float32, '/Right_lane_detection/Right_Delta_x', self.right_delta_callback, 10)
        self.create_subscription(Float32, '/lidar_steer', self.lidar_angle_callback, 10)
        self.create_subscription(Float32, '/maze_steer', self.maze_steer_callback, 10)
        self.create_subscription(Float32, '/lidar_speed', self.lidar_speed_callback, 10)
        self.create_subscription(Bool, '/stop_line_detected', self.stopline_callback, 10)
        self.create_subscription(Int32, '/obstacle_direction', self.obstacle_direction_callback, 10)
        self.create_subscription(Int32, '/obstacle_direction1', self.obstacle_direction1_callback, 10)
        self.create_subscription(Int32, '/obstacle_direction2', self.obstacle_direction2_callback, 10)
        self.create_subscription(Int32, '/maze_done', self.maze_done_callback, 10)
        self.create_subscription(Int32, '/cone_done', self.cone_done_callback, 10)


        # =================== 상태 변수 ===================
        self.stop_detected = False
        self.left_lane_timer = 0.0

        # =================== 타이머 ===================
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.main_loop)

    # =================== 콜백 ===================
    def delta_callback(self, msg):
        self.delta_x = msg.data

    def left_delta_callback(self, msg):
        self.left_delta_x = msg.data

    def right_delta_callback(self, msg):
        self.right_delta_x = msg.data

    def stopline_callback(self, msg):
        self.stop_detected = msg.data

    def lidar_angle_callback(self,msg):
        self.lidar_angle = msg.data

    def maze_steer_callback(self,msg):
        self.maze_steer= msg.data

    def lidar_speed_callback(self,msg):
        self.lidar_speed = msg.data

    def obstacle_direction_callback(self,msg):
        self.obstacle_direction = msg.data

    def obstacle_direction1_callback(self,msg):
        self.obstacle_direction1 = msg.data

    def obstacle_direction2_callback(self,msg):
        self.obstacle_direction2 = msg.data

    def maze_done_callback(self,msg):
        self.maze_done = msg.data
    def cone_done_callback(self,msg):
        self.cone_done = msg.data
    # =================== 모터 제어 ===================
    def drive(self, angle, speed):
        self.angle_pub.publish(Float32(data=angle))
        self.speed_pub.publish(Float32(data=speed))

    def stop_car(self, duration=0.0):
        self.drive(0.0, 0.0)
        if duration > 0:
            time.sleep(duration)


    # =================== 메인 루프 ===================
    def main_loop(self):
        # =================== 모드 분기 ===================

        if self.mode== First_drive: 
            if self.obstacle_direction2 == 1 :
                self.stop_car()
            else:
                self.get_logger().info("Mode: First_drive")
                steer = 800.0 + self.steer_gain * self.right_delta_x
                self.drive(steer, self.speed_default)

                if self.stop_detected:
                    self.get_logger().info("stopline detected")
                    self.stop_car(4.0)
                    self.mode = Second_drive
                    self.get_logger().info("Second Drive Start")
                    self.stop_detected = False
            

        #전방 장애물이 있으면 오른쪽 차선 주행, 없으면 왼쪽 차선 추행 -> 몇초간 그냥 차선 주행만 하다가 전방 장애물 만나면 1초간 정지 -> 다시 차선 주행 후 정지선 인식하면 정지
        elif self.mode == Second_drive:
            self.get_logger().info("Mode: Second_drive")

            # 처음 진입 시 시작 시간 기록
            if not hasattr(self, 'second_drive_start'):
                self.second_drive_start = time.monotonic()
                self.fixed_direction = None  # 방향 아직 없음

            elapsed = time.monotonic() - self.second_drive_start

            # 처음 장애물 방향 고정
            if self.fixed_direction is None and self.obstacle_direction1 in [0, 1]:
                self.fixed_direction = self.obstacle_direction1
                if self.fixed_direction == 1:
                    self.get_logger().info("🟢 첫 장애물 → 오른쪽 방향 고정")
                else:
                    self.get_logger().info("🟢 첫 장애물 → 왼쪽 방향 고정")

            # 30초 동안 장애물 무시 주행
            if elapsed < 30.0:
                self.get_logger().info(f"⚠️ 장애물 감지 무시 중 ({elapsed:.1f}s/30s)")

                # 방향이 고정된 경우 해당 방향으로만 주행
                if self.fixed_direction == 1:
                    self.get_logger().info("Right lane 유지 중")
                    steer = 800.0 + ((self.steer_gain+0.5) * self.right_delta_x)
                elif self.fixed_direction == 0:
                    self.get_logger().info("Left lane 유지 중")
                    steer = 800.0 + ((self.steer_gain + 0.5) * self.right_delta_x)
                else:
                    steer = 800.0  # 초기값 (아직 감지 안 됨)

                self.drive(steer, self.speed_default)


            else:  # 30초 지나면 정상 주행
                if self.stop_detected:
                    self.get_logger().info("stopline detected")
                    self.stop_car(2.0)
                    self.mode = OBSTACLE
                    self.get_logger().info("Mode change → OBSTACLE")
                    self.stop_detected = False

                else :
                    if self.obstacle_direction == 1 and self.middle_object_flag==0:
                        self.get_logger().info("middle_object_detected ")
                        self.middle_object_flag=1
                        self.stop_car(2.0)

                    elif self.obstacle_direction == 0:
                        steer = 800.0 + self.steer_gain * self.left_delta_x
                        self.drive(steer, self.speed_default)

        # 정지선에서 먼저 양쪽차선 검출 주행 -> 장애물 만나면 노드 실행 -> 장애물 지나면 다시 양쪽 차선 주행 -> 정지선 만나면 정지
        elif self.mode == OBSTACLE:
            self.get_logger().info("Mode 3: OBSTACLE")

            if self.obstacle_direction == 1 and self.obstacle_flag==0:
                self.obstacle_flag=1
                self.get_logger().info("Moving~")

                self.drive(1300.0, self.speed_default)
                time.sleep(4.0)     #ìš°
 
                self.drive(300.0,self.speed_default)
                time.sleep(7.7)     #ì¢Œ -> ë¨¸ë¦¬ ì •ë ¬

                self.drive(1300.0, self.speed_default)
                time.sleep(2.0)     #ìš° -> ë¨¸ë¦¬ ì •ë ¬

            else:
                #ìž¥ì• ë¬¼ ì•ˆë§Œë‚¬ì„ë•Œëž‘ ìž¥ì• ë¬¼ í†µê³¼ í›„ ì–‘ì„ ì°¨ì„  ì£¼í–‰
                steer = 800.0 +self.steer_gain * self.delta_x
                self.drive(steer, self.speed_default)

                if self.stop_detected and self.obstacle_flag==1:
                    self.get_logger().info("stopline detected")
                    self.stop_car(2.0)
                    self.drive(1000.0, self.speed_default)
                    time.sleep(2.0)
                    self.mode = CONE
                    self.get_logger().info("Mode 4 End â†’ Mode 5 (CONE) Start")
                    self.stop_detected = False

        # 라바콘 주행
        elif self.mode == CONE:
            self.get_logger().info("Mode 4: CONE")
            
            if self.obstacle_direction1==1:
                self.get_logger().info("lidar drive")

                steer = self.lidar_angle
                self.drive(steer,self.speed_default)
            else:
                steer = 800.0+self.steer_gain * self.left_delta_x
                self.drive(steer, self.speed_default)
                self.get_logger().info("lane drive")

                if self.stop_detected:
                    self.get_logger().info("stopline detected")
                    self.stop_car(2.0)
                    self.mode = PARKIN
                    self.get_logger().info("PARKING Start")
                    self.stop_detected = False
        
        # 라바콘 주행 끝나고 다음 정지선 넘을때 까지 하드코딩 우회전 -> 왼쪽 차선 주행 -> 노란 정지선 만나면 주차 노드 실행 -> 나와서 왼쪽 차선 주행 후 정지선 인식 후 정지
        elif self.mode == PARKIN:
            self.get_logger().info("Mode 5: PARKIN")
            
            if not hasattr(self, 'parkin_drive_start'):
                self.parkin_drive_start = time.monotonic()

            elapsed10 = time.monotonic() - self.parkin_drive_start

            if elapsed10 <5.0 :
                self.get_logger().info("right lane")
                self.drive(1300, self.speed_default)

            elif elapsed10 <15.0 and elapsed10 >5.0:
                self.get_logger().info("right lane")
                steer = 800.0+self.steer_gain * self.right_delta_x
                self.drive(steer, self.speed_default)

            else :
                self.get_logger().info("left lane")
                steer = 800.0+self.steer_gain * self.left_delta_x
                self.drive(steer, self.speed_default)
            
                if self.stop_detected:
                    self.stop_car()
                    self.mode = PARKING
            
        elif self.mode == PARKING:
            self.get_logger().info("Mode 6: PARKINg")
            steer = 800.0
            self.drive(steer, 200.0)
            time.sleep(1.0)
            steer = 1300.0
            self.drive(steer, -200.0)
            time.sleep(7.0)
            self.stop_car(2.0)
            self.mode = PARKOUT

        elif self.mode == PARKOUT:
            self.get_logger().info("Mode 6: parkout")
            self.parkout_flag=0
            if self.parkout_flag==0:
                self.get_logger().info("Mode 6: flag0")
                self.parkout_flag=1
                steer = 300.0
                self.drive(steer,200.0)
                time.sleep(2.0)

            if self.parkout_flag==1:
                self.get_logger().info("Mode 6: flag1")
                if self.stop_detected:
                    self.parkout_flag=2
                    self.stop_car(1.0)
                else: 
                    steer = 800.0 +self.steer_gain * self.right_delta_x
                    self.drive(steer, self.speed_default)
            
            if self.parkout_flag==2:
                self.get_logger().info("Mode 6: flag2")
                if self.stop_detected:
                    self.parkout_flag=3
                else:
                    steer = 800.0 +self.steer_gain * self.right_delta_x
                    self.drive(steer, self.speed_default)

            if self.parkout_flag==3:
                self.get_logger().info("Mode 6: flag3")
                if self.stop_detected:
                    self.stop_car(2.0)
                    self.mode = MAZE
                    self.get_logger().info("go to maze")
                    self.stop_detected = False
                else:
                    steer = 800.0 +self.steer_gain * self.right_delta_x
                    self.drive(steer, self.speed_default)

            
        # 주차 정지선에서부터 우회전 하드코딩 -> 미로 주행 -> 왼쪽차선 주행 -> 정지선 만나면 정지
        elif self.mode == MAZE:
            self.get_logger().info("Mode 4: maze")
            
            if self.obstacle_direction1==1:
                self.get_logger().info("lidar drive")

                steer = self.lidar_angle
                self.drive(steer,self.speed_default)
        
        # 정지선에서 왼쪽 장애물인지 오른쪽 장애물인지 판단 후 좌회전 or 우회전 하드코딩
        elif self.mode == FINISH:
            self.get_logger().info("Mode 7: FINal drive")
            self.drive(1300, self.speed_default)
            time.sleep(3.0)
            self.stop_car()



def main(args=None):
    rclpy.init(args=args)
    start_mode = int(input("Enter start mode (1~7): "))
    node = MainDriveNode(start_mode=start_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_car()
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()