#!/usr/bin/env python3
"""
Node: Autonomous NMPC Controller with Advanced Safe-Stop Mechanism
Mô tả: Bộ điều khiển bám làn tối ưu phi tuyến NMPC (CasADi) kết hợp Multi-Sensor Fusion
(Camera + IMU + LiDAR + Odometry) và cơ chế DỪNG AN TOÀN (Safe Stop) tức thời khi vi phạm
các ràng buộc an toàn (Lệch làn, Mất lái, Vật cản, Mất dấu camera).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist, TwistStamped, Point
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import casadi as ca
import numpy as np

def euler_from_quaternion(q):
    """Chuyển đổi Quaternion sang góc Yaw (theta) quanh trục Z"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

class AutonomousNMPCNode(Node):
    def __init__(self):
        super().__init__('autonomous_nmpc_node')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()

        # ==================== 1. CẢM BIẾN ĐẦU VÀO ====================
        # 1.1 Camera nhận diện làn đường thực tế
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos_sensor
        )

        # 1.2 IMU: Góc hướng thật và tốc độ góc quán tính
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_sensor
        )

        # 1.3 LiDAR: Quét chướng ngại vật phía trước
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_sensor
        )

        # 1.4 Odometry từ Ackermann Controller
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.odom_callback,
            qos_sensor
        )

        # 1.5 Mode kịch bản tốc độ từ xa
        self.mode_sub = self.create_subscription(
            Float32,
            '/target_speed_mode',
            self.mode_callback,
            10
        )

        # 1.6 Lệnh khởi chạy từ Race Commander (terminal)
        # Xe sẽ ĐỨNG YÊN cho đến khi nhận được /race_start = True
        self.race_start_sub = self.create_subscription(
            Bool,
            '/race_start',
            self.race_start_callback,
            10
        )

        # ==================== 2. ĐẦU RA ĐIỀU KHIỂN & BÁO CÁO ====================
        self.cmd_pub_stamped = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10
        )
        self.cmd_pub_unstamped = self.create_publisher(
            Twist,
            '/ackermann_steering_controller/reference_unstamped',
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Topic phát trạng thái an toàn hệ thống
        self.safety_pub = self.create_publisher(String, '/safety_status', 10)

        # Visualizations
        self.trajectory_pub = self.create_publisher(Marker, '/nmpc_trajectory_line', 10)
        self.debug_img_pub = self.create_publisher(Image, '/camera/lane_debug', 10)

        # ==================== 3. CÁC THÔNG SỐ VẬT LÝ & RÀNG BUỘC AN TOÀN ====================
        self.L = 0.3          # Wheelbase (m)
        self.N = 15           # Prediction Horizon
        self.dt = 0.1         # Chu kỳ điều khiển (s) - 10Hz
        
        self.v_ref = 1.5      # Vận tốc mục tiêu (m/s)
        self.emergency_brake = False

        # --- CẤU HÌNH RÀNG BUỘC AN TOÀN (SAFETY CONSTRAINTS) ---
        # Làn đường rộng ~0.50m => Nửa bề rộng làn là 0.25m. Xe rộng 0.26m.
        # Nếu lệch quá 0.20m, bánh xe bắt đầu đè vạch biên/tim đường => Vi phạm an toàn!
        self.max_allowed_lane_deviation = 0.20    # Ngưỡng lệch làn tối đa khi bám làn (m)
        self.max_allowed_heading_deviation = 0.50 # Ngưỡng góc lệch tối đa (~28.6 độ)
        self.safe_obstacle_dist = 0.90            # Ngưỡng an toàn vật cản LiDAR (m)
        self.max_lane_lost_frames = 8             # Số frame mất dấu làn tối đa trước khi safe stop (0.8s)

        # Trạng thái Safe Stop & FSM Vượt Chướng Ngại Vật
        # Các trạng thái: "NORMAL", "SAFE_STOP_WAITING", "OVERTAKING", "RETURNING"
        self.fsm_state = "NORMAL"
        self.safe_stop_active = False
        self.safe_stop_reason = ""
        self.safe_stop_start_time = None
        self.safe_stop_duration = 5.0             # Thời gian dừng chờ an toàn (5s)
        self.target_lateral_offset = 0.0          # Offset mục tiêu: 0.0 (làn phải), +0.52m (làn trái)
        self.overtake_start_x = 0.0               # Vị trí x khi bắt đầu chuyển làn vượt
        self.lane_lost_counter = 0

        # Trạng thái cảm biến 360 LiDAR
        self.front_dist = 999.0
        self.left_dist = 999.0
        self.right_dist = 999.0
        self.rear_dist = 999.0
        self.surrounding_safety_details = "Chưa có dữ liệu"

        # Tọa độ và vận tốc từ Odometry
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.current_v = 0.0          # Vận tốc dọc (m/s)
        self.imu_yaw = 0.0            # Góc xoay IMU quanh trục Z (rad)
        self.imu_yaw_rate = 0.0       # Tốc độ góc IMU (rad/s)
        self.min_obstacle_dist = 999.0# Khoảng cách LiDAR gần nhất phía trước (m)
        self.obs_x = 20.0
        self.obs_y = 0.0

        # Trạng thái nhận diện làn đường & Lookahead Preview
        self.camera_received = False
        self.lane_detected = False
        self.lateral_error = 0.0      # Sai số lệch ngang làn đường e_y (m)
        self.heading_error = 0.0      # Sai số góc lệch làn đường e_theta (rad)
        self.lookahead_heading = 0.0  # Góc hướng đường phía xa dự báo trước
        self.lane_half_width_px = 75  # Nửa độ rộng làn bên phải trên ảnh (pixel)

        # Điều khiển tích lũy
        self.odom_received = False
        self.current_cmd_v = 0.0

        # === CỜ KHỞI CHẠY TỪ TERMINAL (Race Commander) ===
        # Xe sẽ đứng yên hoàn toàn cho đến khi race_started = True
        self.race_started = False

        # Khởi tạo bộ giải NMPC
        self.init_nmpc_solver()

        # Timer 10Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("=> Autonomous NMPC Node với Cơ chế Safe Stop đã sẵn sàng hoạt động!")
        self.get_logger().info("⏳ Đang CHỜ lệnh khởi chạy từ Race Commander...")
        self.get_logger().info("   Mở terminal mới và chạy: ros2 run my_car_simulation race_commander_node.py")

    # ==================== CALLBACK CẢM BIẾN ====================
    def imu_callback(self, msg: Imu):
        """Xử lý tín hiệu quán tính IMU: góc Yaw và Yaw Rate chính xác"""
        self.imu_yaw = euler_from_quaternion(msg.orientation)
        self.imu_yaw_rate = msg.angular_velocity.z

    def odom_callback(self, msg: Odometry):
        """Đọc vận tốc và vị trí thực tế từ controller"""
        self.current_v = msg.twist.twist.linear.x
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        """
        Xử lý LiDAR 360 độ: Quét 4 vùng xung quanh xe
        - FRONT (Phía trước: [-45°, +45°]): Phát hiện vật cản cần Safe Stop / Né
        - LEFT  (Bên trái   : [+45°, +135°]): Kiểm tra làn trái đủ an toàn để vượt
        - RIGHT (Bên phải  : [-135°, -45°]): Kiểm tra làn phải khi về làn
        - REAR  (Phía sau   : [<-135° hoặc >+135°]): Kiểm tra phía sau thông thoáng
        """
        try:
            ranges = np.array(msg.ranges)
            total_beams = len(ranges)
            if total_beams == 0:
                return

            angles = msg.angle_min + np.arange(total_beams) * msg.angle_increment
            valid_mask = (~np.isinf(ranges)) & (~np.isnan(ranges)) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

            # 1. FRONT: [-45°, +45°] ~ [-0.785, +0.785] rad
            front_mask = valid_mask & (np.abs(angles) <= 0.785)
            if np.any(front_mask):
                front_indices = np.where(front_mask)[0]
                min_i = front_indices[np.argmin(ranges[front_indices])]
                self.front_dist = float(ranges[min_i])
                ang = angles[min_i]
                self.obs_x = self.front_dist * np.cos(ang)
                self.obs_y = self.front_dist * np.sin(ang)
                self.min_obstacle_dist = self.front_dist
            else:
                self.front_dist = 999.0
                self.min_obstacle_dist = 999.0
                self.obs_x = 20.0
                self.obs_y = 0.0

            # 2. LEFT: [+45°, +135°] ~ [0.785, 2.356] rad
            left_mask = valid_mask & (angles > 0.785) & (angles < 2.356)
            self.left_dist = float(np.min(ranges[left_mask])) if np.any(left_mask) else 999.0

            # 3. RIGHT: [-135°, -45°] ~ [-2.356, -0.785] rad
            right_mask = valid_mask & (angles > -2.356) & (angles < -0.785)
            self.right_dist = float(np.min(ranges[right_mask])) if np.any(right_mask) else 999.0

            # 4. REAR: [<-135° hoặc >+135°] ~ |angles| >= 2.356 rad
            rear_mask = valid_mask & (np.abs(angles) >= 2.356)
            self.rear_dist = float(np.min(ranges[rear_mask])) if np.any(rear_mask) else 999.0

        except Exception as e:
            self.get_logger().warn(f"Lỗi xử lý LiDAR 360: {e}")

    def check_surrounding_safety(self):
        """
        Kiểm tra toàn diện điều kiện an toàn phía sau và 2 bên xe:
        - Phía sau : self.rear_dist >= 0.50m (Không có xe bám đuôi quá sát)
        - Làn trái : self.left_dist >= 0.45m (Làn vượt thông thoáng, đủ điều kiện chuyển làn)
        - Phía phải: self.right_dist >= 0.20m (Khoảng cách an toàn biên đường bên phải)
        """
        rear_safe = self.rear_dist >= 0.50
        left_safe = self.left_dist >= 0.45
        right_safe = self.right_dist >= 0.20

        is_safe = rear_safe and left_safe and right_safe
        details = (
            f"Sau: {self.rear_dist:.2f}m ({'OK' if rear_safe else 'GẦN'}) | "
            f"Trái: {self.left_dist:.2f}m ({'OK' if left_safe else 'CẢN'}) | "
            f"Phải: {self.right_dist:.2f}m ({'OK' if right_safe else 'HẸP'})"
        )
        self.surrounding_safety_details = details
        return is_safe, details

    def camera_callback(self, msg: Image):
        """
        Nhận diện làn đường với Camera góc cao (Elevated Camera & Road Lookahead):
        - Tầm nhìn xa mở rộng nhờ nâng góc camera: nhận diện cả vùng GẦN (near lookahead)
          để tính sai số lệch làn ey và vùng XA (far lookahead) để dự báo độ cong góc hướng
        - Bên TRÁI: Vạch vàng tim đường (y=0.0m)
        - Bên PHẢI: Vạch trắng biên đường (y=-0.50m)
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape

            # 1. Trích xuất ROI mở rộng: Từ 32% đến 95% chiều cao ảnh (tầm nhìn xa tới 15-20m)
            roi_y_start = int(h * 0.32)
            roi_y_end = int(h * 0.95)
            roi = cv_image[roi_y_start:roi_y_end, :]
            roi_h = roi.shape[0]

            # Chuyển sang không gian màu HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 2. Mặt nạ lọc màu: Vàng (tim đường) và Trắng (biên đường)
            yellow_mask = cv2.inRange(hsv, np.array([12, 60, 80]), np.array([42, 255, 255]))
            white_mask = cv2.inRange(hsv, np.array([0, 0, 140]), np.array([180, 75, 255]))

            car_center_x = w / 2.0  # Trục giữa của xe (320px)

            # 3. Phân đoạn 2 dải quan sát: VÙNG GẦN (Near) & VÙNG XA (Far Lookahead)
            # Vùng Gần (55% - 95% ROI): Tính sai số lệch ngang tức thời e_y
            near_y1, near_y2 = int(roi_h * 0.55), int(roi_h * 0.95)
            near_yellow = yellow_mask[near_y1:near_y2, :int(w * 0.65)]
            near_white = white_mask[near_y1:near_y2, int(w * 0.35):]

            # Vùng Xa (10% - 45% ROI): Dự báo trước hướng đường (Lookahead)
            far_y1, far_y2 = int(roi_h * 0.10), int(roi_h * 0.45)
            far_yellow = yellow_mask[far_y1:far_y2, :int(w * 0.65)]
            far_white = white_mask[far_y1:far_y2, int(w * 0.35):]

            # --- Xử lý vùng Gần ---
            y_pts_near = cv2.findNonZero(near_yellow)
            w_pts_near = cv2.findNonZero(near_white)
            x_y_near = float(np.mean(y_pts_near[:, :, 0])) if (y_pts_near is not None and len(y_pts_near) > 25) else None
            x_w_near = float(np.mean(w_pts_near[:, :, 0])) + int(w * 0.35) if (w_pts_near is not None and len(w_pts_near) > 25) else None

            if x_y_near is not None and x_w_near is not None:
                near_lane_center = (x_y_near + x_w_near) / 2.0
                detected_half_w = (x_w_near - x_y_near) / 2.0
                if 40 < detected_half_w < 160:
                    self.lane_half_width_px = int(detected_half_w)
                self.lane_detected = True
                self.lane_lost_counter = 0
            elif x_y_near is not None:
                near_lane_center = x_y_near + self.lane_half_width_px
                self.lane_detected = True
                self.lane_lost_counter = 0
            elif x_w_near is not None:
                near_lane_center = x_w_near - self.lane_half_width_px
                self.lane_detected = True
                self.lane_lost_counter = 0
            else:
                near_lane_center = car_center_x
                self.lane_detected = False
                if self.race_started and self.fsm_state == "NORMAL":
                    self.lane_lost_counter += 1

            # --- Xử lý vùng Xa (Lookahead) ---
            y_pts_far = cv2.findNonZero(far_yellow)
            w_pts_far = cv2.findNonZero(far_white)
            x_y_far = float(np.mean(y_pts_far[:, :, 0])) if (y_pts_far is not None and len(y_pts_far) > 20) else None
            x_w_far = float(np.mean(w_pts_far[:, :, 0])) + int(w * 0.35) if (w_pts_far is not None and len(w_pts_far) > 20) else None

            far_half_w = self.lane_half_width_px * 0.75  # Do phối cảnh xa, làn hẹp hơn
            if x_y_far is not None and x_w_far is not None:
                far_lane_center = (x_y_far + x_w_far) / 2.0
            elif x_y_far is not None:
                far_lane_center = x_y_far + far_half_w
            elif x_w_far is not None:
                far_lane_center = x_w_far - far_half_w
            else:
                far_lane_center = near_lane_center

            # 4. Tính sai số lệch ngang e_y
            pixel_offset = near_lane_center - car_center_x
            meters_per_pixel = 0.25 / max(float(self.lane_half_width_px), 50.0)
            measured_ey = float(pixel_offset * meters_per_pixel)

            if self.lane_detected:
                self.lateral_error = 0.75 * measured_ey + 0.25 * self.lateral_error

            # 5. Tính góc hướng dự báo trước từ Camera (Lookahead Heading Preview)
            delta_x_lookahead = (far_lane_center - near_lane_center) * meters_per_pixel
            lookahead_dist_m = 2.6
            self.lookahead_heading = float(np.arctan2(delta_x_lookahead, lookahead_dist_m))

            # Kết hợp góc IMU và góc dự báo camera để NMPC bám cua trước
            self.heading_error = float(self.imu_yaw - 0.20 * self.lookahead_heading)
            self.camera_received = True

            # 6. Vẽ ảnh Debug Camera trực quan
            debug_img = roi.copy()
            y_mid_near = int((near_y1 + near_y2) / 2)
            y_mid_far = int((far_y1 + far_y2) / 2)

            # Vẽ tâm xe (xanh dương)
            cv2.line(debug_img, (int(car_center_x), 0), (int(car_center_x), roi_h), (255, 0, 0), 2)
            # Vẽ tia dự báo tâm làn từ vùng gần ra vùng xa (đường vàng nét đứt)
            cv2.line(debug_img, (int(near_lane_center), y_mid_near), (int(far_lane_center), y_mid_far), (0, 255, 255), 2)
            # Vẽ tâm làn gần (xanh lá) và tâm làn xa (cyan)
            cv2.circle(debug_img, (int(near_lane_center), y_mid_near), 7, (0, 255, 0), -1)
            cv2.circle(debug_img, (int(far_lane_center), y_mid_far), 5, (255, 255, 0), -1)

            # Vị trí các vạch kẻ đường
            if x_y_near is not None:
                cv2.circle(debug_img, (int(x_y_near), y_mid_near), 6, (0, 255, 255), -1)
            if x_w_near is not None:
                cv2.circle(debug_img, (int(x_w_near), y_mid_near), 6, (255, 255, 255), -1)

            # Băng thông tin LiDAR 360 độ trên cùng
            header_text = f"LiDAR: F:{self.front_dist:.1f}m | L:{self.left_dist:.1f}m | R:{self.right_dist:.1f}m | B:{self.rear_dist:.1f}m"
            cv2.putText(debug_img, header_text, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

            # Cảnh báo theo từng trạng thái máy FSM
            if not self.race_started:
                self.lane_lost_counter = 0
                cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], debug_img.shape[0]), (0, 165, 255), 4)
                cv2.putText(debug_img, "WAITING FOR START (race_commander_node)", (15, int(roi_h * 0.88)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 200, 255), 2)
            elif self.fsm_state == "SAFE_STOP_WAITING":
                elapsed = (self.get_clock().now() - self.safe_stop_start_time).nanoseconds * 1e-9 if self.safe_stop_start_time else 0.0
                cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], debug_img.shape[0]), (0, 140, 255), 5)
                cv2.putText(debug_img, f"SAFE STOP: Cho 5s danh gia an toan [{elapsed:.1f}/5.0s]", (15, int(roi_h * 0.80)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 165, 255), 2)
                cv2.putText(debug_img, f"360 Check: {self.surrounding_safety_details}", (15, int(roi_h * 0.90)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            elif self.fsm_state == "OVERTAKING":
                cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], debug_img.shape[0]), (0, 255, 255), 4)
                cv2.putText(debug_img, "OVERTAKING: Chuyen sang lan trai vuot vat can", (15, int(roi_h * 0.88)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
            elif self.fsm_state == "RETURNING":
                cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], debug_img.shape[0]), (255, 100, 255), 4)
                cv2.putText(debug_img, "RETURNING: Da vuot xong. Dang tro ve lan phai", (15, int(roi_h * 0.88)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 100, 255), 2)
            elif self.safe_stop_active:
                cv2.rectangle(debug_img, (0, 0), (debug_img.shape[1], debug_img.shape[0]), (0, 0, 255), 6)
                cv2.putText(debug_img, f"SAFE STOP: {self.safe_stop_reason}", (15, int(roi_h * 0.88)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            else:
                status_color = (0, 255, 0) if self.lane_detected else (0, 165, 255)
                cv2.putText(
                    debug_img,
                    f"e_lat: {self.lateral_error:+.2f}m | Yaw: {np.degrees(self.imu_yaw):+.1f}deg | {'OK' if self.lane_detected else 'LOST'}",
                    (10, 42),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    status_color,
                    2
                )

            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            self.debug_img_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f"Lỗi xử lý ảnh Camera: {e}")

    def mode_callback(self, msg: Float32):
        target = msg.data
        if target < 0:
            self.emergency_brake = True
            self.v_ref = 0.0
            self.get_logger().warn("KÍCH HOẠT PHANH KHẨN CẤP TỪ LỆNH USER!")
        else:
            self.emergency_brake = False
            self.v_ref = target
            self.get_logger().info(f"Cập nhật tốc độ mục tiêu: {self.v_ref * 3.6:.1f} km/h")

    def race_start_callback(self, msg: Bool):
        """Nhận tín hiệu bắt đầu / dừng từ Race Commander (terminal)"""
        self.race_started = msg.data
        if self.race_started:
            self.safe_stop_active = False
            self.emergency_brake = False
            self.lane_lost_counter = 0
            self.fsm_state = "NORMAL"
            self.target_lateral_offset = 0.0
            self.current_cmd_v = 0.55
            self.get_logger().info("🏁 [RACE START] ĐÃ NHẬN LỆNH KHỞI HÀNH! Xe bắt đầu tự bám làn NMPC!")
        else:
            self.get_logger().warn("🛑 [TERMINAL STOP] ĐÃ NHẬN LỆNH DỪNG TỪ TERMINAL! Dừng xe khẩn cấp.")
            self.race_started = False
            self.fsm_state = "NORMAL"
            self.target_lateral_offset = 0.0
            self.current_cmd_v = 0.0
            self._publish_cmd(0.0, 0.0)
            safety_msg = String()
            safety_msg.data = "TERMINAL_STOP: Xe đã dừng an toàn theo lệnh từ terminal"
            self.safety_pub.publish(safety_msg)

    # ==================== BỘ GIẢI NMPC ====================
    def init_nmpc_solver(self):
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); theta = ca.SX.sym('theta'); v = ca.SX.sym('v')
        states = ca.vertcat(x, y, theta, v)
        n_states = states.numel()

        a = ca.SX.sym('a'); delta = ca.SX.sym('delta')
        controls = ca.vertcat(a, delta)
        n_controls = controls.numel()

        rhs = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            (v / self.L) * ca.tan(delta),
            a
        )
        f = ca.Function('f', [states, controls], [rhs])

        X = ca.SX.sym('X', n_states, self.N + 1)
        U = ca.SX.sym('U', n_controls, self.N)
        P = ca.SX.sym('P', n_states + 4) # [x0, y0, theta0, v0, y_ref, v_ref, obs_x, obs_y]

        obj = 0  
        g = []   

        # Trọng số chi phí tối ưu hóa
        Q_y = 50.0     # Trọng số bám chặt tim làn bên phải
        Q_theta = 20.0 # Trọng số triệt tiêu góc lệch xe
        Q_v = 3.0      # Duy trì tốc độ
        R_a = 0.1
        R_delta = 3.5  # Phạt lái gấp để xe chạy mượt
        W_obs = 35.0   # Né vật cản LiDAR

        g.append(X[:, 0] - P[:4])

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]

            obj += Q_y * (st[1] - P[4])**2 + Q_theta * (st[2])**2 + Q_v * (st[3] - P[5])**2
            obj += R_a * (con[0])**2 + R_delta * (con[1])**2

            dist_to_obs = ca.sqrt((st[0] - P[6])**2 + (st[1] - P[7])**2)
            obj += W_obs / (dist_to_obs**2 + 0.1)

            st_next = X[:, k+1]
            st_step = st + self.dt * f(st, con)
            g.append(st_next - st_step)

        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}
        
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes', 'ipopt.max_iter': 40}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        self.lbx = []
        self.ubx = []

        # Ràng buộc trạng thái an toàn trong NMPC:
        # Cho phép y trong khoảng [-0.65, +0.65]m để xe đủ không gian chuyển làn vượt vật cản
        for _ in range(self.N + 1):
            self.lbx += [-ca.inf, -0.65, -0.75, 0.0]
            self.ubx += [ ca.inf,  0.65,  0.75, 3.0]

        for _ in range(self.N):
            self.lbx += [-3.0, -0.45]  # Gia tốc [-3.0, 1.8] m/s², góc lái [-0.45, 0.45] rad
            self.ubx += [ 1.8,  0.45]

        self.lbg = [0.0] * ((self.N + 1) * n_states)
        self.ubg = [0.0] * ((self.N + 1) * n_states)

    # ==================== CƠ CHẾ SAFE STOP & VÒNG LẶP ĐIỀU KHIỂN ====================
    def check_safety_constraints(self):
        """
        Kiểm tra toàn diện các ràng buộc an toàn của xe:
        1. Lệch khỏi quỹ đạo cho phép
        2. Mất lái góc hướng
        3. Mất dấu vạch kẻ đường liên tục
        """
        violations = []

        # Trong trạng thái OVERTAKING hoặc RETURNING:
        # Xe đang chủ động chuyển làn (từ y=0m sang y=0.50m rồi về y=0m).
        # Chỉ báo vi phạm nếu xe lao ra khỏi ranh giới đường (> 0.68m) hoặc mất lái nghiêm trọng (> 40 độ)
        if self.fsm_state in ("OVERTAKING", "RETURNING"):
            if abs(self.lateral_error) > 0.68:
                violations.append(f"XE RA KHỎI ĐƯỜNG (|y| = {abs(self.lateral_error):.2f}m > 0.68m)")
            if abs(self.heading_error) > 0.70:
                violations.append(f"MẤT LÁI GÓC HƯỚNG KHI VƯỢT (|theta| = {np.degrees(abs(self.heading_error)):.1f}° > 40°)")
            if self.front_dist < 0.25:
                violations.append(f"NGUY CƠ VA CHẠM TRỰC DIỆN ({self.front_dist:.2f}m < 0.25m)")

        elif self.fsm_state == "NORMAL":
            # Khi bám làn bình thường bên phải:
            if abs(self.lateral_error) > self.max_allowed_lane_deviation:
                violations.append(f"LỆCH LÀN QUÁ MỨC (|e_y| = {abs(self.lateral_error):.2f}m > {self.max_allowed_lane_deviation:.2f}m)")
            if abs(self.heading_error) > self.max_allowed_heading_deviation:
                violations.append(f"MẤT LÁI GÓC HƯỚNG (|theta| = {np.degrees(abs(self.heading_error)):.1f}° > {np.degrees(self.max_allowed_heading_deviation):.1f}°)")
            if self.lane_lost_counter > self.max_lane_lost_frames:
                violations.append(f"MẤT TẦM NHÌN LÀN ĐƯỜNG ({self.lane_lost_counter} frames)")

        return violations

    def _publish_cmd(self, v_cmd, omega_cmd):
        """Xuất lệnh điều khiển ra controller và cmd_vel"""
        msg_stamped = TwistStamped()
        msg_stamped.header.stamp = self.get_clock().now().to_msg()
        msg_stamped.header.frame_id = 'base_link'
        msg_stamped.twist.linear.x = float(v_cmd)
        msg_stamped.twist.angular.z = float(omega_cmd)
        self.cmd_pub_stamped.publish(msg_stamped)

        msg_unstamped = Twist()
        msg_unstamped.linear.x = float(v_cmd)
        msg_unstamped.angular.z = float(omega_cmd)
        self.cmd_pub_unstamped.publish(msg_unstamped)
        self.cmd_vel_pub.publish(msg_unstamped)

    def control_loop(self):
        # 0. NẾU CHƯA CÓ LỆNH KHỞI CHẠY TỪ TERMINAL -> XE ĐỨNG YÊN HOÀN TOÀN
        if not self.race_started:
            self.current_cmd_v = 0.0
            safety_msg = String()
            safety_msg.data = "WAITING_FOR_START: Xe đang chờ lệnh START từ terminal..."
            self.safety_pub.publish(safety_msg)
            self._publish_cmd(0.0, 0.0)
            return

        safety_msg = String()
        now = self.get_clock().now()

        # Nếu có lệnh phanh khẩn cấp cưỡng bức từ người dùng (mode < 0)
        if self.emergency_brake:
            self.safe_stop_active = True
            self.safe_stop_reason = "PHANH KHẨN CẤP TỪ USER"
            safety_msg.data = "SAFE_STOP_ACTIVE: LỆNH PHANH KHẨN CẤP TỪ USER"
            self.safety_pub.publish(safety_msg)
            self.current_cmd_v = 0.0
            self._publish_cmd(0.0, 0.0)
            return

        # =========================================================================
        # 1. MÁY TRẠNG THÁI (FSM): BÁM LÀN - SAFE STOP 5S - ĐÁNH GIÁ 360° - VƯỢT VẬT CẢN
        # =========================================================================

        # --- TRẠNG THÁI 1: NORMAL (Bám làn bên phải) ---
        if self.fsm_state == "NORMAL":
            # Kiểm tra các điều kiện cần Safe Stop (Vật cản hoặc vi phạm an toàn)
            violations = self.check_safety_constraints()
            need_stop = False
            reason = ""

            if self.front_dist < self.safe_obstacle_dist and self.obs_x > 0.0:
                need_stop = True
                reason = f"VẬT CẢN PHÍA TRƯỚC {self.front_dist:.2f}m"
            elif len(violations) > 0:
                need_stop = True
                reason = " | ".join(violations)

            if need_stop:
                self.fsm_state = "SAFE_STOP_WAITING"
                self.safe_stop_start_time = now
                self.safe_stop_active = True
                self.safe_stop_reason = reason
                self.current_cmd_v = 0.0
                self.get_logger().warn(
                    f"\n🛑 [SAFE STOP KÍCH HOẠT]: {reason}!\n"
                    f"   Xe dừng an toàn tại chỗ và bắt đầu đếm ngược 5s đánh giá an toàn...\n"
                )
                safety_msg.data = f"SAFE_STOP_WAITING: {reason} [0.0/5.0s]"
                self.safety_pub.publish(safety_msg)
                self._publish_cmd(0.0, 0.0)
                return

        # --- TRẠNG THÁI 2: SAFE_STOP_WAITING (Dừng an toàn 5s & đánh giá xung quanh) ---
        if self.fsm_state == "SAFE_STOP_WAITING":
            self.safe_stop_active = True
            time_stopped = (now - self.safe_stop_start_time).nanoseconds * 1e-9 if self.safe_stop_start_time else 0.0
            is_safe, details = self.check_surrounding_safety()

            if time_stopped < self.safe_stop_duration:
                # Đang trong 5 giây dừng chờ an toàn: xe giữ đứng yên và đếm ngược
                self.safe_stop_reason = f"Dừng an toàn 5s. Đang quan sát 360° [{time_stopped:.1f}/5.0s]"
                safety_msg.data = f"SAFE_STOP_WAITING: {self.safe_stop_reason} ({details})"
                self.safety_pub.publish(safety_msg)

                self.current_cmd_v = 0.0
                self._publish_cmd(0.0, 0.0)
                return
            else:
                # =============================================================
                # ĐÃ ĐỦ 5 GIÂY ĐẾM NGƯỢC -> CHO XE TIẾP TỤC CHẠY!
                # =============================================================
                self.safe_stop_active = False
                self.lane_lost_counter = 0

                # Kiểm tra nếu phía trước có vật cản (< 1.2m) -> Đánh lái chuyển làn trái để vượt
                if self.front_dist < 1.2 or self.min_obstacle_dist < 1.2:
                    self.fsm_state = "OVERTAKING"
                    self.target_lateral_offset = 0.50 # Chuyển sang làn trái (+0.50m)
                    self.overtake_start_x = self.pos_x
                    self.current_cmd_v = 0.55 # Mồi vận tốc xuất phát để vượt ma sát tĩnh Gazebo

                    self.get_logger().info("\n" + "="*70)
                    self.get_logger().info(f"🚀 [HẾT 5S ĐÁNH GIÁ AN TOÀN]: {details}")
                    self.get_logger().info("   ==> BẮT ĐẦU CHO XE TIẾP TỤC CHẠY: ĐÁNH LÁI CHUYỂN LÀN TRÁI VƯỢT VẬT CẢN!")
                    self.get_logger().info("="*70 + "\n")
                    safety_msg.data = f"OVERTAKE_ACTIVE: Hết 5s -> Đánh lái chuyển làn vượt vật cản! ({details})"
                    self.safety_pub.publish(safety_msg)

                else:
                    # Phía trước thông thoáng -> Tiếp tục chạy thẳng bám làn phải
                    self.fsm_state = "NORMAL"
                    self.target_lateral_offset = 0.0
                    self.current_cmd_v = 0.55

                    self.get_logger().info("\n" + "="*70)
                    self.get_logger().info(f"🟢 [HẾT 5S ĐÁNH GIÁ AN TOÀN]: Đường thông thoáng ({details})")
                    self.get_logger().info("   ==> BẮT ĐẦU CHO XE TIẾP TỤC CHẠY BÁM LÀN!")
                    self.get_logger().info("="*70 + "\n")
                    safety_msg.data = f"STATUS_OK: Hết 5s -> Xe tiếp tục chạy bám làn ({details})"
                    self.safety_pub.publish(safety_msg)

        # --- TRẠNG THÁI 3: OVERTAKING (Chuyển làn sang trái và vượt vật cản) ---
        elif self.fsm_state == "OVERTAKING":
            dx = self.pos_x - self.overtake_start_x
            # Điều kiện vượt xong:
            # 1. Đã đi được quãng đường dx >= 2.2m
            # 2. VÀ khoảng cách bên phải xe thông thoáng >= 0.50m (vật cản đã ở sau đuôi xe)
            passed_obstacle = (dx >= 2.2 or (self.obs_x < 0.0 and dx >= 1.6)) and (self.right_dist >= 0.50)

            if passed_obstacle:
                self.fsm_state = "RETURNING"
                self.target_lateral_offset = 0.0 # Mục tiêu trở về làn bên phải
                self.get_logger().info("\n" + "="*70)
                self.get_logger().info(f"🔄 [VƯỢT XONG VẬT CẢN - dx={dx:.2f}m]: Làn phải thông thoáng (Phải={self.right_dist:.2f}m)!")
                self.get_logger().info("   ==> ĐÁNH LÁI TRỞ VỀ LÀN BÊN PHẢI VÀ TIẾP TỤC HÀNH TRÌNH!")
                self.get_logger().info("="*70 + "\n")
                safety_msg.data = "RETURNING_TO_LANE: Đã vượt xong vật cản. Đang đánh lái về làn phải."
                self.safety_pub.publish(safety_msg)
            else:
                safety_msg.data = f"OVERTAKING: Đang vượt ở làn trái (Tiến độ: {dx:.1f}m/2.2m | Phải: {self.right_dist:.2f}m)"
                self.safety_pub.publish(safety_msg)

        # --- TRẠNG THÁI 4: RETURNING (Đánh lái về lại làn phải) ---
        elif self.fsm_state == "RETURNING":
            # Khi xe đã căn chỉnh ổn định trong làn phải
            if abs(self.lateral_error) < 0.15 and abs(self.heading_error) < 0.20:
                self.fsm_state = "NORMAL"
                self.get_logger().info("✅ [HOÀN TẤT VƯỢT]: Đã về làn bên phải an toàn và ổn định. Tiếp tục bám làn đường!")
                safety_msg.data = "STATUS_OK: Bám làn phải ổn định sau khi vượt"
                self.safety_pub.publish(safety_msg)
            else:
                safety_msg.data = f"RETURNING: Đang căn chỉnh về tim làn phải (e_y={self.lateral_error:+.2f}m)"
                self.safety_pub.publish(safety_msg)

        # =========================================================================
        # 2. KIỂM TRA RÀNG BUỘC AN TOÀN TRONG KHI DI CHUYỂN
        # =========================================================================
        violations = self.check_safety_constraints()

        if len(violations) > 0:
            # Nếu vi phạm trong khi di chuyển -> chuyển về Safe Stop 5s để tự phục hồi
            self.fsm_state = "SAFE_STOP_WAITING"
            self.safe_stop_start_time = now
            self.safe_stop_active = True
            reason = " | ".join(violations)
            self.safe_stop_reason = reason
            safety_msg.data = f"SAFE_STOP_WAITING: {reason} [0.0/5.0s]"
            self.safety_pub.publish(safety_msg)

            self.current_cmd_v = 0.0
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(f"🛑 [SAFE STOP]: {reason} => Dừng 5s đánh giá an toàn...")
            return

        # =========================================================================
        # 3. GIẢI BÀI TOÁN TỐI ƯU NMPC VỚI DỰ BÁO QUỸ ĐẠO & CAMERA GÓC CAO
        # =========================================================================
        self.safe_stop_active = False
        current_ey = self.lateral_error
        current_etheta = self.heading_error + 0.08 * self.imu_yaw_rate
        current_v = max(self.current_v, 0.0)

        # Vận tốc mục tiêu: 1.2 m/s khi vượt/về làn, 1.5 m/s khi chạy thẳng bình thường
        active_v_ref = 1.2 if self.fsm_state in ("OVERTAKING", "RETURNING") else self.v_ref

        current_state = np.array([0.0, current_ey, current_etheta, current_v])
        p_val = np.concatenate([
            current_state,
            [self.target_lateral_offset, active_v_ref, self.obs_x, self.obs_y]
        ])
        x0 = np.zeros(((self.N + 1) * 4 + self.N * 2, 1))

        try:
            res = self.solver(x0=x0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=p_val)
            u_opt = res['x'][(self.N + 1) * 4:]
            a_cmd = float(u_opt[0])
            delta_cmd = float(u_opt[1])

            # Vẽ quỹ đạo tương lai lên RViz
            x_opt = res['x'][:(self.N + 1) * 4].full().reshape((4, self.N + 1))
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nmpc_trajectory"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            if self.fsm_state == "OVERTAKING":
                marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.0; marker.color.a = 0.95
            elif self.fsm_state == "RETURNING":
                marker.color.r = 0.6; marker.color.g = 0.2; marker.color.b = 1.0; marker.color.a = 0.95
            else:
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.2; marker.color.a = 0.90

            marker.points = []
            for k in range(self.N + 1):
                p = Point()
                p.x = float(x_opt[0, k])
                p.y = float(x_opt[1, k])
                p.z = 0.05
                marker.points.append(p)
            self.trajectory_pub.publish(marker)

        except Exception as e:
            self.get_logger().error(f"Lỗi bộ giải NMPC (Infeasible): {e} => Tạm dừng 0.5s")
            a_cmd = -2.0
            delta_cmd = 0.0

        # Tính toán vận tốc điều khiển
        self.current_cmd_v = float(np.clip(
            self.current_cmd_v + a_cmd * self.dt,
            0.0,
            active_v_ref
        ))
        # Đảm bảo vận tốc mồi để không bị kẹt ma sát tĩnh khi bắt đầu chạy
        if self.current_cmd_v < 0.35 and active_v_ref > 0.0:
            self.current_cmd_v = 0.45

        v_cmd = self.current_cmd_v
        v_calc = max(v_cmd, 0.35)
        omega_cmd = float((v_calc / self.L) * np.tan(delta_cmd))

        self._publish_cmd(v_cmd, omega_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
