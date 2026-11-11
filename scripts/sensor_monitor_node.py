#!/usr/bin/env python3
"""
Node: Sensor Monitor Node
Mô tả: Giám sát toàn diện trạng thái cảm biến: Camera (Làn đường), IMU (Yaw, Gia tốc xoay),
LiDAR (Khoảng cách vật cản), Odometry và Trạng thái SAFE STOP của NMPC.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import String
import numpy as np

class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__('sensor_monitor_node')
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.odom_callback,
            qos_policy
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_policy
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_policy
        )
        self.cam_sub = self.create_subscription(
            Image,
            '/camera/lane_debug',
            self.cam_callback,
            10
        )
        self.safety_sub = self.create_subscription(
            String,
            '/safety_status',
            self.safety_callback,
            10
        )
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.current_v = 0.0
        self.current_yaw = 0.0
        self.imu_yaw_deg = 0.0
        self.yaw_rate = 0.0
        self.min_obstacle_dist = 999.0
        self.camera_frames = 0
        self.camera_status = "Đang chờ ảnh..."
        self.safety_status = "Đang khởi tạo hệ thống an toàn..."

        self.front_dist = 999.0
        self.left_dist = 999.0
        self.right_dist = 999.0
        self.rear_dist = 999.0

        self.monitor_timer = self.create_timer(0.5, self.print_sensor_status)
        self.get_logger().info("=> Sensor Monitor Node đã sẵn sàng giám sát toàn diện cảm biến!")

    def odom_callback(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.current_v = msg.twist.twist.linear.x

    def imu_callback(self, msg: Imu):
        self.yaw_rate = msg.angular_velocity.z
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.imu_yaw_deg = float(np.degrees(self.current_yaw))

    def lidar_callback(self, msg: LaserScan):
        total = len(msg.ranges)
        if total > 0:
            ranges = np.array(msg.ranges)
            angles = msg.angle_min + np.arange(total) * msg.angle_increment
            valid = (~np.isinf(ranges)) & (~np.isnan(ranges)) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

            # Front: [-45°, +45°]
            front_mask = valid & (np.abs(angles) <= 0.785)
            self.front_dist = float(np.min(ranges[front_mask])) if np.any(front_mask) else 999.0
            self.min_obstacle_dist = self.front_dist

            # Left: [+45°, +135°]
            left_mask = valid & (angles > 0.785) & (angles < 2.356)
            self.left_dist = float(np.min(ranges[left_mask])) if np.any(left_mask) else 999.0

            # Right: [-135°, -45°]
            right_mask = valid & (angles > -2.356) & (angles < -0.785)
            self.right_dist = float(np.min(ranges[right_mask])) if np.any(right_mask) else 999.0

            # Rear: |angles| >= 135°
            rear_mask = valid & (np.abs(angles) >= 2.356)
            self.rear_dist = float(np.min(ranges[rear_mask])) if np.any(rear_mask) else 999.0

    def cam_callback(self, msg: Image):
        self.camera_frames += 1
        self.camera_status = f"ĐANG HOẠT ĐỘNG (Frame #{self.camera_frames} | Góc cao mở rộng)"

    def safety_callback(self, msg: String):
        self.safety_status = msg.data

    def print_sensor_status(self):
        f_str = f"{self.front_dist:.2f}m" if self.front_dist < 10.0 else ">10m"
        l_str = f"{self.left_dist:.2f}m" if self.left_dist < 10.0 else ">10m"
        r_str = f"{self.right_dist:.2f}m" if self.right_dist < 10.0 else ">10m"
        b_str = f"{self.rear_dist:.2f}m" if self.rear_dist < 10.0 else ">10m"

        if "WAITING_FOR_START" in self.safety_status or "SAFE_STOP_WAITING" in self.safety_status:
            status_icon = "⏳"
        elif "OVERTAKE" in self.safety_status:
            status_icon = "🚀"
        elif "RETURNING" in self.safety_status:
            status_icon = "🔄"
        elif "SAFE_STOP" in self.safety_status:
            status_icon = "🛑"
        else:
            status_icon = "🟢"
        
        self.get_logger().info(
            f"\n================ BẢNG GIÁM SÁT TRẠNG THÁI CẢM BIẾN & AN TOÀN ================\n"
            f"  {status_icon} [TRẠNG THÁI NMPC]   : {self.safety_status}\n"
            f"  📷 [CAMERA GÓC CAO]   : {self.camera_status}\n"
            f"  🧭 [IMU GÓC QUANH Z]  : Yaw = {self.imu_yaw_deg:+.2f}° | YawRate = {self.yaw_rate:+.3f} rad/s\n"
            f"  📡 [LiDAR 360 4 HƯỚNG]: Trước: {f_str} | Trái: {l_str} | Phải: {r_str} | Sau: {b_str}\n"
            f"  🚗 [ODOM CONTROLLER]  : Vận tốc = {self.current_v:.2f} m/s | Tọa độ (x={self.pos_x:.2f}m, y={self.pos_y:.2f}m)\n"
            f"=============================================================================="
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
