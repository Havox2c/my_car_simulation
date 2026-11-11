#!/usr/bin/env python3
"""
Node: Stop Car - Lệnh dừng xe nhanh từ terminal
Mô tả: Gửi tín hiệu dừng khẩn cấp đến bộ điều khiển NMPC và controller xe.
Sử dụng:
    ros2 run my_car_simulation stop_car.py
Hoặc chạy script:
    ./stop_race.sh
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
import time
import sys


class StopCarNode(Node):
    def __init__(self):
        super().__init__('stop_car_node')

        # Publisher lệnh dừng tới NMPC Node
        self.stop_pub = self.create_publisher(Bool, '/race_start', 10)

        # Publisher lệnh vận tốc 0 trực tiếp tới controller
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

    def execute_stop(self):
        # Chờ 0.3s để DDS thiết lập kết nối tới các subscribers
        time.sleep(0.3)

        # 1. Gửi lệnh race_start = False tới NMPC Node
        msg = Bool()
        msg.data = False

        # 2. Tạo lệnh dừng vận tốc 0
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Phát liên tiếp 3 đợt để đảm bảo node NMPC và controller nhận được ngay lập tức
        for _ in range(3):
            self.stop_pub.publish(msg)
            self.cmd_pub_stamped.publish(twist_stamped)
            self.cmd_pub_unstamped.publish(twist)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = StopCarNode()
    print("\n" + "=" * 55)
    print("  🛑  ĐANG GỬI LỆNH DỪNG XE TỚI HỆ THỐNG NMPC...")
    node.execute_stop()
    print("  ✅  ĐÃ GỬI LỆNH DỪNG THÀNH CÔNG! XE ĐÃ DỪNG LẠI.")
    print("=" * 55 + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

