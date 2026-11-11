#!/usr/bin/env python3
"""
Node: Race Commander - Điều khiển khởi chạy bằng terminal
Mô tả: Node chờ tín hiệu xác nhận từ người dùng qua terminal trước khi
       cho phép NMPC bắt đầu điều khiển xe tự động.
       
       Sau khi chạy launch.py và simulation đã nạp xong, mở terminal mới và chạy:
           ros2 run my_car_simulation race_commander_node.py
       
       Rồi nhập lệnh và nhấn ENTER để xe bắt đầu tự chạy.

Lệnh hỗ trợ:
  START / GO / (ENTER trống)   => Khởi động NMPC điều khiển xe
  STOP / E / DUNG              => Dừng an toàn tức thì
  STATUS                       => Hiển thị trạng thái hiện tại
  QUIT / EXIT                  => Thoát Race Commander
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import sys
import time


class RaceCommanderNode(Node):
    def __init__(self):
        super().__init__('race_commander_node')

        # Publisher: Gửi lệnh START/STOP tới NMPC Node
        self.start_pub = self.create_publisher(Bool, '/race_start', 10)

        # Subscriber: Nhận trạng thái an toàn từ NMPC Node
        self.safety_sub = self.create_subscription(
            String,
            '/safety_status',
            self.safety_callback,
            10
        )

        # Trạng thái nội bộ
        self._active = True
        self._race_started = False
        self._last_safety_status = "Chờ dữ liệu từ NMPC..."
        self._safety_lock = threading.Lock()

        # Đảm bảo publisher kết nối xong trước khi vào vòng lặp nhập liệu
        time.sleep(1.0)

        # Chạy vòng lặp nhập liệu trong thread riêng để không block spin()
        self._input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info("Race Commander Node đã sẵn sàng. Đang chờ lệnh từ terminal...")

    def safety_callback(self, msg: String):
        """Nhận và lưu trạng thái an toàn từ NMPC"""
        with self._safety_lock:
            self._last_safety_status = msg.data
        # Nếu NMPC báo Safe Stop và xe đang chạy, thông báo cho người dùng
        if "SAFE_STOP_ACTIVE" in msg.data and self._race_started:
            reason = msg.data.replace("SAFE_STOP_ACTIVE: ", "")
            print(f"\n  ⚠️  [SAFE STOP TỰ ĐỘNG]: {reason}")
            print("  Nhập 'START' để thử khởi chạy lại hoặc 'QUIT' để thoát.")
            print(">> Lệnh: ", end="", flush=True)

    def _publish_start(self, start: bool):
        """Gửi lệnh START=True hoặc STOP=False tới NMPC"""
        msg = Bool()
        msg.data = start
        self.start_pub.publish(msg)
        self._race_started = start

    def _print_banner(self):
        """In banner chào mừng ra terminal"""
        print()
        print("=" * 65)
        print("  🏁  RACE COMMANDER - Hệ thống điều khiển xe tự động NMPC  🏁")
        print("=" * 65)
        print()
        print("  Xe đã sẵn sàng tại vị trí xuất phát (Làn PHẢI).")
        print()
        print("  DANH SÁCH LỆNH:")
        print("  ┌──────────────────────────────────────────────────────")
        print("  │  START / GO / [ENTER]  → Khởi động xe tự lái NMPC")
        print("  │  STOP  / S  / DUNG     → Dừng xe khẩn cấp ngay lập tức")
        print("  │  STATUS                → Xem trạng thái hệ thống")
        print("  │  QUIT  / EXIT          → Thoát Race Commander")
        print("  └──────────────────────────────────────────────────────")
        print()

    def input_loop(self):
        """Vòng lặp nhập lệnh từ terminal (chạy trong thread riêng)"""
        self._print_banner()

        while self._active:
            try:
                raw = input(">> Lệnh: ")
                cmd = raw.strip().upper()
            except EOFError:
                # Stdin bị đóng (không phải terminal tương tác)
                self.get_logger().warn("EOF phát hiện - Race Commander thoát.")
                break
            except KeyboardInterrupt:
                print("\n  Race Commander bị ngắt bằng Ctrl+C.")
                break

            # ========== XỬ LÝ LỆNH ==========
            if cmd in ("", "START", "GO", "BAT DAU", "BẮT ĐẦU", "RUN", "CHAY", "CHẠY"):
                print()
                print("  ✅  Lệnh KHỞI CHẠY đã được gửi!")
                print("  🚗  Xe bắt đầu tự lái theo NMPC...")
                print()
                self._publish_start(True)

            elif cmd in ("STOP", "S", "DUNG", "DỪNG", "PAUSE", "HALT", "BRAKE", "0", "E"):
                print()
                print("  🛑  Lệnh DỪNG XE KHẨN CẤP đã được gửi!")
                print("  🔴  NMPC đã nhận lệnh dừng. Xe phanh và dừng lại ngay lập tức.")
                print()
                self._publish_start(False)

            elif cmd == "STATUS":
                with self._safety_lock:
                    status = self._last_safety_status
                race_state = "🟢 ĐANG CHẠY" if self._race_started else "🔴 ĐỨNG YÊN"
                print()
                print(f"  Trạng thái Race Commander : {race_state}")
                print(f"  Trạng thái NMPC/An toàn   : {status}")
                print()

            elif cmd in ("QUIT", "EXIT", "THOAT", "THOÁT"):
                print()
                print("  👋  Race Commander thoát.")
                print()
                self._active = False
                break

            else:
                print(f"  ❓  Lệnh không hợp lệ: '{raw}'")
                print("       Gõ [ENTER] hoặc START để chạy, gõ STOP hoặc S để dừng xe.")


def main(args=None):
    rclpy.init(args=args)
    node = RaceCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

