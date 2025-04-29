import rclpy
from rclpy.node import Node
import os

class MenuNode(Node):
    def __init__(self):
        super().__init__('menu_node')
        self.get_logger().info("📋 Menu Node started")
        self.run_menu()

    def run_menu(self):
        while rclpy.ok():
            print("\n========== Drone Control Menu ==========")
            print("1. Start Shape Control Node")  # فقط نود shape_control_node.py
            print("2. Start Control Node")       # اضافه کردن گزینه نود کنترل
            print("3. Exit")
            print("========================================")

            choice = input("👉 Please select an option (1-3): ")  # تغییر شماره‌گذاری

            if choice == '1':
                self.run_node('shape_control_node')  # فراخوانی نود shape_control_node
            elif choice == '2':
                self.run_node('control')            # فراخوانی نود control (نام صحیح نود)
            elif choice == '3':
                print("👋 Exiting menu.")
                rclpy.shutdown()
                break
            else:
                print("❌ Invalid option. Try again.")

    def run_node(self, node_name):
        self.get_logger().info(f"🚀 Running node: {node_name}")
        os.system(f"gnome-terminal -- bash -c 'source ~/ros2_px4_offboard_example_ws/install/setup.bash && ros2 run px4_offboard {node_name}; exec bash'")

def main(args=None):
    rclpy.init(args=args)
    menu_node = MenuNode()
    rclpy.spin(menu_node)
    menu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
