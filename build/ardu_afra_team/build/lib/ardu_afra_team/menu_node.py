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
            print("1. Start Shape Control Node")
            print("2. Start Manual Control Node")
            print("3. Exit")
            print("========================================")

            choice = input("👉 Please select an option (1-3): ")

            if choice == '1':
                self.run_node('shape_control_node')
            elif choice == '2':
                self.run_node('control')
            elif choice == '3':
                print("👋 Exiting menu.")
                rclpy.shutdown()
                break
            else:
                print("❌ Invalid option. Try again.")

    def run_node(self, node_name):
        self.get_logger().info(f"🚀 Running node: {node_name}")
        os.system(f"gnome-terminal -- bash -c 'source ~/ros2_ardu_afra_team_ws/install/setup.bash && ros2 run ardu_afra_team {node_name}; exec bash'")

def main(args=None):
    rclpy.init(args=args)
    node = MenuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
