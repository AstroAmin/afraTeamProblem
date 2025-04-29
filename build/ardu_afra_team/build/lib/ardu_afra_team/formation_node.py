import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from pynput import keyboard

# تنظیمات فاصله پیش‌فرض بین پهپادها
DISTANCE = 5.0

# انواع فرمیشن‌ها
class FormationStyles:
    TRIANGLE = "triangle"          # مثلثی
    COLUMN = "column"              # ستونی
    LINE_3D = "line_3D"           # خط سه‌بعدی
    STRAIGHT_PATH = "straight_path"  # مسیر مستقیم
    QUAD_SHAPE = "quad_shape"      # چهارضلعی
    A_TO_B_TO_C = "A_to_B_to_C"    # مسیر A به B به C

# تعریف کلاس پهپاد
class Drone:
    def __init__(self, identifier, shade, node):
        self.identifier = identifier
        self.shade = shade
        self.position = np.array([0.0, 0.0, 20.0])
        self.publisher = node.create_publisher(Pose, f'/{identifier}/pose', 10)

    def update_location(self, pos):
        """به‌روزرسانی موقعیت پهپاد"""
        self.position = pos
        msg = Pose()
        msg.position.x = float(pos[0])
        msg.position.y = float(pos[1])
        msg.position.z = float(pos[2])
        self.publisher.publish(msg)

# مدیریت حرکت و فرمیشن‌ها
class SwarmManager:
    def __init__(self, node, drones):
        self.node = node
        self.drones = drones
        self.central_point = np.array([0.0, 0.0, 20.0])
        self.formation_style = FormationStyles.TRIANGLE  # فرمیشن پیش‌فرض
        self.is_vertical = False  # حالت عمودی یا افقی
        self.leader = drones[0]  # پهپاد رهبر
        self.followers = drones[1:]  # پهپادهای دنبال‌کننده

        # نقاط مسیر A به B به C
        self.origin = np.array([0.0, 0.0, 20.0])
        self.intermediate = np.array([10.0, 10.0, 20.0])
        self.final_dest = np.array([20.0, 0.0, 20.0])
        self.target_point = self.origin

    def calculate_offsets(self):
        """محاسبه آفست‌های مربوط به هر فرمیشن"""
        if self.formation_style == FormationStyles.TRIANGLE:
            if self.is_vertical:
                return [
                    np.array([0, 0, 0]),
                    np.array([DISTANCE, 0, DISTANCE]),
                    np.array([-DISTANCE / 2, 0, DISTANCE * np.sqrt(3) / 2]),
                    np.array([-DISTANCE / 2, 0, -DISTANCE * np.sqrt(3) / 2])
                ]
            else:
                return [
                    np.array([0, 0, 0]),
                    np.array([DISTANCE, 0, 0]),
                    np.array([-DISTANCE / 2, DISTANCE * np.sqrt(3) / 2, 0]),
                    np.array([-DISTANCE / 2, -DISTANCE * np.sqrt(3) / 2, 0])
                ]
        elif self.formation_style == FormationStyles.COLUMN:
            if self.is_vertical:
                return [
                    np.array([0, 0, 0]),
                    np.array([0, 0, DISTANCE]),
                    np.array([0, 0, 2 * DISTANCE]),
                    np.array([0, 0, 3 * DISTANCE])
                ]
            else:
                return [
                    np.array([0, 0, 0]),
                    np.array([DISTANCE, 0, 0]),
                    np.array([2 * DISTANCE, 0, 0]),
                    np.array([3 * DISTANCE, 0, 0])
                ]
        elif self.formation_style == FormationStyles.LINE_3D:
            if self.is_vertical:
                return [
                    np.array([0, 0, 0]),
                    np.array([0, DISTANCE, 0]),
                    np.array([0, 2 * DISTANCE, 0]),
                    np.array([0, 3 * DISTANCE, 0])
                ]
            else:
                return [
                    np.array([0, 0, 0]),
                    np.array([DISTANCE, 0, 0]),
                    np.array([2 * DISTANCE, 0, 0]),
                    np.array([3 * DISTANCE, 0, 0])
                ]
        elif self.formation_style == FormationStyles.A_TO_B_TO_C:
            return [
                self.origin,
                self.intermediate,
                self.final_dest
            ]

    def update_formation(self):
        """به‌روزرسانی موقعیت‌های پهپادها"""
        offsets = self.calculate_offsets()
        if self.formation_style == FormationStyles.A_TO_B_TO_C:
            self.advance_to_next_target()

        self.leader.update_location(self.central_point)

        for i, follower in enumerate(self.followers):
            follower_position = self.leader.position + offsets[i]
            follower.update_location(follower_position)

    def advance_to_next_target(self):
        """حرکت از یک نقطه به نقطه بعدی در مسیر A به B به C"""
        distance_to_target = np.linalg.norm(self.central_point - self.target_point)

        if distance_to_target < DISTANCE:
            if self.target_point is self.origin:
                self.target_point = self.intermediate
            elif self.target_point is self.intermediate:
                self.target_point = self.final_dest
            else:
                return

        direction = self.target_point - self.central_point
        direction = direction / np.linalg.norm(direction)
        self.central_point += direction * DISTANCE

# شبیه‌سازی و گره ROS برای مدیریت پهپادها
class DroneSimulation(Node):
    def __init__(self):
        super().__init__('drone_simulation')

        # لیست پهپادها
        self.drones = [
            Drone("drone1", 'red', self),
            Drone("drone2", 'blue', self),
            Drone("drone3", 'green', self),
            Drone("drone4", 'orange', self),
        ]
        self.manager = SwarmManager(self, self.drones)

        # تایمر برای به‌روزرسانی موقعیت پهپادها
        self.timer = self.create_timer(0.1, self.update_loop)
        self.get_logger().info("Simulation node has started.")

        # شروع شنود کیبورد
        self.listener = keyboard.Listener(on_press=self.handle_key_press)
        self.listener.start()

    def handle_key_press(self, key):
        """پردازش ورودی‌های کیبورد برای تغییرات فرمیشن و حرکت پهپادها"""
        try:
            if key.char == '1':
                self.manager.formation_style = FormationStyles.TRIANGLE
            elif key.char == '2':
                self.manager.formation_style = FormationStyles.COLUMN
            elif key.char == '3':
                self.manager.formation_style = FormationStyles.LINE_3D
            elif key.char == '4':
                self.manager.formation_style = FormationStyles.STRAIGHT_PATH
            elif key.char == '5':
                self.manager.formation_style = FormationStyles.QUAD_SHAPE
            elif key.char == '6':
                self.manager.formation_style = FormationStyles.A_TO_B_TO_C
            elif key.char == 'v':
                self.manager.is_vertical = not self.manager.is_vertical
                print(f"Vertical Mode: {self.manager.is_vertical}")
            elif key.char == 'w':
                self.manager.central_point[1] += DISTANCE
            elif key.char == 's':
                self.manager.central_point[1] -= DISTANCE
            elif key.char == 'a':
                self.manager.central_point[0] -= DISTANCE
            elif key.char == 'd':
                self.manager.central_point[0] += DISTANCE
        except AttributeError:
            pass

    def update_loop(self):
        """به‌روزرسانی موقعیت پهپادها در شبیه‌ساز"""
        self.manager.update_formation()

# تابع اصلی برای راه‌اندازی ROS و شبیه‌سازی
def main(args=None):
    rclpy.init(args=args)
    node = DroneSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
