import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
import math

class DroneFormation(Node):
    def __init__(self):
        super().__init__('drone_formation_control')

        # Initial positions for drones
        self.drones_positions = {
            1: [0.0, 0.0, 5.0],
            2: [2.0, 0.0, 5.0],
            3: [4.0, 0.0, 5.0],
            4: [6.0, 0.0, 5.0]
        }

        # Initializing the state of the drone
        self.current_state = State()

        # Set up MAVROS topics
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.set_mode_service = self.create_service(SetMode, '/mavros/set_mode', self.set_mode_callback)
        self.arming_service = self.create_service(CommandBool, '/mavros/cmd/arming', self.arm_service_callback)

        # Drone control loop parameters
        self.timer = self.create_timer(0.5, self.move_formation)  # Update every 0.5 seconds

    def state_callback(self, msg):
        self.current_state = msg

    def update_position(self, drone_id, dx, dy, dz):
        # Update position for the drone
        self.drones_positions[drone_id][0] += dx
        self.drones_positions[drone_id][1] += dy
        self.drones_positions[drone_id][2] += dz

    def set_drone_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pose_pub.publish(pose)

    def arm_drone(self, arm):
        if arm:
            rclpy.wait_for_service('/mavros/cmd/arming')
            try:
                self.arming_service(True)
                self.get_logger().info("Drone armed")
            except Exception as e:
                self.get_logger().error("Arming failed: %s" % e)

    def set_mode_callback(self, request, response):
        if self.current_state.mode != request.custom_mode:
            try:
                response.success = True
                self.get_logger().info(f"Mode set to {request.custom_mode}")
            except Exception as e:
                response.success = False
                self.get_logger().error(f"Failed to set mode: {e}")
        return response

    def arm_service_callback(self, request, response):
        self.arm_drone(request.value)
        response.success = True
        return response

    def move_formation(self):
        # Simulate movement of drones in formation
        for drone_id in self.drones_positions:
            # Update positions with a small increment
            self.update_position(drone_id, 0.2, 0.2, 0.1)

            # Set new position for the drone
            x, y, z = self.drones_positions[drone_id]
            self.set_drone_pose(x, y, z)

            self.get_logger().info(f"Drone {drone_id} Position: X={round(x, 2)}, Y={round(y, 2)}, Z={round(z, 2)}")

        # Print updated drone positions in a nice table format
        self.print_drone_positions()

    def print_drone_positions(self):
        print("\nUpdated Drone Positions:")
        print("+-------+---------+---------+---------+")
        print("| ID    |   X(m)  |   Y(m)  |   Z(m)  |")
        print("+-------+---------+---------+---------+")
        for drone_id, position in self.drones_positions.items():
            print(f"|   {drone_id:<2} |  {round(position[0], 2):<7} |  {round(position[1], 2):<7} |  {round(position[2], 2):<7} |")
        print("+-------+---------+---------+---------+\n")

    def calculate_distance(self, drone_id1, drone_id2):
        x1, y1, z1 = self.drones_positions[drone_id1]
        x2, y2, z2 = self.drones_positions[drone_id2]
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        return round(distance, 2)

def main(args=None):
    rclpy.init(args=args)
    drone_formation = DroneFormation()
    rclpy.spin(drone_formation)
    drone_formation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
