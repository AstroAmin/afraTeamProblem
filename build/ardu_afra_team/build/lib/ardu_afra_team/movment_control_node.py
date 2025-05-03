import time
import random
import threading

class Drone:
    def __init__(self, drone_id, x=0, y=0, z=0):
        self.id = drone_id
        self.x = x
        self.y = y
        self.z = z
        self.active = True

    def move_towards(self, target_x, target_y, target_z, step_size):
        if not self.active:
            return
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        if distance == 0:
            return
        self.x += step_size * dx / distance
        self.y += step_size * dy / distance
        self.z += step_size * dz / distance

def print_drones(drones):
    print("+-------+---------+---------+---------+")
    print("| ID    |   X(m)  |   Y(m)  |   Z(m)  |")
    print("+-------+---------+---------+---------+")
    for drone in drones:
        if drone.active:
            print(f"|  {drone.id:^5} | {drone.x:>7.2f} | {drone.y:>7.2f} | {drone.z:>7.2f} |")
        else:
            print(f"|  {drone.id:^5} |   🚫    |   🚫    |   🚫    |")
    print("+-------+---------+---------+---------+")

def form_initial_formation(drones, formation_center_x, formation_center_y, formation_center_z, spacing):
    leader = drones[0]
    leader.x = formation_center_x
    leader.y = formation_center_y
    leader.z = formation_center_z
    drones[1].x = formation_center_x - spacing
    drones[1].y = formation_center_y - spacing
    drones[1].z = formation_center_z
    drones[2].x = formation_center_x
    drones[2].y = formation_center_y - spacing
    drones[2].z = formation_center_z
    drones[3].x = formation_center_x + spacing
    drones[3].y = formation_center_y - spacing
    drones[3].z = formation_center_z

def select_new_leader(drones):
    active_drones = [d for d in drones if d.active]
    if active_drones:
        new_leader = random.choice(active_drones)
        print(f"\n>>> New Leader: Drone {new_leader.id} 🚀\n")
        return new_leader
    else:
        print("No active drones left.")
        return None

# Shared variable for emergency command
command = "none"

def command_listener():
    global command
    while True:
        cmd = input("Emergency command? (d = remove leader / q = quit / s = stop): ").strip().lower()
        if cmd in ['d', 'q', 's']:
            command = cmd

def move_drones(drones, target_point, speed):
    global command
    step_time = 0.5
    step_size = speed * step_time

    while True:
        print_drones(drones)

        all_reached = True
        for drone in drones:
            if drone.active:
                dist = ((drone.x - target_point[0])**2 + (drone.y - target_point[1])**2 + (drone.z - target_point[2])**2) ** 0.5
                if dist > 0.5:
                    drone.move_towards(*target_point, step_size)
                    all_reached = False

        # Check command
        if command == 'd':
            drones[0].active = False
            new_leader = select_new_leader(drones)
            if new_leader:
                drones.remove(new_leader)
                drones.insert(0, new_leader)
            command = "none"

        elif command == 'q':
            print("Program terminated. 🚨")
            exit()

        elif command == 's':
            print("\nDrone movement stopped. 🛑")
            continue_move = input("Do you want to continue the movement? (yes/no): ").strip().lower()
            if continue_move == 'yes':
                new_coordinates = input("Enter the new coordinates (X Y Z): ").strip()
                try:
                    new_x, new_y, new_z = map(float, new_coordinates.split())
                    target_point = (new_x, new_y, new_z)
                except ValueError:
                    print("Invalid coordinates! Please try again. 🚫")
                    continue
            else:
                print("Movement stopped. 🛑")
                command = "none"
                continue

        if all_reached:
            break

        time.sleep(step_time)

# Function to validate input coordinates
def get_valid_coordinates(prompt):
    while True:
        try:
            coords = input(prompt)
            x, y, z = map(float, coords.split())
            if x < 0 or y < 0 or z < 0:  # Check if coordinates are negative
                print("Coordinates cannot be negative. Please try again. 🚫")
            else:
                return x, y, z
        except ValueError:
            print("Invalid input. Please enter three numbers (X Y Z). 🔴")

# Function to validate numerical input
def get_valid_float(prompt):
    while True:
        try:
            value = float(input(prompt))
            if value <= 0:  # Check if the number is non-positive
                print("The number must be greater than zero. Please try again. 🚫")
            else:
                return value
        except ValueError:
            print("Invalid input. Please enter a valid number. 🔴")

def main():
    global command

    # Get valid input for speed and spacing
    speed = get_valid_float("Enter the speed of the drones (meters per second): 🚁 ")
    spacing = get_valid_float("Enter the spacing between drones (meters): 📏 ")

    # Get coordinates for the second target point with input validation
    print("Enter the coordinates for the second point (X Y Z): 🛸 ")
    point2 = get_valid_coordinates("Coordinates for second point (X Y Z): 🚀 ")

    # Get coordinates for the third target point with input validation
    print("Enter the coordinates for the third point (X Y Z): 🛸 ")
    point3 = get_valid_coordinates("Coordinates for third point (X Y Z): 🚀 ")

    # Initial formation setup using Z from point2
    formation_z = point2[2]

    # Create drones
    drones = [Drone(drone_id=i+1) for i in range(4)]

    form_initial_formation(drones, 0, 0, formation_z, spacing)
    print("\nInitial formation set up 🚁:\n")
    print_drones(drones)

    # Start command listener thread
    threading.Thread(target=command_listener, daemon=True).start()

    time.sleep(2)

    print("\nMoving towards second point... 🛸\n")
    move_drones(drones, point2, speed)

    print("\nMoving towards third point... 🚀\n")
    move_drones(drones, point3, speed)

if __name__ == "__main__":
    main()
