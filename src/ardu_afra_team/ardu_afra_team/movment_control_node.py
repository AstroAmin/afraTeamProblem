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
            print(f"|  {drone.id:^5} |   ❌    |   ❌    |   ❌    |")
    print("+-------+---------+---------+---------+")

def go_to_altitude(drones, target_z, step_size):
    all_reached = False
    while not all_reached:
        all_reached = True
        for drone in drones:
            if drone.active and abs(drone.z - target_z) > 0.1:
                drone.move_towards(drone.x, drone.y, target_z, step_size)
                all_reached = False
        print_drones(drones)
        time.sleep(0.5)

def form_arrow_formation(drones, spacing):
    # موقعیت پیکانی در سطح Z=2
    drones[0].x, drones[0].y, drones[0].z = 0, 0, 2
    drones[1].x, drones[1].y, drones[1].z = -spacing, -spacing, 2
    drones[2].x, drones[2].y, drones[2].z = 0, -spacing, 2
    drones[3].x, drones[3].y, drones[3].z = spacing, -spacing, 2

def select_new_leader(drones):
    active_drones = [d for d in drones if d.active]
    if active_drones:
        new_leader = random.choice(active_drones)
        print(f"\n>>> لیدر جدید: درون {new_leader.id}\n")
        return new_leader
    else:
        print("هیچ درون فعالی باقی نمانده است.")
        return None

command = "none"
def command_listener():
    global command
    while True:
        cmd = input("دستور اضطراری؟ (d = حذف لیدر / q = خروج / s = توقف): ").strip().lower()
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

        # بررسی دستور
        if command == 'd':
            drones[0].active = False
            new_leader = select_new_leader(drones)
            if new_leader:
                drones.remove(new_leader)
                drones.insert(0, new_leader)
            command = "none"

        elif command == 'q':
            print("برنامه متوقف شد.")
            exit()

        elif command == 's':
            input("درون‌ها متوقف شدند. برای ادامه Enter را بزنید.")
            command = "none"

        if all_reached:
            break

        time.sleep(step_time)

def main():
    global command

    num_drones = 4
    speed = float(input("سرعت حرکت درون‌ها (متر بر ثانیه): "))
    spacing = float(input("فاصله بین درون‌ها (متر): "))

    print("مختصات نقطه دوم (X Y Z): ")
    point2 = list(map(float, input().split()))

    print("مختصات نقطه سوم (X Y Z): ")
    point3 = list(map(float, input().split()))

    drones = [Drone(drone_id=i+1, x=random.uniform(-5, 5), y=random.uniform(-5, 5), z=0) for i in range(num_drones)]

    print("\nدرون‌ها از مکان تصادفی شروع می‌کنند. ابتدا رفتن به ارتفاع ۲ متر...\n")
    step_size = speed * 0.5
    go_to_altitude(drones, 2, step_size)

    print("\nدر حال شکل‌گیری آرایش پیکانی...\n")
    form_arrow_formation(drones, spacing)
    print_drones(drones)

    threading.Thread(target=command_listener, daemon=True).start()
    time.sleep(2)

    print("\nحرکت به سمت نقطه دوم...\n")
    move_drones(drones, point2, speed)

    print("\nحرکت به سمت نقطه سوم...\n")
    move_drones(drones, point3, speed)

if __name__ == "__main__":
    main()
