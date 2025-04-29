import time
import random
import os

class Drone:
    def __init__(self, drone_id, x=0, y=0, z=0):
        self.id = drone_id
        self.x = x
        self.y = y
        self.z = z
        self.active = True  # برای حذف لیدر

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

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

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

def form_initial_formation(drones, formation_center_x, formation_center_y, altitude, spacing):
    # لیدر جلو، بقیه پشت سرش
    leader = drones[0]
    leader.x = formation_center_x
    leader.y = formation_center_y
    leader.z = altitude
    drones[1].x = formation_center_x - spacing
    drones[1].y = formation_center_y - spacing
    drones[1].z = altitude
    drones[2].x = formation_center_x
    drones[2].y = formation_center_y - spacing
    drones[2].z = altitude
    drones[3].x = formation_center_x + spacing
    drones[3].y = formation_center_y - spacing
    drones[3].z = altitude

def select_new_leader(drones):
    active_drones = [d for d in drones if d.active]
    if active_drones:
        new_leader = random.choice(active_drones)
        print(f"\n>>> لیدر جدید: درون {new_leader.id}\n")
        return new_leader
    else:
        print("هیچ درون فعالی باقی نمانده است.")
        return None

def main():
    # دریافت پارامترها
    num_drones = 4
    speed = float(input("سرعت حرکت درون‌ها (متر بر ثانیه): "))
    spacing = float(input("فاصله بین درون‌ها (متر): "))
    altitude = float(input("ارتفاع پروازی (متر): "))

    point2 = list(map(float, input("مختصات نقطه دوم (X Y Z): ").split()))
    point3 = list(map(float, input("مختصات نقطه سوم (X Y Z): ").split()))

    # ایجاد درون‌ها
    drones = [Drone(drone_id=i+1) for i in range(num_drones)]

    # چینش اولیه
    form_initial_formation(drones, 0, 0, altitude, spacing)
    clear_screen()
    print("آرایش اولیه شکل گرفت:\n")
    print_drones(drones)

    time.sleep(2)

    # پرواز به سمت نقطه ۲
    print("\nحرکت به سمت نقطه دوم...\n")
    move_drones(drones, point2, speed)

    # پرواز به سمت نقطه ۳
    print("\nحرکت به سمت نقطه سوم...\n")
    move_drones(drones, point3, speed)

def move_drones(drones, target_point, speed):
    step_time = 0.5  # نیم ثانیه
    step_size = speed * step_time  # میزان حرکت در هر نیم ثانیه

    while True:
        clear_screen()
        print_drones(drones)

        all_reached = True
        for drone in drones:
            if drone.active:
                dist = ((drone.x - target_point[0])**2 + (drone.y - target_point[1])**2 + (drone.z - target_point[2])**2) ** 0.5
                if dist > 0.5:  # اگر به هدف خیلی نزدیک نشده بود
                    drone.move_towards(*target_point, step_size)
                    all_reached = False

        time.sleep(step_time)

        # هر ۴ ثانیه یک بار سوال بپرسد
        if int(time.time()) % 4 == 0:
            user_input = input("\nدستور؟ (Enter برای ادامه / d برای حذف لیدر / q برای خروج): ").strip().lower()
            if user_input == 'd':
                # حذف لیدر فعلی
                leader = drones[0]
                leader.active = False
                new_leader = select_new_leader(drones)
                if new_leader:
                    # جابجایی لیدر به ابتدای لیست
                    drones.remove(new_leader)
                    drones.insert(0, new_leader)
            elif user_input == 'q':
                print("برنامه متوقف شد.")
                exit()

        if all_reached:
            break

if __name__ == "__main__":
    main()
