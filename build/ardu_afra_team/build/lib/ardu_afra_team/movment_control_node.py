import time
import math
import os
import threading

# تعریف کلاس درون
class Drone:
    def __init__(self, drone_id, x=0, y=0, z=5.0):  # ارتفاع پیش‌فرض ۵ متر
        self.id = drone_id
        self.x = x
        self.y = y
        self.z = z
        self.active = True
        self.leader = False  # فیلد برای نشان دادن لیدر بودن

    def move_towards(self, target_x, target_y, target_z, step_size):
        if not self.active:
            return
        # محاسبه تغییرات در هر سه محور X, Y, Z
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance == 0:
            return
        # حرکت در سه بعد (X, Y, Z)
        self.x += step_size * dx / distance
        self.y += step_size * dy / distance
        self.z += step_size * dz / distance  # تغییرات در محور Z هم اعمال میشه

# نمایش مختصات به‌روز شده در هر گام
def print_drones(drones):
    os.system('cls' if os.name == 'nt' else 'clear')  # پاک کردن صفحه
    print("+-------+---------+---------+---------+")
    print("| ID    |   X(m)  |   Y(m)  |   Z(m)  |")
    print("+-------+---------+---------+---------+")
    for drone in drones:
        if drone.active:
            print(f"|  {drone.id:^5} | {drone.x:>7.2f} | {drone.y:>7.2f} | {drone.z:>7.2f} |")
        else:
            print(f"|  {drone.id:^5} |   ❌    |   ❌    |   ❌    |")
    print("+-------+---------+---------+---------+")

# تابع برای حرکت درون‌ها
def move_drones(drones, target_point, speed):
    step_time = 0.5  # زمان گام
    step_size = speed * step_time  # گام حرکت در هر فریم
    while True:
        all_reached = True
        for drone in drones:
            if drone.active:
                # اگر درون لیدر باشد، به سمت هدف حرکت می‌کند
                if drone.leader:
                    dist = ((drone.x - target_point[0])**2 + (drone.y - target_point[1])**2 + (drone.z - target_point[2])**2) ** 0.5
                    if dist > 0.5:  # حداقل فاصله که نشان‌دهنده رسیدن به نقطه است
                        drone.move_towards(*target_point, step_size)
                        all_reached = False
                # اگر درون فالوئر باشد، به سمت لیدر حرکت می‌کند
                else:
                    leader = next(d for d in drones if d.leader)
                    dist_to_leader = ((drone.x - leader.x)**2 + (drone.y - leader.y)**2 + (drone.z - leader.z)**2) ** 0.5
                    if dist_to_leader > 0.5:  # فاصله از لیدر
                        drone.move_towards(leader.x, leader.y, leader.z, step_size)
                        all_reached = False

        # نمایش مختصات جدید در هر گام
        print_drones(drones)

        time.sleep(step_time)

        if all_reached:
            break

# تابع برای گرفتن دستور از کاربر (فقط بعد از شروع حرکت)
def command_listener(drones):
    while True:
        command = input("دستور؟ (Enter برای ادامه / d برای حذف لیدر / q برای خروج): ")
        if command == 'd':
            print("لیدر حذف شد.")
            # حذف لیدر از لیست درون‌ها
            leader = next(d for d in drones if d.leader)
            leader.active = False  # حذف لیدر
            print(f"لیدر {leader.id} حذف شد.")
        elif command == 'q':
            print("خروج از برنامه.")
            exit()

# تابع اصلی
def main():
    # دریافت ورودی‌ها از کاربر
    num_drones = 4
    speed = float(input("سرعت حرکت درون‌ها (متر بر ثانیه): "))
    spacing = float(input("فاصله بین درون‌ها (متر): "))

    point2 = list(map(float, input("مختصات نقطه دوم (X Y Z): ").split()))
    point3 = list(map(float, input("مختصات نقطه سوم (X Y Z): ").split()))

    drones = [Drone(drone_id=i+1) for i in range(num_drones)]

    # استفاده از Z نقطه دوم به‌جای ارتفاع جداگانه
    for i, drone in enumerate(drones):
        drone.x = i * spacing
        drone.y = 0
        drone.z = point2[2]  # مقدار اولیه ارتفاع

    # انتخاب لیدر (در اینجا اولین درون به‌عنوان لیدر انتخاب می‌شود)
    drones[0].leader = True

    print("آرایش اولیه شکل گرفت:\n")
    print_drones(drones)

    # شروع حرکت درون‌ها
    print("\nحرکت به سمت نقطه دوم...\n")
    move_drones(drones, point2, speed)

    print("\nحرکت به سمت نقطه سوم...\n")
    move_drones(drones, point3, speed)

    # پس از شروع حرکت درون‌ها، درخواست دستور از کاربر
    command_thread = threading.Thread(target=command_listener, args=(drones,))
    command_thread.daemon = True
    command_thread.start()

if __name__ == "__main__":
    main()
