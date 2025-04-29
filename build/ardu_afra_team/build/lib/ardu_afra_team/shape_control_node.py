import math
import time
from enum import Enum
from typing import List, Tuple

class FormationShape(Enum):
    LINE = 1
    TRIANGLE = 2
    QUADRILATERAL = 3
    IRREGULAR_QUADRILATERAL = 4  # حالت چهارم

class FormationGenerator:
    def __init__(self, shape: int, height: float, size: float, speed: float, sides: List[float] = None):
        self.shape = shape
        self.initial_height = height
        self.size = size
        self.speed = speed
        self.sides = sides
        self.positions = []
        self.is_emergency = False  # وضعیت اضطراری
        self.validate_inputs()

    def validate_inputs(self):
        if self.shape not in FormationShape._value2member_map_:
            raise ValueError("Invalid shape input! Please enter 1, 2, 3, or 4.")
        if self.size < 1.0:
            raise ValueError("Size must not be less than 1 meter.")
        if self.initial_height <= 0:
            raise ValueError("Height must be greater than zero.")
        if self.speed <= 0:
            raise ValueError("Speed must be greater than zero.")
        if self.shape == FormationShape.IRREGULAR_QUADRILATERAL and self.sides is None:
            raise ValueError("For irregular quadrilateral, sides of the shape must be provided.")

    # اضافه کردن بررسی وضعیت اضطراری
    def check_emergency(self):
        if self.speed < 0.5:  # فرض کنیم سرعت کم می‌شود که به معنای مشکل در پرواز است
            self.is_emergency = True
            print("⚠️ Emergency detected: Low speed. Returning to base...")
            return True
        return False

    def return_to_base(self):
        # فرض کنید نقطه (0, 0, height) به عنوان خانه است
        print("Returning to base...")
        self.positions = [(0, 0, self.initial_height) for _ in self.positions]

    def emergency_stop(self):
        # متوقف کردن فوری پهپادها
        print("Emergency stop: All drones are stopping.")
        self.speed = 0

    def generate_positions(self) -> List[Tuple[float, float, float]]:
        method = getattr(self, f"_generate_{FormationShape(self.shape).name.lower()}", None)
        if not method:
            raise NotImplementedError(f"Formation '{self.shape}' is not implemented.")
        self.positions = method()
        self._validate_spacing(self.positions)
        return self.positions

    def _generate_line(self):
        return [(i * self.size, 0, self.initial_height) for i in range(4)]

    def _generate_quadrilateral(self):
        return [
            (0, 0, self.initial_height),
            (self.size, 0, self.initial_height),
            (self.size, self.size, self.initial_height),
            (0, self.size, self.initial_height)
        ]

    def _generate_triangle(self):
        a = (0, 0, self.initial_height)
        b = (self.size, 0, self.initial_height)
        c = (self.size / 2, math.sqrt(3) / 2 * self.size, self.initial_height)
        centroid = (
            (a[0] + b[0] + c[0]) / 3,
            (a[1] + b[1] + c[1]) / 3,
            self.initial_height
        )
        return [a, b, c, centroid]

    def _generate_irregular_quadrilateral(self):
        # محاسبه موقعیت‌های چهارضلعی غیرمنظم
        a, b, c, d = self.sides
        # برای سادگی فرض می‌کنیم که چهار ضلعی با چهار ضلع غیر منتظم در مختصات دو بعدی مشخص شده است
        # اینجا می‌توانیم از فرمول‌هایی برای محاسبه مختصات نقاط استفاده کنیم، اما برای سادگی اینجا فقط یک شکل ساده قرار می‌دهیم.
        # مثلاً به طور تقریبی از اندازه‌های داده شده برای چهار ضلع استفاده می‌کنیم.
        return [
            (0, 0, self.initial_height),  # نقطه اول
            (a, 0, self.initial_height),  # نقطه دوم
            (a + b * math.cos(math.pi/4), b * math.sin(math.pi/4), self.initial_height),  # نقطه سوم
            (c, d, self.initial_height)   # نقطه چهارم
        ]

    def _validate_spacing(self, positions):
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0]) ** 2 +
                    (positions[i][1] - positions[j][1]) ** 2 +
                    (positions[i][2] - positions[j][2]) ** 2
                )
                if dist < 1.0:
                    raise ValueError(f"Warning: Drone {i+1} and {j+1} are too close: {dist:.2f} meters")

    def update_positions(self):
        if self.check_emergency():  # در صورت وضعیت اضطراری به خانه برگرد
            self.return_to_base()
        elif not self.is_emergency:
            for i in range(len(self.positions)):
                self.positions[i] = (
                    self.positions[i][0] + self.speed * 0.1,
                    self.positions[i][1] + self.speed * 0.1,
                    self.positions[i][2] + self.speed * 0.05
                )

    def display_positions(self):
        print("\nUpdated Drone Positions:")
        print("+-------+---------+---------+---------+")
        print("| ID    |   X(m)  |   Y(m)  |   Z(m)  |")
        print("+-------+---------+---------+---------+")
        for i, pos in enumerate(self.positions):
            print(f"|   {i+1}   |  {pos[0]:6.2f} |  {pos[1]:6.2f} |  {pos[2]:6.2f} |")
        print("+-------+---------+---------+---------+")

def main():
    try:
        print("\n🚀 Welcome to Space Drone Formation Console 🌌")
        print("Choose your formation shape:")
        print("🛸 1. Line Formation")
        print("🪐 2. Triangle Formation")
        print("🛸 3. Quadrilateral Formation")
        print("🌍 4. Irregular Quadrilateral Formation")  # گزینه چهارم

        while True:
            try:
                shape = int(input("Enter formation shape (1-4): "))
                if shape in [1, 2, 3, 4]:
                    break
                else:
                    print("Invalid input, please choose 1, 2, 3, or 4.")
            except ValueError:
                print("Invalid input, please enter a number.")

        if shape == 4:
            sides = []
            print("Enter the lengths of the four sides of the irregular quadrilateral:")
            for i in range(4):
                while True:
                    try:
                        side = float(input(f"Enter side {i+1} length (m): "))
                        if side > 0:
                            sides.append(side)
                            break
                        else:
                            print("Side length must be greater than zero.")
                    except ValueError:
                        print("Invalid input, please enter a number.")
        else:
            sides = None  # برای سایر حالت‌ها، این مقدار لازم نیست.

        while True:
            try:
                height = float(input("Enter flight height (m): "))
                if height > 0:
                    break
                else:
                    print("Height must be greater than zero.")
            except ValueError:
                print("Invalid input, please enter a number.")

        while True:
            try:
                size = float(input("Enter size/distance (m): "))
                if size >= 1.0:
                    break
                else:
                    print("Size must not be less than 1 meter.")
            except ValueError:
                print("Invalid input, please enter a number.")

        while True:
            try:
                speed = float(input("Enter flight speed (m/s): "))
                if speed > 0:
                    break
                else:
                    print("Speed must be greater than zero.")
            except ValueError:
                print("Invalid input, please enter a number.")

        formation = FormationGenerator(shape, height, size, speed, sides)
        formation.generate_positions()

        print("\nFormation starting...\n")
        for _ in range(10):
            formation.display_positions()
            formation.update_positions()  # به‌روزرسانی وضعیت
            time.sleep(1)

        print("\nMission Completed! Returning to base...\n")

    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
