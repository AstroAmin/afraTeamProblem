from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('ardu_afra_team'),
        'models',
        'iris_with_ardupilot',
        'model.sdf'
    )

    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    def spawn_drone(name, x, y, z):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', name, '-file', model_path, '-x', str(x), '-y', str(y), '-z', str(z)],
            output='screen'
        )

    # لیست موقعیت‌ها برای درون‌ها
    drone_positions = [
        ('drone1', 0, 0, 1),
        ('drone2', 2, 0, 1),
        ('drone3', 0, 2, 1),
        ('drone4', 2, 2, 1)
    ]

    # ایجاد نودهای spawn برای همه درون‌ها
    spawn_nodes = [spawn_drone(name, x, y, z) for name, x, y, z in drone_positions]

    # اضافه کردن نود منو به لانچ
    menu_node = Node(
        package='ardu_afra_team',
        executable='menu_node',
        output='screen',
        name='menu_node',
    )

    return LaunchDescription(spawn_nodes + [menu_node])
