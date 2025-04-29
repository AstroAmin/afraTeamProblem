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

    # تابع برای اسپاون کردن درون‌ها
    def spawn_drone(name, x, y, z):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', name,
                '-file', model_path,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen'
        )

    # تابع برای ایجاد نودهای کنترل
    def control_drone(name, node_type):
        return Node(
            package='ardu_afra_team',  
            executable=node_type,
            namespace=name,
            output='screen'
        )

    return LaunchDescription([
        # اسپاون کردن درون‌ها
        spawn_drone('drone1', 0, 0, 1),
        spawn_drone('drone2', 2, 0, 1),
        spawn_drone('drone3', 0, 2, 1),
        spawn_drone('drone4', 2, 2, 1),

        # نودهای مختلف برای هر درون
        control_drone('drone1', 'movment_control_node'),
        control_drone('drone2', 'shape_control_node'),
        control_drone('drone3', 'menu_node'),
        control_drone('drone4', 'control'),
    ])
