from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ardu_afra_team'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # اصلاح شده: فقط فایل موجود
        ('share/' + package_name + '/launch', [
            'launch/spawn_drones.launch.py',  # حذف main_launch.py که وجود ندارد
        ]),

        (os.path.join('share', package_name, 'models', 'iris_with_ardupilot'),
         glob('models/iris_with_ardupilot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amin',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'dev': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'control = ardu_afra_team.control:main',
            'menu_node = ardu_afra_team.menu_node:main',
            'movment_control_node = ardu_afra_team.movment_control_node:main',
            'shape_control_node = ardu_afra_team.shape_control_node:main',
        ],
    },
)
