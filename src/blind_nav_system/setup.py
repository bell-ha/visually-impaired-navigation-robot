import glob
from setuptools import find_packages, setup

package_name = 'blind_nav_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.xml') + glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jongha',
    maintainer_email='jongha8273@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_interface = blind_nav_system.voice_interface:main',
            'main_state_machine = blind_nav_system.main_state_machine:main',
            'navigation_client = blind_nav_system.navigation_client:main',
            'sensor_monitor = blind_nav_system.sensor_monitor:main',
            'elevator_node = blind_nav_system.elevator_button_press.elevator_node:main',
            'visual_servo_node = blind_nav_system.elevator_button_press.visual_servo_node:main',
        ],
    },
)
