from setuptools import setup

package_name = 'ur3e_esp32_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UR3e ESP32 pushbutton control via micro-ROS and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_input_node = ur3e_esp32_control.button_input_node:main',
            'joint_state_tracker = ur3e_esp32_control.joint_state_tracker:main',
        ],
    },
)