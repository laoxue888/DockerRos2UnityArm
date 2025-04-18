from setuptools import find_packages, setup
import os

package_name = 'unity_control_example'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share_dir, 'launch'), ['launch/arm_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_control_node = unity_control_example.unity_control_node:main',
            'follow_joint_trajectory_monitor = unity_control_example.follow_joint_trajectory_monitor:main',
            'mover_service_server = unity_control_example.mover_service_server:main',
            'mover_panda_arm = unity_control_example.mover_panda_arm:main',
        ],
    },
)
