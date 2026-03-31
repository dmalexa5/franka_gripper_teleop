from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'franka_gripper_teleop'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [f for f in glob("launch/*") if os.path.isfile(f)],
        ),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("config/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Alexander',
    maintainer_email='dmalexa5@ncsu.edu',
    description='Serial teleoperation node for controlling a Franka gripper in ROS 2.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_teleop_node = franka_gripper_teleop.gripper_teleop_node:main',
        ],
    },
)
