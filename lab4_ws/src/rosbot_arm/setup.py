from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'rosbot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_node = rosbot_arm.ik_node:main',
            'arm_control = rosbot_arm.arm_control:main',
            'arm_test = rosbot_arm.arm_test:main',
            'block_picker = rosbot_arm.block_picker:main',
            'block_stacker = rosbot_arm.block_stacker:main',
        ],
    },
)
