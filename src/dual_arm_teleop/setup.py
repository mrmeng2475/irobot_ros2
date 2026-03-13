
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # !! 添加了这一行来安装 launch 文件 !!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sti',
    maintainer_email='zhenyong.shan@robosense.cn',
    # 更新了描述
    description='A package for teleoperating a dual-arm robot.',
    # 更新了许可证
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = dual_arm_teleop.teleop_node:main',
            'teleop_clip_node = dual_arm_teleop.teleop_clip_node:main',
        ],
    },
)
