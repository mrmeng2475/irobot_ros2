# ==========================================================
# 这是完整且正确的 setup.py 文件内容
# ==========================================================

from setuptools import setup
import os
from glob import glob

package_name = 'dual_arm_ik'

setup(
    name=package_name,
    version='0.0.1', # 版本号建议使用 0.0.1
    packages=[package_name], # 这里使用 package_name 变量，而不是 find_packages
    
    # data_files 必须包含 urdf 和 launch 文件的安装规则
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # !! 下面这两行是您缺失的关键配置 !!
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装 urdf 文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meng', # 您可以保留您的名字
    maintainer_email='753942512@qq.com', # 您可以保留您的邮箱
    description='irobot body angle control', # 建议添加描述
    license='Apache-2.0', # 建议声明一个许可证
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = dual_arm_ik.ik_node:main',
            'ik_posture_node = dual_arm_ik.ik_posture_node:main',
            'gripper_control_node = dual_arm_ik.gripper_control_node:main',
            'head_control_node = dual_arm_ik.head_control_node:main',
            'joint_state_aggregator_node = dual_arm_ik.joint_state_aggregator_node:main',
            'zero_node = dual_arm_ik.zero_node:main',
            'clip_control_sim_real_node = dual_arm_ik.clip_control_sim_real_node:main',
        ],
    },
)