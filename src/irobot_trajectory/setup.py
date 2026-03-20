import warnings
from setuptools import setup

warnings.filterwarnings(
    'ignore', '.*Usage of dash-separated.*',
    module='setuptools.dist'
)

package_name = 'irobot_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrmeng',
    maintainer_email='mrmeng@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irobot_plan = irobot_trajectory.irobot_plan:main',
            'irobot_plan2 = irobot_trajectory.irobot_plan2:main',
            'irobot_plan3 = irobot_trajectory.irobot_plan3:main',
            'irobot_plan4 = irobot_trajectory.irobot_plan4:main',
            'cube_stack_mujoco = irobot_trajectory.cube_stack_mujoco:main',
        ],
    },
)
