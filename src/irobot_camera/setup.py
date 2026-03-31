from setuptools import find_packages, setup

package_name = 'irobot_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='753942512@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = irobot_camera.irobot_aruco:main',
            'camera_service_node = irobot_camera.irobot_aruco_service:main',
            'camera_depth_node = irobot_camera.irobot_aruco_depth:main',
            'head_track_node = irobot_camera.head_track:main',
            'head_track_service_node = irobot_camera.head_track_service:main',
            'object_position_node = irobot_camera.object_position:main',
            'cube_position_node = irobot_camera.cube_position:main',
        ],
    },
)
