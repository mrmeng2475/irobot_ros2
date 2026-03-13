import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_share = get_package_share_directory('irobot_description')

    # 请确保这里的文件名是你自己正确的URDF文件名
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'irobot.urdf')

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 声明一个启动参数 'use_gui', 默认为 'true'
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    # 定义 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 定义 joint_state_publisher_gui 节点, 仅在 use_gui 为 true 时启动
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    # 定义 rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')]
    )

    # 返回要启动的节点列表
    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])