# ~/work_space/irobot_ros2/src/dual_arm_ik/launch/controllers.launch.py

from  launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # # 启动双臂位置逆解节点
    # dual_arm_pos_node = Node(
    #     package='dual_arm_ik',
    #     executable='ik_node',
    #     name='ik_node',
    #     output='screen'
    # )

    # # 启动双臂位姿逆解节点
    # dual_arm_posture_node = Node(
    #     package='dual_arm_ik',
    #     executable='ik_posture_node',
    #     name='ik_posture_node',
    #     output='screen'
    # )
    # 启动夹爪仿真控制节点
    gripper_control_node = Node(
        package='dual_arm_ik',           # 节点所在的功能包
        executable='gripper_control_node', # 在setup.py中定义的可执行文件名称
        name='gripper_control_node',     # 节点在ROS网络中的名称
        output='screen'                  # 将节点的日志输出到终端屏幕
    )

    # 启动夹爪开合控制节点
    clip_control_sim_real_node = Node(
        package='dual_arm_ik',           # 节点所在的功能包
        executable='clip_control_sim_real_node', # 在setup.py中定义的可执行文件名称
        name='clip_control_sim_real_node',     # 节点在ROS网络中的名称
        output='screen'                  # 将节点的日志输出到终端屏幕
    )

    # # 启动头部控制节点
    head_control_node = Node(
        package='dual_arm_ik',
        executable='head_control_node',
        name='head_control_node',
        output='screen'
    )

    # 启动关节状态聚合节点
    joint_state_aggregator_node = Node(
        package='dual_arm_ik',
        executable='joint_state_aggregator_node',
        name='joint_state_aggregator_node',
        output='screen'
    )

    return LaunchDescription([
        # dual_arm_pos_node,
        # dual_arm_posture_node,
        gripper_control_node,
        clip_control_sim_real_node,
        head_control_node,
        joint_state_aggregator_node
    ])