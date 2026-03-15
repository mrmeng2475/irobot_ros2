#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class GoToZeroNode(Node):
    def __init__(self):
        super().__init__('go_to_zero_node')
        
        self.move_initiated = False

        # --- 新增: 定义双臂关节名称和过渡点 ---
        # 1. 定义需要控制的双臂关节名称列表 (14个)
        self.arm_joint_names = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6', 'right_arm_joint7',
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6', 'left_arm_joint7'
        ]

        # 2. 定义第一个过渡点 (Ready Pose)
        self.q_transition_1 = np.array([
            0, 0,  0, 1.5708,  0, 0.0, 0.0,       # 右臂
            0, 0,  0, 1.5708, 0, 0.0, 0.0        # 左臂
        ])

        # 3. 定义第二个过渡点 (Folded Pose)
        self.q_transition_2 = np.array([
            -1.2, 1.3,  0, 1.5708,  0, 0, 0,     # 右臂
            1.2, -1.3,   0, 1.5708, 0, 0, 0      # 左臂
        ])
        
        # 创建发布者和订阅者
        self.joint_command_publisher = self.create_publisher(JointState, '/ik/joint_states', 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        self.get_logger().info("✅ 归零节点已启动，正在等待当前的关节状态...")

    def joint_state_callback(self, msg):
        if self.move_initiated:
            return

        self.move_initiated = True
        self.get_logger().info("🎉 已接收到当前关节状态，即将开始归零运动...")

        # 存储完整的起始状态 (包括所有关节)
        all_joint_names = msg.name
        q_start_full = np.array(msg.position)
        
        self.destroy_subscription(self.joint_state_subscriber)

        # --- 核心修改: 根据条件判断执行不同路径 ---
        try:
            # 建立关节名到索引的映射，方便查找
            name_to_idx = {name: i for i, name in enumerate(all_joint_names)}
            
            # 获取需要检查的关节的索引和当前角度
            idx_j4r = name_to_idx['right_arm_joint4']
            idx_j4l = name_to_idx['left_arm_joint4']
            angle_j4r = q_start_full[idx_j4r]
            angle_j4l = q_start_full[idx_j4l]

            self.get_logger().info(f"进行初始姿态检查... "
                                 f"right_arm_joint4: {angle_j4r:.4f}, left_arm_joint4: {angle_j4l:.4f}")

            # 执行判断
            if angle_j4r > -0.8 and angle_j4l < 0.8:
                self.get_logger().info("✅ 条件满足，将直接归零。")
                self.execute_direct_move(all_joint_names, q_start_full)
            else:
                self.get_logger().info("❌ 条件不满足，将通过两个过渡点分段归零。")
                self.execute_staged_move(all_joint_names, q_start_full)

        except KeyError as e:
            self.get_logger().error(f"❌ 关键关节 {e} 不在接收到的 /joint_states 消息中，无法执行运动。")
        
        # 任务完成后，销毁节点
        self.get_logger().info("✅ 归零运动完成，节点将关闭。")
        self.destroy_node()

    def execute_direct_move(self, joint_names, q_start):
        """路径1: 直接从当前位置移动到零位"""
        q_target = np.zeros_like(q_start)
        self._interpolate_and_publish(joint_names, q_start, q_target, duration=8.0)

    def execute_staged_move(self, all_joint_names, q_start_full):
        """路径2: 通过两个过渡点，分三段移动到零位"""
        name_to_idx = {name: i for i, name in enumerate(all_joint_names)}

        # --- 构建全尺寸的过渡目标向量 ---
        # 目标1: 将14个关节的目标姿态 self.q_transition_1 嵌入到一个全尺寸的零向量中
        q_target1_full = np.zeros_like(q_start_full)
        for i, name in enumerate(self.arm_joint_names):
            if name in name_to_idx:
                q_target1_full[name_to_idx[name]] = self.q_transition_1[i]

        # 目标2: 将 self.q_transition_2 嵌入
        q_target2_full = np.zeros_like(q_start_full)
        for i, name in enumerate(self.arm_joint_names):
            if name in name_to_idx:
                q_target2_full[name_to_idx[name]] = self.q_transition_2[i]
        
        # 最终目标: 全零位
        q_final_full = np.zeros_like(q_start_full)

        # --- 分段执行运动 ---
        self.get_logger().info("--- 阶段1: 移动到第一个过渡点 ---")
        self._interpolate_and_publish(all_joint_names, q_start_full, q_target1_full, duration=5.0)
        
        self.get_logger().info("--- 阶段2: 移动到第二个过渡点 ---")
        self._interpolate_and_publish(all_joint_names, q_target1_full, q_target2_full, duration=5.0)

        self.get_logger().info("--- 阶段3: 移动到最终零位 ---")
        self._interpolate_and_publish(all_joint_names, q_target2_full, q_final_full, duration=5.0)

    def _interpolate_and_publish(self, joint_names, q_start, q_target, duration):
        """辅助函数，用于执行两个姿态之间的平滑插值运动"""
        frequency = 50.0
        dt = 1.0 / frequency
        num_steps = int(duration * frequency)

        self.get_logger().info(f"将在 {duration} 秒内从 {np.round(q_start, 2)} 移动到 {np.round(q_target, 2)}...")

        for i in range(num_steps + 1):
            if not rclpy.ok():
                self.get_logger().warn("节点关闭，运动中断。")
                return
            
            alpha = float(i) / num_steps
            q_interpolated = (1 - alpha) * q_start + alpha * q_target

            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = joint_names
            cmd_msg.position = q_interpolated.tolist()
            
            self.joint_command_publisher.publish(cmd_msg)
            time.sleep(dt)
            
        final_cmd_msg = JointState()
        final_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        final_cmd_msg.name = joint_names
        final_cmd_msg.position = q_target.tolist()
        self.joint_command_publisher.publish(final_cmd_msg)
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    try:
        go_to_zero_node = GoToZeroNode()
        rclpy.spin(go_to_zero_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()