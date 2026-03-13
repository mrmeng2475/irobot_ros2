#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class GoToZeroNode(Node):
    def __init__(self):
        super().__init__('go_to_zero_node')
        
        # 标志位，确保初始化移动只执行一次
        self.move_initiated = False

        # 1. 创建发布者，用于发送关节指令
        #    假设您的机器人控制器订阅这个话题
        #    请确保这个话题名称 `/ik/joint_states` 是您机器人控制器期望的
        self.joint_command_publisher = self.create_publisher(JointState, '/ik/joint_states', 10)
        
        # 2. 创建订阅者，用于获取机器人当前的关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10) # 使用QoS=10来确保能收到最新的消息
            
        self.get_logger().info("✅ 归零节点已启动，正在等待当前的关节状态...")

    def joint_state_callback(self, msg):
        # 3. 收到第一个关节状态消息后，执行归零运动
        if self.move_initiated:
            return

        self.move_initiated = True
        self.get_logger().info("🎉 已接收到当前关节状态，即将开始归零运动...")

        # 存储起始状态
        start_joint_names = msg.name
        start_joint_positions = np.array(msg.position)
        
        # 确保我们不再处理后续的/joint_states消息
        self.destroy_subscription(self.joint_state_subscriber)

        # 4. 执行平滑移动
        self.execute_move(start_joint_names, start_joint_positions)
        
        # 5. 任务完成后，销毁节点
        self.get_logger().info("✅ 归零运动完成，节点将关闭。")
        self.destroy_node()

    def execute_move(self, joint_names, q_start):
        # 目标是全零位
        q_target = np.zeros_like(q_start)

        # 定义运动参数
        duration = 5.0  # 移动总时间（秒）
        frequency = 50.0 # 控制频率（赫兹）
        dt = 1.0 / frequency
        num_steps = int(duration * frequency)

        self.get_logger().info(f"将在 {duration} 秒内移动到零位...")

        for i in range(num_steps + 1):
            if not rclpy.ok(): # 如果节点被外部关闭，则停止
                self.get_logger().warn("节点关闭，运动中断。")
                break
            
            # 线性插值
            alpha = float(i) / num_steps
            q_interpolated = (1 - alpha) * q_start + alpha * q_target

            # 创建并发布JointState消息
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = joint_names
            cmd_msg.position = q_interpolated.tolist()
            
            self.joint_command_publisher.publish(cmd_msg)
            
            time.sleep(dt)
            
        # 为确保精确到达，在循环结束后再发布一次最终目标
        final_cmd_msg = JointState()
        final_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        final_cmd_msg.name = joint_names
        final_cmd_msg.position = q_target.tolist()
        self.joint_command_publisher.publish(final_cmd_msg)
        time.sleep(0.1) # 稍作等待，确保消息发出


def main(args=None):
    rclpy.init(args=args)
    try:
        go_to_zero_node = GoToZeroNode()
        # spin会一直运行，直到节点被销毁 (destroy_node)
        rclpy.spin(go_to_zero_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 在程序退出前确保rclpy被正确关闭
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()