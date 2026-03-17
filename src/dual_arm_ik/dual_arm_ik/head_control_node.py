#!/usr/bin/python3
# ros2 topic pub --once /head/cmd_joints irobot_interfaces/msg/HeadCommand "{head_joint1: 0.0, head_joint2: 0.5}"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# 导入我们自己的自定义消息
from irobot_interfaces.msg import HeadCommand

class HeadControlNode(Node):
    def __init__(self):
        super().__init__('head_control_node')
        
        self.joint_names = ['head_joint1', 'head_joint2']
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # --- 1. 接收上层算法的指令 ---
        self.subscription = self.create_subscription(
            HeadCommand,
            '/head/cmd_joints',
            self.head_cmd_callback,
            10)
            
        # --- 2. 桥接与分发：将指令转发给真机和仿真 ---
        # 只要真机(C++节点)和 MuJoCo 都在监听 '/head_cmd'，它们就会同时收到指令
        self.cmd_publisher_ = self.create_publisher(
            HeadCommand, 
            '/head_cmd', 
            10)

        # --- 3. 状态反馈发布 (用于 RViz 等可视化) ---
        self.state_publisher_ = self.create_publisher(JointState, '/head/joint_states', 10)
        self.publish_timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("✅ 头部桥接分发节点已启动，监听 /head/cmd_joints，分发至 /head_cmd")

    def head_cmd_callback(self, msg):
        # 1. 更新内部状态用于 RViz 的 JointState 发布
        self.joint_positions['head_joint1'] = msg.head_joint1
        self.joint_positions['head_joint2'] = msg.head_joint2
        
        # 2. 【核心功能】直接将消息转发到 /head_cmd 话题
        self.cmd_publisher_.publish(msg)
        
        self.get_logger().info(f"🔄 已分发指令至真机和仿真: head1={msg.head_joint1:.2f}, head2={msg.head_joint2:.2f}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.state_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()