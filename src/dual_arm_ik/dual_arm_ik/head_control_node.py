#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# 导入我们自己的自定义消息
from irobot_interfaces.msg import HeadCommand

class HeadControlNode(Node):
    def __init__(self):
        super().__init__('head_control_node')
        
        self.joint_names = ['joint_neck1', 'joint_neck2']
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # --- ROS 2 通信 ---
        self.subscription = self.create_subscription(
            HeadCommand, # <-- 使用自定义接口
            '/head/cmd_joints',
            self.head_cmd_callback,
            10)
            
        self.publisher_ = self.create_publisher(JointState, '/head/joint_states', 10)
        self.publish_timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("✅ 头部控制节点已启动 (使用自定义接口)")

    def head_cmd_callback(self, msg):
        # 从消息的字段中获取数据
        self.joint_positions['joint_neck1'] = msg.neck_joint_1
        self.joint_positions['joint_neck2'] = msg.neck_joint_2
        self.get_logger().info(f"收到头部指令: neck1={msg.neck_joint_1:.2f}, neck2={msg.neck_joint_2:.2f}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.publisher_.publish(msg)

# ... main 函数不变 ...
def main(args=None):
    rclpy.init(args=args)
    node = HeadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()