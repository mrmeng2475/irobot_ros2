#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# 导入我们自己的自定义消息
from irobot_interfaces.msg import GripperCommand

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        
        # --- 参数定义 ---
        self.max_clip_angle = 0.5 
        self.joint_names = ['right_hand_joint1', 'right_hand_joint2', 'left_hand_joint1', 'left_hand_joint2']
        self.joint_positions = {name: 0.0 for name in self.joint_names}

        # --- ROS 2 通信 ---
        # 使用自定义消息类型
        self.right_gripper_sub = self.create_subscription(
            GripperCommand, 
            '/gripper/cmd_right',
            self.right_gripper_callback,
            10)

        self.left_gripper_sub = self.create_subscription(
            GripperCommand, 
            '/gripper/cmd_left',
            self.left_gripper_callback,
            10)
            
        self.publisher_ = self.create_publisher(JointState, '/gripper/joint_states', 10)
        self.publish_timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("✅ 夹爪控制节点已启动 (使用自定义接口)")

    def right_gripper_callback(self, msg):
        # 从消息的 'position' 字段获取数据
        position = min(max(msg.position, -0.6), 1.0) 
        target_angle = position * self.max_clip_angle
        self.joint_positions['right_hand_joint1'] = target_angle
        self.joint_positions['right_hand_joint2'] = -target_angle
        self.get_logger().info(f"收到右夹爪指令: {position:.2f}, 设定角度: +/-{target_angle:.2f}")

    def left_gripper_callback(self, msg):
        position = min(max(msg.position, -1.0), 0.6)
        target_angle = position * self.max_clip_angle
        self.joint_positions['left_hand_joint1'] = target_angle
        self.joint_positions['left_hand_joint2'] = -target_angle
        self.get_logger().info(f"收到左夹爪指令: {position:.2f}, 设定角度: +/-{target_angle:.2f}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[name] for name in self.joint_names]
        self.publisher_.publish(msg)

# ... main 函数不变 ...
def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()