#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

class JointStateAggregatorNode(Node):
    def __init__(self):
        super().__init__('joint_state_aggregator_node')
        
        # 定义机器人所有关节的名称
        self.all_joint_names = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6', 'right_arm_joint7',
            'right_hand_joint1', 'right_hand_joint2',
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6', 'left_arm_joint7',
            'left_hand_joint1', 'left_hand_joint2',
            'head_joint1', 'head_joint2'
        ]
        
        # ++++++++++ 这里是关键的修改 ++++++++++
        # 将发布的关节名修改为C++节点期望的格式 (移除了 "_R" 后缀)
        # 并且，我们将关节8命名为 'joint8'，以对应夹爪 'joint_clip1_R'
        self.right_arm_cmd_joint_names_map = {
            'right_arm_joint1': 'joint1',
            'right_arm_joint2': 'joint2',
            'right_arm_joint3': 'joint3',
            'right_arm_joint4': 'joint4',
            'right_arm_joint5': 'joint5',
            'right_arm_joint6': 'joint6',
            'right_arm_joint7': 'joint7',
            # 'right_hand_joint1': 'joint8' # 映射夹爪到ID为8的关节
        }
        # +++++++++++++++++++++++++++++++++++++
        
        self.joint_state_map = {name: 0.0 for name in self.all_joint_names}
        self.lock = threading.Lock()

        # 订阅所有控制器的输出
        self.arm_sub = self.create_subscription(
            JointState, '/ik/joint_states', self.update_map_callback, 10)
        self.gripper_sub = self.create_subscription(
            JointState, '/gripper/joint_states', self.update_map_callback, 10)
        self.head_sub = self.create_subscription(
            JointState, '/head/joint_states', self.update_map_callback, 10)
            
        # 用于发布完整仿真状态的发布者
        self.full_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
            
        # 用于发布右臂硬件控制指令的发布者
        self.right_arm_cmd_pub = self.create_publisher(
            JointState, '/joint_cmd', 10)
            
        # 定时器，用于周期性发布消息
        self.publish_timer = self.create_timer(0.02, self.publish_states)
        self.get_logger().info("✅ 关节状态聚合与指令分发节点已启动")

    def update_map_callback(self, msg):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_state_map:
                    self.joint_state_map[name] = msg.position[i]
        
    def publish_states(self):
        now = self.get_clock().now().to_msg()
        
        # --- 1. 发布完整的 /joint_states 用于仿真 ---
        full_state_msg = JointState()
        full_state_msg.header.stamp = now
        with self.lock:
            full_state_msg.name = self.all_joint_names
            full_state_msg.position = [self.joint_state_map[name] for name in self.all_joint_names]
        self.full_state_pub.publish(full_state_msg)

        # --- 2. 发布右臂的 /joint_cmd 用于硬件控制 ---
        right_arm_cmd_msg = JointState()
        right_arm_cmd_msg.header.stamp = now
        
        cmd_names = []
        cmd_positions = []
        
        with self.lock:
            for source_name, target_name in self.right_arm_cmd_joint_names_map.items():
                cmd_names.append(target_name)
                cmd_positions.append(self.joint_state_map.get(source_name, 0.0))
        
        right_arm_cmd_msg.name = cmd_names
        right_arm_cmd_msg.position = cmd_positions
        self.right_arm_cmd_pub.publish(right_arm_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()