import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading
import numpy as np

# 导入所有需要的自定义消息类型
from irobot_interfaces.msg import GripperControl, ClipCommand, GripperCommand


class JointAggregator(Node):
    def __init__(self):
        super().__init__('joint_aggregator')
        
        # --- 原有部分保持不变 ---
        self.ik_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.hand_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.ik_subscriber = self.create_subscription(
            JointState,
            '/ik/joint_states',
            self.ik_callback,
            10)
            
        self.hand_subscriber = self.create_subscription(
            JointState,
            '/hand/joint_states',
            self.hand_callback,
            10)

        self.real_gripper_pub = self.create_publisher(ClipCommand, '/clip_cmd', 10)
        self.sim_right_gripper_pub = self.create_publisher(GripperCommand, '/gripper/cmd_right', 10)
        self.sim_left_gripper_pub = self.create_publisher(GripperCommand, '/gripper/cmd_left', 10)

        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10)
            
        self.get_logger().info('✅ Joint Aggregator with Corrected Gripper Control is ready.')

    def ik_callback(self, msg):
        self.ik_publisher.publish(msg)

    def hand_callback(self, msg):
        self.hand_publisher.publish(msg)

    def gripper_control_callback(self, msg):
        """
        这个回调函数是核心逻辑。
        它接收GripperControl指令，并将其转换为实机和仿真节点能理解的具体指令。
        """
        if msg.gripper_select == GripperControl.GRIPPER_RIGHT:
            
            if msg.command == GripperControl.COMMAND_OPEN:
                self.get_logger().info('指令: 打开右夹爪 -> 分发到实机和仿真...')
                
                # 给仿真节点发送指令 (保持不变)
                sim_msg = GripperCommand()
                sim_msg.position = 0.7
                self.sim_right_gripper_pub.publish(sim_msg)
                
                # 给实机节点发送指令 (保持不变)
                real_msg = ClipCommand()
                real_msg.mode = 1
                real_msg.value = 2.0
                self.real_gripper_pub.publish(real_msg)

            elif msg.command == GripperControl.COMMAND_CLOSE:
                self.get_logger().info('指令: 关闭右夹爪 -> 分发仿真指令并启动实机电流递减...')
                
                # 给仿真节点发送指令 (保持不变)
                sim_msg = GripperCommand()
                sim_msg.position = -0.7
                self.sim_right_gripper_pub.publish(sim_msg)
                
                # --- 新增：启动一个新线程来处理实机夹爪的电流递减 ---
                # 这样做可以避免阻塞ROS的主循环
                ramp_thread = threading.Thread(target=self.ramp_down_gripper_current,
                                             args=(-0.8, -0.5, 1.0, 100))
                ramp_thread.start()
        
        # --- 左夹爪控制逻辑 (保持不变) ---
        elif msg.gripper_select == GripperControl.GRIPPER_LEFT:
            self.get_logger().warn('当前实机控制器只支持一个夹爪ID，仅向下游仿真节点转发左夹爪指令。')
            if msg.command == GripperControl.COMMAND_OPEN:
                sim_msg = GripperCommand()
                sim_msg.position = -0.7
                self.sim_left_gripper_pub.publish(sim_msg)
            elif msg.command == GripperControl.COMMAND_CLOSE:
                sim_msg = GripperCommand()
                sim_msg.position = 0.7
                self.sim_left_gripper_pub.publish(sim_msg)

    def ramp_down_gripper_current(self, start_current, end_current, duration_secs, frequency):
        """
        一个在新线程中运行的函数，用于发布线性递减的电流指令。
        
        :param start_current: 起始电流 (例如 -0.8)
        :param end_current: 结束电流 (例如 -0.5)
        :param duration_secs: 持续时间 (秒)
        :param frequency: 发布频率 (Hz)
        """
        num_steps = int(duration_secs * frequency)
        sleep_duration = 1.0 / frequency
        
        # 使用numpy的linspace函数生成一个从起始值到结束值的等差数列
        current_values = np.linspace(start_current, end_current, num_steps)
        
        self.get_logger().info(f"开始电流递减: 从 {start_current}A 到 {end_current}A，持续 {duration_secs}s...")

        # 循环发布每一个计算出的电流值
        for current in current_values:
            real_msg = ClipCommand()
            real_msg.mode = 2  # 确保是电流模式
            real_msg.value = float(current) # 确保是标准的float类型
            self.real_gripper_pub.publish(real_msg)
            time.sleep(sleep_duration) # 等待一小段时间，以达到指定的频率
        
        self.get_logger().info("电流递减完成。")


def main(args=None):
    rclpy.init(args=args)
    node = JointAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()