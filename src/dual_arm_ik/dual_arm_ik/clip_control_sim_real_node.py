import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading
import numpy as np

# 导入所有需要的自定义消息类型
from irobot_interfaces.msg import GripperControl, ClipCommand, GripperCommand
## ros2 topic pub --once /l_clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 1, value: 0.0}"
#ros2 topic pub --once /l_clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 1, value: 2.0}"

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

        # ================= 更改点 1：明确区分左右实机夹爪发布者 =================
        self.real_right_gripper_pub = self.create_publisher(ClipCommand, '/r_clip_cmd', 10)
        self.real_left_gripper_pub  = self.create_publisher(ClipCommand, '/l_clip_cmd', 10)
        
        self.sim_right_gripper_pub = self.create_publisher(GripperCommand, '/gripper/cmd_right', 10)
        self.sim_left_gripper_pub  = self.create_publisher(GripperCommand, '/gripper/cmd_left', 10)

        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10)
            
        self.get_logger().info('✅ Joint Aggregator (支持双臂实机+仿真同步) 已就绪.')

    def ik_callback(self, msg):
        self.ik_publisher.publish(msg)

    def hand_callback(self, msg):
        self.hand_publisher.publish(msg)

    def gripper_control_callback(self, msg):
        """
        接收全局 GripperControl 指令，并同步分发到实机和仿真环境。
        """
        # ================= 更改点 2：完善右爪与左爪的对称控制逻辑 =================
        if msg.gripper_select == GripperControl.GRIPPER_RIGHT:
            
            if msg.command == GripperControl.COMMAND_OPEN:
                self.get_logger().info('指令: [打开] 右夹爪')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = 0.7
                self.sim_right_gripper_pub.publish(sim_msg)
                
                # 实机节点 (位置模式: 1)
                real_msg = ClipCommand()
                real_msg.mode = 1
                real_msg.value = 2.0  # 张开的位置值，根据实机调整
                self.real_right_gripper_pub.publish(real_msg)

            elif msg.command == GripperControl.COMMAND_CLOSE:
                self.get_logger().info('指令: [关闭] 右夹爪 (启用电流递减)')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = -0.7
                self.sim_right_gripper_pub.publish(sim_msg)
                
                # 实机电流控制 (传入右侧发布者)
                ramp_thread = threading.Thread(target=self.ramp_down_gripper_current,
                                             args=(-0.8, -0.5, 1.0, 100, self.real_right_gripper_pub, "右"))
                ramp_thread.start()
        
        elif msg.gripper_select == GripperControl.GRIPPER_LEFT:
            
            if msg.command == GripperControl.COMMAND_OPEN:
                self.get_logger().info('指令: [打开] 左夹爪')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = -0.7 # 左侧张开在仿真里的方向可能相反，依你原代码保留
                self.sim_left_gripper_pub.publish(sim_msg)
                
                # 实机节点 (位置模式: 1)
                real_msg = ClipCommand()
                real_msg.mode = 1
                real_msg.value = 6.0  # 张开的位置值，根据实机调整
                self.real_left_gripper_pub.publish(real_msg)

            elif msg.command == GripperControl.COMMAND_CLOSE:
                self.get_logger().info('指令: [关闭] 左夹爪 (启用电流递减)')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = 0.7
                self.sim_left_gripper_pub.publish(sim_msg)
                
                # 实机电流控制 (传入左侧发布者)
                ramp_thread = threading.Thread(target=self.ramp_down_gripper_current,
                                             args=(-0.8, -0.5, 1.0, 100, self.real_left_gripper_pub, "左"))
                ramp_thread.start()

    # ================= 更改点 3：让电流递减函数通用化 =================
    def ramp_down_gripper_current(self, start_current, end_current, duration_secs, frequency, target_publisher, side_name):
        """
        在新线程中运行，用于向指定的发布者发布线性递减的电流指令。
        """
        num_steps = int(duration_secs * frequency)
        sleep_duration = 1.0 / frequency
        
        current_values = np.linspace(start_current, end_current, num_steps)
        
        self.get_logger().info(f"[{side_name}爪] 开始柔性抓取: {start_current}A -> {end_current}A，持续 {duration_secs}s")

        for current in current_values:
            real_msg = ClipCommand()
            real_msg.mode = 2  # 电流模式
            real_msg.value = float(current) 
            # 使用传入的特定发布者发布
            target_publisher.publish(real_msg)
            time.sleep(sleep_duration) 
        
        self.get_logger().info(f"[{side_name}爪] 电流递减完成，抓取稳固。")


def main(args=None):
    rclpy.init(args=args)
    node = JointAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()