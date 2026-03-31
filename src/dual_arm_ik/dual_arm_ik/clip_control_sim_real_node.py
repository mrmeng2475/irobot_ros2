import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading

# 导入所有需要的自定义消息类型
from irobot_interfaces.msg import GripperControl, ClipCommand, GripperCommand

class JointAggregator(Node):
    def __init__(self):
        super().__init__('joint_aggregator')
        
        # --- 状态聚合发布者与订阅者 ---
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

        # --- 左右实机夹爪发布者 ---
        self.real_right_gripper_pub = self.create_publisher(ClipCommand, '/r_clip_cmd', 10)
        self.real_left_gripper_pub  = self.create_publisher(ClipCommand, '/l_clip_cmd', 10)
        
        # --- 左右仿真夹爪发布者 ---
        self.sim_right_gripper_pub = self.create_publisher(GripperCommand, '/gripper/cmd_right', 10)
        self.sim_left_gripper_pub  = self.create_publisher(GripperCommand, '/gripper/cmd_left', 10)

        # --- 全局控制指令订阅者 ---
        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10)
            
        self.get_logger().info('✅ Joint Aggregator (支持双臂实机三段式柔性抓取+仿真同步) 已就绪.')

    def ik_callback(self, msg):
        self.ik_publisher.publish(msg)

    def hand_callback(self, msg):
        self.hand_publisher.publish(msg)

    def gripper_control_callback(self, msg):
        """
        接收全局 GripperControl 指令，并同步分发到实机和仿真环境。
        """
        # ================= 右爪控制 =================
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
                real_msg.value = 2.0  
                self.real_right_gripper_pub.publish(real_msg)

            elif msg.command == GripperControl.COMMAND_CLOSE:
                self.get_logger().info('指令: [关闭] 右夹爪 (启用三段式柔性抓取)')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = -0.7
                self.sim_right_gripper_pub.publish(sim_msg)
                
                # 实机电流控制 (开启新线程执行三段式抓取)
                grasp_thread = threading.Thread(target=self.soft_grasp_open_loop,
                                             args=(self.real_right_gripper_pub, "右"))
                grasp_thread.start()
        
        # ================= 左爪控制 =================
        elif msg.gripper_select == GripperControl.GRIPPER_LEFT:
            
            if msg.command == GripperControl.COMMAND_OPEN:
                self.get_logger().info('指令: [打开] 左夹爪')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = -0.7 
                self.sim_left_gripper_pub.publish(sim_msg)
                
                # 实机节点 (位置模式: 1)
                real_msg = ClipCommand()
                real_msg.mode = 1
                real_msg.value = 6.0  
                self.real_left_gripper_pub.publish(real_msg)

            elif msg.command == GripperControl.COMMAND_CLOSE:
                self.get_logger().info('指令: [关闭] 左夹爪 (启用三段式柔性抓取)')
                
                # 仿真节点
                sim_msg = GripperCommand()
                sim_msg.position = 0.7
                self.sim_left_gripper_pub.publish(sim_msg)
                
                # 实机电流控制 (开启新线程执行三段式抓取)
                grasp_thread = threading.Thread(target=self.soft_grasp_open_loop,
                                             args=(self.real_left_gripper_pub, "左"))
                grasp_thread.start()

    # ================= 核心修改点：三段式开环柔性控制 =================
    def soft_grasp_open_loop(self, target_publisher, side_name):
        """
        三段式柔性抓取：启动脉冲 -> 极低速滑行 -> 夹紧保持
        在新线程中运行，避免阻塞 ROS 2 回调。
        """
        msg = ClipCommand()
        msg.mode = 2  # 电流模式

        try:
            # 阶段 1：启动脉冲 (Breakaway)
            # 目的：提供足够大的瞬间电流克服静摩擦力
            msg.value = -0.6
            target_publisher.publish(msg)
            time.sleep(0.1)  # 仅需 100ms 左右让夹爪“动起来”

            # 阶段 2：滑行合拢 (Coasting)
            # 目的：用仅能克服动摩擦的极小电流，让夹爪缓慢、匀速合拢，减小撞击
            # 注意：-0.15 需要根据你的实机硬件阻力进行微调！
            msg.value = -0.1 
            target_publisher.publish(msg)
            time.sleep(0.5)  # 预计夹爪触碰到物体所需的时间

            # 阶段 3：稳固保持 (Holding)
            # 目的：接触到物体后，增加电流以确保夹取力足够
            msg.value = -0.6 
            target_publisher.publish(msg)
            
            self.get_logger().info(f"[{side_name}爪] 柔性抓取完成，已进入保持状态。")

        except Exception as e:
            self.get_logger().error(f"[{side_name}爪] 抓取线程出现异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JointAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 确保 rclpy 正常关闭
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()