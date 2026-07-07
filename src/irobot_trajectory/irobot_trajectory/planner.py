#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import time

# 导入双臂位姿和夹爪控制的消息接口
from irobot_interfaces.msg import DualArmPoseTargets, GripperControl

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.publish_rate = 200  # Hz, 发布频率
        self.trajectory_duration = 2.0  # 秒，大模型单步指令的运动耗时
        
        # ==========================================
        # 1. 初始化发布者 (发送给底层实际/仿真控制端)
        # ==========================================
        self.target_publisher = self.create_publisher(
            DualArmPoseTargets,
            '/dual_arm/ik_targets',
            10)
            
        self.gripper_publisher = self.create_publisher(
            GripperControl,
            '/gripper_control',
            10)

        # ==========================================
        # 2. 初始化订阅者 (接收来自上层大模型大脑的控制)
        # ==========================================
        self.llm_posture_sub = self.create_subscription(
            DualArmPoseTargets,
            '/llm/posture_cmd',
            self.llm_posture_callback,
            10)

        self.llm_gripper_sub = self.create_subscription(
            GripperControl,
            '/llm/gripper_cmd',
            self.llm_gripper_callback,
            10)

        # ==========================================
        # 3. 维护当前机器人的状态与轨迹队列
        # ==========================================
        # 记录机器人双臂当前的实际/末端位姿，作为下一段插值的起点
        self.current_right_pos = np.array([0.382, -0.33394, 1.2435])
        self.current_right_quat = np.array([-0.3429, -0.000, 0.000, 0.93937])
        self.current_left_pos = np.array([0.382, 0.33394, 1.2435])
        self.current_left_quat = np.array([0.3429, -0.000, 0.000, 0.93937])

        # 在线生成的点阵队列
        self.trajectory_queue = []

        self.get_logger().info("动态轨迹规划节点已启动，等待大模型控制中枢下发指令...")

        # 初始化夹爪
        self.send_gripper_command(GripperControl.GRIPPER_LEFT, GripperControl.COMMAND_OPEN)
        self.send_gripper_command(GripperControl.GRIPPER_RIGHT, GripperControl.COMMAND_OPEN)
        time.sleep(1.0)

        # 启动 200Hz 的高频定时器，负责平滑输出队列中的每一个点
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_one_step_callback)

    def send_gripper_command(self, gripper_select, command):
        msg = GripperControl()
        msg.gripper_select = gripper_select
        msg.command = command
        self.gripper_publisher.publish(msg)

    # ==========================================
    # 核心回调：接收上层大模型发送的双臂目标点
    # ==========================================
    def llm_posture_callback(self, msg):
        """
        当大模型决策端发布了新的目标位姿，这里会自动计算 2 秒钟的五次或线性/Slerp插值，
        并将这 400 个点有序追加进执行队列。
        """
        num_steps = int(self.trajectory_duration * self.publish_rate) # 2.0 * 200 = 400 步
        
        # 1. 决定本次插值的起点。
        # 如果队列里还有没发完的点，则从最后一个点的终点开始接续插值，保证动作连续
        if len(self.trajectory_queue) > 0:
            last_msg = self.trajectory_queue[-1]
            start_left_pos = np.array([last_msg.left_target.position.x, last_msg.left_target.position.y, last_msg.left_target.position.z])
            start_left_quat = np.array([last_msg.left_target.orientation.x, last_msg.left_target.orientation.y, last_msg.left_target.orientation.z, last_msg.left_target.orientation.w])
            start_right_pos = np.array([last_msg.right_target.position.x, last_msg.right_target.position.y, last_msg.right_target.position.z])
            start_right_quat = np.array([last_msg.right_target.orientation.x, last_msg.right_target.orientation.y, last_msg.right_target.orientation.z, last_msg.right_target.orientation.w])
        else:
            # 如果当前没有任何正在执行的任务，直接从当前实际记录点出发
            start_left_pos = self.current_left_pos
            start_left_quat = self.current_left_quat
            start_right_pos = self.current_right_pos
            start_right_quat = self.current_right_quat

        # 2. 提取大模型下发的终点位姿
        end_left_pos = np.array([msg.left_target.position.x, msg.left_target.position.y, msg.left_target.position.z])
        end_left_quat = np.array([msg.left_target.orientation.x, msg.left_target.orientation.y, msg.left_target.orientation.z, msg.left_target.orientation.w])
        end_right_pos = np.array([msg.right_target.position.x, msg.right_target.position.y, msg.right_target.position.z])
        end_right_quat = np.array([msg.right_target.orientation.x, msg.right_target.orientation.y, msg.right_target.orientation.z, msg.right_target.orientation.w])

        # 3. 进行位置线性插值与姿态球面线性插值 (Slerp)
        interp_times = np.linspace(0, 1, num_steps)
        
        left_pos_traj = np.linspace(start_left_pos, end_left_pos, num_steps)
        left_slerp = Slerp([0, 1], Rotation.from_quat([start_left_quat, end_left_quat]))
        left_quat_traj = left_slerp(interp_times).as_quat()

        right_pos_traj = np.linspace(start_right_pos, end_right_pos, num_steps)
        right_slerp = Slerp([0, 1], Rotation.from_quat([start_right_quat, end_right_quat]))
        right_quat_traj = right_slerp(interp_times).as_quat()

        # 4. 生成 400 个高频点，打包追加到指令队列中
        for i in range(num_steps):
            step_msg = DualArmPoseTargets()
            
            # 填充左手臂插值结果
            step_msg.left_target.position.x = left_pos_traj[i][0]
            step_msg.left_target.position.y = left_pos_traj[i][1]
            step_msg.left_target.position.z = left_pos_traj[i][2]
            step_msg.left_target.orientation.x = left_quat_traj[i][0]
            step_msg.left_target.orientation.y = left_quat_traj[i][1]
            step_msg.left_target.orientation.z = left_quat_traj[i][2]
            step_msg.left_target.orientation.w = left_quat_traj[i][3]
            
            # 填充右手臂插值结果
            step_msg.right_target.position.x = right_pos_traj[i][0]
            step_msg.right_target.position.y = right_pos_traj[i][1]
            step_msg.right_target.position.z = right_pos_traj[i][2]
            step_msg.right_target.orientation.x = right_quat_traj[i][0]
            step_msg.right_target.orientation.y = right_quat_traj[i][1]
            step_msg.right_target.orientation.z = right_quat_traj[i][2]
            step_msg.right_target.orientation.w = right_quat_traj[i][3]

            self.trajectory_queue.append(step_msg)
            
        self.get_logger().info(f"📈 收到大模型目标位姿{msg}. 已成功生成 2s 内的 {num_steps} 个插值过渡点并加入执行缓存槽。")

    # ==========================================
    # 回调：接收并立即转发夹爪控制
    # ==========================================
    def llm_gripper_callback(self, msg):
        """
        上层大模型决策端发送闭合/张开夹爪指令时，通过这里直接透明转发给底层驱动或仿真器
        """
        gripper_str = "左夹爪" if msg.gripper_select == GripperControl.GRIPPER_LEFT else "右夹爪"
        action_str = "打开" if msg.command == GripperControl.COMMAND_OPEN else "关闭"
        self.get_logger().info(f"⚡ 收到大模型夹爪动作信号 -> 转发执行: {gripper_str} {action_str}")
        self.gripper_publisher.publish(msg)

    # ==========================================
    # 定时器：以 200Hz 频率不断消费队列中的轨迹点
    # ==========================================
    def publish_one_step_callback(self):
        # 如果队列中没有任何待执行的路径点，则静止，不做任何发布
        if len(self.trajectory_queue) == 0:
            return

        # 从队列最前端（头部）推出一个插值点并执行发布
        msg = self.trajectory_queue.pop(0)
        self.target_publisher.publish(msg)

        # 实时将当前发布的点记录为机械臂最新的“当前实际位置”
        # 这样即使大模型在机械臂运动中途发来了新的指令，也能瞬间从当时的位置继续平滑过渡
        self.current_left_pos = np.array([msg.left_target.position.x, msg.left_target.position.y, msg.left_target.position.z])
        self.current_left_quat = np.array([msg.left_target.orientation.x, msg.left_target.orientation.y, msg.left_target.orientation.z, msg.left_target.orientation.w])
        self.current_right_pos = np.array([msg.right_target.position.x, msg.right_target.position.y, msg.right_target.position.z])
        self.current_right_quat = np.array([msg.right_target.orientation.x, msg.right_target.orientation.y, msg.right_target.orientation.z, msg.right_target.orientation.w])

def main(args=None):
    rclpy.init(args=args)
    planner_node = TrajectoryPlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()