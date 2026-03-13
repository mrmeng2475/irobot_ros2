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
        
        # --- 原有的发布者，用于发布机械臂IK目标 ---
        self.target_publisher = self.create_publisher(
            DualArmPoseTargets,
            '/dual_arm/ik_targets',
            10)
            
        # --- 新增：创建用于控制夹爪的发布者 ---
        self.gripper_publisher = self.create_publisher(
            GripperControl,
            '/gripper_control',
            10)
            
        self.get_logger().info("轨迹规划节点已启动。")

        self.trajectory = []
        # --- 新增：用一个字典来存储需要在特定轨迹点执行的夹爪动作 ---
        self.action_points = {}

        self.precompute_multi_stage_trajectory() 
        self.get_logger().info(f"已预计算好包含 {len(self.trajectory)} 个点的轨迹，并标记了 {len(self.action_points)} 个夹爪动作点。")

        self.step_counter = 0
        # --- 修改：在启动定时器前，先执行初始的夹爪动作 ---
        self.get_logger().info("正在初始化夹爪状态（张开）...")
        self.send_gripper_command(GripperControl.COMMAND_OPEN)
        time.sleep(1.5) # 等待1.5秒确保夹爪动作完成

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_one_step_callback)
            
        self.get_logger().info(f"定时器已启动，将以 {self.publish_rate} Hz 的频率发布轨迹。")

    def send_gripper_command(self, command):
        """一个用于发送夹爪指令的辅助函数"""
        msg = GripperControl()
        msg.gripper_select = GripperControl.GRIPPER_RIGHT # 这里我们默认控制右夹爪
        msg.command = command
        
        action_str = "打开" if command == GripperControl.COMMAND_OPEN else "关闭"
        self.get_logger().info(f"----> 发送夹爪指令: {action_str} <----")
        self.gripper_publisher.publish(msg)

    def precompute_multi_stage_trajectory(self):
        """
        计算多段轨迹，并在关键路径点“标记”上夹爪动作。
        """
        
        # 定义初始位姿
        initial_right_pose = (np.array([0.382, -0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000]))
        initial_left_pose = (np.array([0.382, 0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000]))
        
        # 定义路径点列表
        waypoints = [
            {
                "right_pose": (np.array([0.382, -0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 4.0  
            },#移开

            {
                "right_pose": (np.array([0.407, -0.250, 1.2531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#至上方抓取

            {
                "right_pose": (np.array([0.407, -0.080, 1.2531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#至抓取点上方

            {
                "right_pose": (np.array([0.407, -0.080, 1.0431]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0,
            },#抓取点抓取

            {
                "right_pose": (np.array([0.407, -0.080, 1.0431]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
                "action": "close"  
            },#抓取点抓取

            {
                "right_pose": (np.array([0.407, -0.080, 1.0431]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 2.0  
            },#等待抓取完成

            {
                "right_pose": (np.array([0.382, -0.050, 1.3031]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#移至倒水点

            {
                "right_pose": (np.array([0.382, -0.050, 1.3031]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#开始倒水

            {
                "right_pose": (np.array([0.382, -0.050, 1.3031]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#等待

            {
                "right_pose": (np.array([0.382, -0.050, 1.3031]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.250, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#恢复姿态

            {
                "right_pose": (np.array([0.382, -0.160, 1.0531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0,
            },#返回放置点

            {
                "right_pose": (np.array([0.382, -0.160, 1.0531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
            },#等待放置

            {
                "right_pose": (np.array([0.382, -0.160, 1.0531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
                "action": "open" 
            },#打开夹爪

            {
                "right_pose": (np.array([0.382, -0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.382, 0.160, 1.1831]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
            },#返回初始点
        ]

        # 循环计算每一段轨迹
        current_left_pose = initial_left_pose
        current_right_pose = initial_right_pose
        total_steps = 0

        for waypoint in waypoints:
            # ... (轨迹插值计算部分保持不变) ...
            next_left_pose = waypoint["left_pose"]
            next_right_pose = waypoint["right_pose"]
            duration = waypoint["duration"]
            num_steps = int(duration * self.publish_rate)
            start_left_pos, start_left_quat = current_left_pose
            end_left_pos, end_left_quat = next_left_pose
            start_right_pos, start_right_quat = current_right_pose
            end_right_pos, end_right_quat = next_right_pose
            interp_times = np.linspace(0, 1, num_steps)
            left_pos_traj = np.linspace(start_left_pos, end_left_pos, num_steps)
            left_slerp = Slerp([0, 1], Rotation.from_quat([start_left_quat, end_left_quat]))
            left_quat_traj = left_slerp(interp_times).as_quat()
            right_pos_traj = np.linspace(start_right_pos, end_right_pos, num_steps)
            right_slerp = Slerp([0, 1], Rotation.from_quat([start_right_quat, end_right_quat]))
            right_quat_traj = right_slerp(interp_times).as_quat()

            for i in range(num_steps):
                msg = DualArmPoseTargets()
                # ... (填充消息部分保持不变) ...
                msg.left_target.position.x = left_pos_traj[i][0]
                msg.left_target.position.y = left_pos_traj[i][1]
                msg.left_target.position.z = left_pos_traj[i][2]
                msg.left_target.orientation.x = left_quat_traj[i][0]
                msg.left_target.orientation.y = left_quat_traj[i][1]
                msg.left_target.orientation.z = left_quat_traj[i][2]
                msg.left_target.orientation.w = left_quat_traj[i][3]
                msg.right_target.position.x = right_pos_traj[i][0]
                msg.right_target.position.y = right_pos_traj[i][1]
                msg.right_target.position.z = right_pos_traj[i][2]
                msg.right_target.orientation.x = right_quat_traj[i][0]
                msg.right_target.orientation.y = right_quat_traj[i][1]
                msg.right_target.orientation.z = right_quat_traj[i][2]
                msg.right_target.orientation.w = right_quat_traj[i][3]
                self.trajectory.append(msg)
            
            # --- 新增：检查当前路径点是否有夹爪动作标记 ---
            if "action" in waypoint:
                # 我们将动作标记在轨迹段的最后一个点上
                action_step_index = total_steps + num_steps - 1
                if waypoint["action"] == "close":
                    self.action_points[action_step_index] = GripperControl.COMMAND_CLOSE
                elif waypoint["action"] == "open":
                    self.action_points[action_step_index] = GripperControl.COMMAND_OPEN

            # 更新当前位姿和总步数
            current_left_pose = next_left_pose
            current_right_pose = next_right_pose
            total_steps += num_steps


    def publish_one_step_callback(self):
        """
        按顺序发布轨迹点，并在到达“标记点”时触发夹爪动作。
        """
        if self.step_counter >= len(self.trajectory):
            self.get_logger().info("全部轨迹点发布完成。")
            self.publish_timer.cancel()
            return

        # 1. 发布当前的机械臂位姿
        msg = self.trajectory[self.step_counter]
        self.target_publisher.publish(msg)

        # 2. 新增：检查当前计数值是否是一个需要执行夹爪动作的“标记点”
        if self.step_counter in self.action_points:
            command = self.action_points[self.step_counter]
            self.send_gripper_command(command)
            # 在执行夹爪动作后，可以短暂暂停轨迹发布，给夹爪物理动作时间
            self.publish_timer.cancel()
            time.sleep(1.5) # 暂停1.5秒
            self.publish_timer.reset() # 重新启动定时器

        self.step_counter += 1

def main(args=None):
    rclpy.init(args=args)
    planner_node = TrajectoryPlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()