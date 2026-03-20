#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import time

# 导入消息接口
from irobot_interfaces.msg import DualArmPoseTargets, GripperControl
from geometry_msgs.msg import PoseStamped  

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.publish_rate = 200  # Hz, 发布频率
        
        # --- 发布者 ---
        self.target_publisher = self.create_publisher(DualArmPoseTargets, '/dual_arm/ik_targets', 10)
        self.gripper_publisher = self.create_publisher(GripperControl, '/gripper_control', 10)
            
        # --- 订阅者 (获取视觉目标位置) ---
        self.create_subscription(PoseStamped, '/target_pose/bottle1', self.bottle_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose/cup1', self.cup_callback, 10)

        # 存储动态获取的位置信息
        self.bottle_pos = None
        self.cup_pos = None
        self.trajectory_started = False  # 防止重复规划和启动

        self.trajectory = []
        self.action_points = {}
        self.step_counter = 0

        self.get_logger().info("🚀 轨迹规划节点已启动。正在等待 /target_pose/bottle1 和 cup1 的位置数据...")

        # 预先张开夹爪
        self.get_logger().info("正在初始化双臂夹爪状态（全部张开）...")
        self.send_gripper_command(GripperControl.GRIPPER_LEFT, GripperControl.COMMAND_OPEN)
        self.send_gripper_command(GripperControl.GRIPPER_RIGHT, GripperControl.COMMAND_OPEN)
        time.sleep(1.5)

    def send_gripper_command(self, gripper_select, command):
        msg = GripperControl()
        msg.gripper_select = gripper_select
        msg.command = command
        
        gripper_str = "左夹爪" if gripper_select == GripperControl.GRIPPER_LEFT else "右夹爪"
        action_str = "打开" if command == GripperControl.COMMAND_OPEN else "关闭"
        self.get_logger().info(f"----> 发送指令: {gripper_str} {action_str} <----")
        
        self.gripper_publisher.publish(msg)

    # ================= 动态坐标接收回调 =================
    def bottle_callback(self, msg: PoseStamped):
        if not self.trajectory_started:
            self.bottle_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self.check_ready()

    def cup_callback(self, msg: PoseStamped):
        if not self.trajectory_started:
            self.cup_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self.check_ready()

    def check_ready(self):
        """检查是否两个目标点都已获取到，如果获取到则开始规划轨迹"""
        if self.bottle_pos is not None and self.cup_pos is not None and not self.trajectory_started:
            self.trajectory_started = True
            self.get_logger().info("🎯 已获取到双臂目标动态位置！")
            
            # 开始预计算轨迹
            self.precompute_multi_stage_trajectory()
            self.get_logger().info(f"已预计算好包含 {len(self.trajectory)} 个点的轨迹。开始执行！")

            # 启动定时器，开始以 200Hz 发布动作
            self.publish_timer = self.create_timer(
                1.0 / self.publish_rate, 
                self.publish_one_step_callback)
    # ====================================================

    def precompute_multi_stage_trajectory(self):
        # ================= 1. 初始姿态配置 (应用你的新数据) =================
        initial_right_pose = (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937]))
        initial_left_pose = (np.array([0.382, 0.33394, 1.2435]), np.array([0.3429, -0.000, 0.000, 0.93937]))
        
        # 定义常用姿态四元数
        quat_default = np.array([0.000, -0.000, 0.000, 1.000])
        quat_pour    = np.array([-0.531, -0.001, -0.001, 0.848])

        # ================= 2. 动态目标处理 (偏移与钳制) =================
        B_pos = self.bottle_pos.copy()
        C_pos = self.cup_pos.copy()

        # 高度偏移 (Offset)
        B_pos[2] -= 0.08  # 右臂抓瓶子：Z轴减少 2cm
        C_pos[2] += 0.02  # 左臂抓杯子：Z轴增加 2cm

        # 桌面安全下限检查 (Clamping)
        safe_z_limit = 1.0331
        if B_pos[2] < safe_z_limit:
            self.get_logger().warn(f"⚠️ 警告: 瓶子抓取点Z触底！强行拉升至 {safe_z_limit}")
            B_pos[2] = safe_z_limit
        if C_pos[2] < safe_z_limit:
            self.get_logger().warn(f"⚠️ 警告: 杯子抓取点Z触底！强行拉升至 {safe_z_limit}")
            C_pos[2] = safe_z_limit

        self.get_logger().info(f"最终抓取点 -> 右臂(Bottle): {B_pos}, 左臂(Cup): {C_pos}")
        
        # 计算抓取点上方的预备点 (Z轴抬高 24cm，匹配你原来的 1.2731 - 1.0331)
        B_pre = B_pos + np.array([0.0, 0.0, 0.24])
        C_pre = C_pos + np.array([0.0, 0.0, 0.24])

        # ================= 3. 固定的操作点 =================
        pour_ready_right = np.array([0.387, -0.050, 1.3531])
        pour_ready_left  = np.array([0.357, 0.100, 1.1531])
        pour_action_right= np.array([0.387, -0.050, 1.3031])

        # 为了防止左臂在等待倒水时与右臂发生干涉，我们让左臂X轴先对齐 0.357
        C_wait_pour = np.array([0.357, C_pos[1], C_pos[2]])

        # ================= 4. 融合后的完整轨迹点位 (Waypoints) =================
        waypoints = [
            {
                "right_pose": (B_pre, quat_default),
                "left_pose":  (C_pre, quat_default),
                "duration": 3.0  
            }, # 1. 直接至抓取点上方 (从初始的倾斜四元数过渡到默认姿态)
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 5.0,
            }, # 2. 等待抓取 (下降)
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 3.0,
                "action": [
                    {"gripper": "right", "command": "close"},
                    {"gripper": "left", "command": "close"}
                ]
            }, # 3. 抓取点抓取
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 2.0  
            }, # 4. 等待抓取完成
            {
                "right_pose": (pour_ready_right, quat_default),
                "left_pose":  (C_wait_pour, quat_default),
                "duration": 5.0  
            }, # 5. 水瓶移至倒水点 (左臂原地跟随一下X轴)
            {
                "right_pose": (pour_ready_right, quat_default),
                "left_pose":  (pour_ready_left, quat_default),
                "duration": 5.0  
            }, # 6. 水杯移至接水点
            {
                "right_pose": (pour_action_right, quat_pour),
                "left_pose":  (pour_ready_left, quat_default),
                "duration": 3.0  
            }, # 7. 开始倒水
            {
                "right_pose": (pour_action_right, quat_pour),
                "left_pose":  (pour_ready_left, quat_default),
                "duration": 5.0  
            }, # 8. 等待倒水完成
            {
                "right_pose": (pour_action_right, quat_default),
                "left_pose":  (pour_ready_left, quat_default),
                "duration": 3.0  
            }, # 9. 恢复姿态 (右臂手腕回正)
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 5.0,
            }, # 10. 返回放置点 (动态原始抓取位置)
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 3.0,
            }, # 11. 等待放置
            {
                "right_pose": (B_pos, quat_default),
                "left_pose":  (C_pos, quat_default),
                "duration": 3.0,
                "action": [
                    {"gripper": "right", "command": "open"},
                    {"gripper": "left", "command": "open"}
                ]
            }, # 12. 打开夹爪
            {
                "right_pose": (B_pre, quat_default),
                "left_pose":  (C_pre, quat_default),
                "duration": 3.0,
            }, # 13. 上升 (回到安全预备高度)
            {
                "right_pose": initial_right_pose,
                "left_pose":  initial_left_pose,
                "duration": 3.0,
            }, # 14. 返回你的新初始点 (恢复四元数姿态)
        ]

        current_left_pose = initial_left_pose
        current_right_pose = initial_right_pose
        total_steps = 0

        for waypoint in waypoints:
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
                
                # 填充左臂目标
                msg.left_target.position.x = left_pos_traj[i][0]
                msg.left_target.position.y = left_pos_traj[i][1]
                msg.left_target.position.z = left_pos_traj[i][2]
                msg.left_target.orientation.x = left_quat_traj[i][0]
                msg.left_target.orientation.y = left_quat_traj[i][1]
                msg.left_target.orientation.z = left_quat_traj[i][2]
                msg.left_target.orientation.w = left_quat_traj[i][3]
                
                # 填充右臂目标
                msg.right_target.position.x = right_pos_traj[i][0]
                msg.right_target.position.y = right_pos_traj[i][1]
                msg.right_target.position.z = right_pos_traj[i][2]
                msg.right_target.orientation.x = right_quat_traj[i][0]
                msg.right_target.orientation.y = right_quat_traj[i][1]
                msg.right_target.orientation.z = right_quat_traj[i][2]
                msg.right_target.orientation.w = right_quat_traj[i][3]

                self.trajectory.append(msg)
            
            if "action" in waypoint:
                action_step_index = total_steps + num_steps - 1
                self.action_points[action_step_index] = waypoint["action"]

            current_left_pose = next_left_pose
            current_right_pose = next_right_pose
            total_steps += num_steps

    def publish_one_step_callback(self):
        if self.step_counter >= len(self.trajectory):
            self.get_logger().info("✅ 全部轨迹点发布完成。任务圆满结束。")
            self.publish_timer.cancel()
            return

        msg = self.trajectory[self.step_counter]
        self.target_publisher.publish(msg)

        if self.step_counter in self.action_points:
            actions_to_perform = self.action_points[self.step_counter]
            
            for action_item in actions_to_perform:
                gripper = action_item.get("gripper")
                command_str = action_item.get("command")
                
                if gripper == "left":
                    gripper_select = GripperControl.GRIPPER_LEFT
                elif gripper == "right":
                    gripper_select = GripperControl.GRIPPER_RIGHT
                else:
                    self.get_logger().warn(f"未知的夹爪: {gripper}")
                    continue

                if command_str == "open":
                    command = GripperControl.COMMAND_OPEN
                elif command_str == "close":
                    command = GripperControl.COMMAND_CLOSE
                else:
                    self.get_logger().warn(f"未知的指令: {command_str}")
                    continue

                self.send_gripper_command(gripper_select, command)

            self.publish_timer.cancel()
            time.sleep(1.5)
            self.publish_timer.reset()

        self.step_counter += 1

def main(args=None):
    rclpy.init(args=args)
    planner_node = TrajectoryPlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()