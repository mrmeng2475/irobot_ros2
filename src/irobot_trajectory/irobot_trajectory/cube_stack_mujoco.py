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
        
        self.target_publisher = self.create_publisher(
            DualArmPoseTargets,
            '/dual_arm/ik_targets',
            10)
            
        self.gripper_publisher = self.create_publisher(
            GripperControl,
            '/gripper_control',
            10)
            
        self.get_logger().info("轨迹规划节点已启动。")

        self.trajectory = []
        self.action_points = {}

        self.precompute_multi_stage_trajectory() 
        self.get_logger().info(f"已预计算好包含 {len(self.trajectory)} 个点的轨迹，并标记了 {len(self.action_points)} 个夹爪动作点。")

        self.step_counter = 0
        
        self.get_logger().info("正在初始化双臂夹爪状态（全部张开）...")
        self.send_gripper_command(GripperControl.GRIPPER_LEFT, GripperControl.COMMAND_OPEN)
        self.send_gripper_command(GripperControl.GRIPPER_RIGHT, GripperControl.COMMAND_OPEN)
        time.sleep(1.5)

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_one_step_callback)
            
        self.get_logger().info(f"定时器已启动，将以 {self.publish_rate} Hz 的频率发布轨迹。")

    def send_gripper_command(self, gripper_select, command):
        msg = GripperControl()
        msg.gripper_select = gripper_select
        msg.command = command
        
        gripper_str = "左夹爪" if gripper_select == GripperControl.GRIPPER_LEFT else "右夹爪"
        action_str = "打开" if command == GripperControl.COMMAND_OPEN else "关闭"
        self.get_logger().info(f"----> 发送指令: {gripper_str} {action_str} <----")
        
        self.gripper_publisher.publish(msg)

    def precompute_multi_stage_trajectory(self):
        initial_right_pose = (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937]))
        initial_left_pose = (np.array([0.382, 0.33394, 1.2435]), np.array([0.3429, -0.000, 0.000, 0.93937]))
        
        waypoints = [
            {
                "right_pose": (np.array([0.400, -0.150, 1.2731]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.400, 0.150, 1.2731]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#至抓取点上方
            {
                "right_pose": (np.array([0.400, -0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.400, 0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0,
            },#等待抓取
            {
                "right_pose": (np.array([0.400, -0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.400, 0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
                "action": [
                    {"gripper": "right", "command": "close"},
                    {"gripper": "left", "command": "close"}
                ]
            },#抓取点抓取
            {
                "right_pose": (np.array([0.400, -0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.400, 0.150, 1.0331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 2.0  
            },#等待抓取完成
            {
                "right_pose": (np.array([0.350, -0.0, 1.2531]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.350, 0.150, 1.2331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#右手方块移至中间方块上方，左手拿起方块等待
            {
                "right_pose": (np.array([0.350, -0.0, 1.0931]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.350, 0.150, 1.2331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#右手方块下降，左手等待
            {
                "right_pose": (np.array([0.350, -0.0, 1.0931]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.350, 0.150, 1.2331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
                "action": [
                    {"gripper": "right", "command": "open"},
                    # {"gripper": "left", "command": "close"}
                ]
            },# 右手方块放置到中间方块上，左手等待
            {
                "right_pose": (np.array([0.350, -0.0, 1.231]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.350, 0.150, 1.2331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#右手回到上方，左手等待
            {
                "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
                "left_pose":  (np.array([0.350, 0.0, 1.2331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 4.0   
            },#右手回到初始点，左手移动到中间方块上方
            {
                "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
                "left_pose":  (np.array([0.350, 0.0, 1.14331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
            },#左手开始放置
            {
                "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
                "left_pose":  (np.array([0.355, 0.0, 1.14331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
                "action": [
                    # {"gripper": "right", "command": "open"},
                    {"gripper": "left", "command": "open"}
                ]
            },#左手打开夹爪
            {
                "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
                "left_pose":  (np.array([0.350, 0.0, 1.25331]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0,
            },#左手回到上方
            {
                "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
                "left_pose":  (np.array([0.382, 0.33394, 1.2435]), np.array([0.3429, -0.000, 0.000, 0.93937]))
                "duration": 3.0,
            },#左手返回初始点
            # {
            #     "right_pose": (np.array([0.382, -0.33394, 1.2435]), np.array([-0.3429, -0.000, 0.000, 0.93937])),
            #     "left_pose":  (np.array([0.382, 0.33394, 1.2435]), np.array([0.3429, -0.000, 0.000, 0.93937])),
            #     "duration": 3.0,
            # },#返回初始点
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
            self.get_logger().info("全部轨迹点发布完成。")
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