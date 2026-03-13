#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from irobot_interfaces.msg import DualArmPoseTargets
from geometry_msgs.msg import Pose

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')

        self.publish_rate = 200  # Hz, 发布频率
        self.target_publisher = self.create_publisher(
            DualArmPoseTargets,
            '/dual_arm/ik_targets',
            10)
            
        self.get_logger().info("轨迹规划节点已启动。")

        self.trajectory = []
        self.precompute_multi_stage_trajectory() 
        self.get_logger().info(f"已预先计算好包含 {len(self.trajectory)} 个点的多段轨迹。")

        self.step_counter = 0
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_one_step_callback)
            
        self.get_logger().info(f"定时器已启动，将以 {self.publish_rate} Hz 的频率发布轨迹。")


    def precompute_multi_stage_trajectory(self):
        """计算一个包含多个路径点的连续轨迹。"""
        
        # 1. 定义初始位姿 (机器人运动的起点)
        # 这个位姿本身不包含在下面的waypoints列表中
        initial_right_pose = (np.array([0.375, -0.160, 1.080]), np.array([0.000, -0.000, 0.000, 1.000]))
        initial_left_pose = (np.array([0.375, 0.160, 1.080]), np.array([0.000, -0.000, 0.000, 1.000]))
        

        # 2. 定义一个路径点(waypoints)列表
        # 每个路径点是一个字典，包含左右臂位姿和“从上一点到此点”的运动时间
        waypoints = [
            {
                "right_pose": (np.array([0.375, -0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 4.0  
            },#移开

            {
                "right_pose": (np.array([0.375, -0.120, 1.150]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#至抓取点上方

            {
                "right_pose": (np.array([0.375, -0.120, 1.00]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#抓取点抓取

            {
                "right_pose": (np.array([0.375, -0.120, 1.00]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#等待抓取完成
            {
                "right_pose": (np.array([0.375, -0.050, 1.20]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#移至倒水点
            {
                "right_pose": (np.array([0.375, -0.050, 1.20]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#开始倒水

            {
                "right_pose": (np.array([0.375, -0.050, 1.20]), np.array([-0.531, -0.001, -0.001, 0.848])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#等待
            {
                "right_pose": (np.array([0.375, -0.050, 1.20]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.250, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 3.0  
            },#恢复
            {
                "right_pose": (np.array([0.375, -0.160, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "left_pose":  (np.array([0.375, 0.160, 1.080]), np.array([0.000, -0.000, 0.000, 1.000])),
                "duration": 5.0  
            },#回到初始点放置

            
        ]

        # 3. 循环计算每一段轨迹并拼接
        current_left_pose = initial_left_pose
        current_right_pose = initial_right_pose

        for waypoint in waypoints:
            # -- 获取当前段的参数 --
            next_left_pose = waypoint["left_pose"]
            next_right_pose = waypoint["right_pose"]
            duration = waypoint["duration"]
            num_steps = int(duration * self.publish_rate)

            # -- 提取位置和姿态 --
            start_left_pos, start_left_quat = current_left_pose
            end_left_pos, end_left_quat = next_left_pose
            start_right_pos, start_right_quat = current_right_pose
            end_right_pos, end_right_quat = next_right_pose

            # -- 进行插值计算 (和之前一样) --
            interp_times = np.linspace(0, 1, num_steps)
            
            # 左臂
            left_pos_traj = np.linspace(start_left_pos, end_left_pos, num_steps)
            left_slerp = Slerp([0, 1], Rotation.from_quat([start_left_quat, end_left_quat]))
            left_quat_traj = left_slerp(interp_times).as_quat()

            # 右臂
            right_pos_traj = np.linspace(start_right_pos, end_right_pos, num_steps)
            right_slerp = Slerp([0, 1], Rotation.from_quat([start_right_quat, end_right_quat]))
            right_quat_traj = right_slerp(interp_times).as_quat()

            # -- 将生成的轨迹段拼接到总轨迹中 --
            for i in range(num_steps):
                msg = DualArmPoseTargets()
                # 填充左臂
                msg.left_target.position.x = left_pos_traj[i][0]
                msg.left_target.position.y = left_pos_traj[i][1]
                msg.left_target.position.z = left_pos_traj[i][2]
                msg.left_target.orientation.x = left_quat_traj[i][0]
                msg.left_target.orientation.y = left_quat_traj[i][1]
                msg.left_target.orientation.z = left_quat_traj[i][2]
                msg.left_target.orientation.w = left_quat_traj[i][3]
                # 填充右臂
                msg.right_target.position.x = right_pos_traj[i][0]
                msg.right_target.position.y = right_pos_traj[i][1]
                msg.right_target.position.z = right_pos_traj[i][2]
                msg.right_target.orientation.x = right_quat_traj[i][0]
                msg.right_target.orientation.y = right_quat_traj[i][1]
                msg.right_target.orientation.z = right_quat_traj[i][2]
                msg.right_target.orientation.w = right_quat_traj[i][3]
                self.trajectory.append(msg)
            
            # -- 更新当前位姿，为下一段轨迹做准备 --
            current_left_pose = next_left_pose
            current_right_pose = next_right_pose

    def publish_one_step_callback(self):
        """这个回调函数无需任何修改，它只负责按顺序发布轨迹点。"""
        if self.step_counter >= len(self.trajectory):
            self.get_logger().info("全部轨迹点发布完成。")
            self.publish_timer.cancel()
            return

        msg = self.trajectory[self.step_counter]
        self.target_publisher.publish(msg)
        # print("msg : ", msg) # 如果需要可以取消注释来调试

        self.step_counter += 1

def main(args=None):
    rclpy.init(args=args)
    planner_node = TrajectoryPlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()