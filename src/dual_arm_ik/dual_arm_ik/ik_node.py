#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
import pinocchio as pin
import numpy as np
from scipy.optimize import minimize
import time
from sensor_msgs.msg import JointState
from irobot_interfaces.msg import DualArmTargets
from visualization_msgs.msg import Marker, MarkerArray

class DualArmIkNode(Node):
    def __init__(self):
        super().__init__('dual_arm_ik_node')

        # --- 状态标志位，防止未就绪时处理回调 ---
        self.is_ready = False

        # --- 变量初始化 ---
        self.joint_state_subscriber = None # 用于临时订阅

        try:
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory('dual_arm_ik')
            urdf_path = os.path.join(package_path, 'urdf', 'irobot.urdf')
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.get_logger().info(f"✅ 模型加载成功! Path: {urdf_path}")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            raise e
            
        # 识别并存储左右臂的关节名
        self.model_joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]
        self.left_joint_names = {name for name in self.model_joint_names if 'left_arm' in name}
        self.right_joint_names = {name for name in self.model_joint_names if 'right_arm' in name}
        self.get_logger().info(f"左臂关节: {self.left_joint_names}")
        self.get_logger().info(f"右臂关节: {self.right_joint_names}")
        
        # 用于阻塞式等待的状态变量
        self._left_arm_joints_received = False
        self._right_arm_joints_received = False
        self._received_joint_map = {}

        # Pinocchio 模型相关配置
        self.left_arm_frame_name = "left_arm_link8"
        self.right_arm_frame_name = "right_arm_link8"
        self.left_id = self.model.getFrameId(self.left_arm_frame_name)
        self.right_id = self.model.getFrameId(self.right_arm_frame_name)
        
        # IK求解相关参数
        self.q_preferred = np.array([0, 0,  0, 1.5708,  0, 0.0, 0.0,       
                                      0,  0, 0, 1.5708, 0, 0.0, 0.0,])
        self.q = np.copy(self.q_preferred) 
        self.last_published_q = np.copy(self.q) 
        self.last_publish_time = self.get_clock().now()
        self.max_joint_velocity = 1.0
        self.posture_weight = 1e-3 

        # 碰撞区域定义
        # self.collision_zones = [
        #     { 'x': [-0.5, 0.12], 'y': [-0.2, 0.2], 'z': [-0.5, 0.07] },
        #     { 'x': [-0.5, 0.28], 'y': [-0.16, 0.16], 'z': [0.36, 1.2] }
        # ]

        self.get_logger().info(f"💪 已设定期望关节姿态目标 (准备姿态)，权重为: {self.posture_weight}")

        # --- ROS 2 通信 ---
        self.joint_angle_publisher = self.create_publisher(
            JointState,
            '/ik/joint_states',
            10)
        self.get_logger().info("📢 将在 '/ik/joint_states' 上发布关节角度")

        self.target_subscription = self.create_subscription(
            DualArmTargets,
            '/dual_arm/ik_targets',
            self.target_callback,
            10)
        self.get_logger().info("📥 订阅 '/dual_arm/ik_targets' 以接收双臂目标位置")

        # 为RViz可视化创建Marker发布者
        self.marker_publisher = self.create_publisher(MarkerArray, '/dual_arm/ik_target_markers', 10)
        self.get_logger().info("📢 将在 '/dual_arm/ik_target_markers' 上发布可视化标记")

        # 启动时移动到准备姿态
        self._move_to_ready_pose()

        self.get_logger().info("✅ 节点初始化完成，等待目标位置输入...")

    def _initial_joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._received_joint_map[name] = pos
        if not self._left_arm_joints_received and all(name in self._received_joint_map for name in self.left_joint_names):
            self._left_arm_joints_received = True
            self.get_logger().info("✅ 已成功读取到左臂关节信息!")
        if not self._right_arm_joints_received and all(name in self._received_joint_map for name in self.right_joint_names):
            self._right_arm_joints_received = True
            self.get_logger().info("✅ 已成功读取到右臂关节信息!")

    def _move_to_ready_pose(self):
        self.get_logger().info("正在等待双臂的初始关节状态...")
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self._initial_joint_state_callback, 10)

        while rclpy.ok() and not (self._left_arm_joints_received and self._right_arm_joints_received):
            self.get_logger().info("等待双臂关节状态信息...", throttle_duration_sec=2)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self._left_arm_joints_received and self._right_arm_joints_received:
            self.get_logger().info("🎉 双臂关节状态信息已全部接收！")
        else:
            self.get_logger().warn("节点关闭，未能接收到完整的关节状态。")
            return

        self.destroy_subscription(self.joint_state_subscriber)
        self.joint_state_subscriber = None

        q_start = np.zeros(self.model.nq)
        try:
            for i, name in enumerate(self.model_joint_names):
                q_start[i] = self._received_joint_map[name]
        except KeyError as e:
            self.get_logger().error(f"关节名称不匹配: 无法在接收到的状态中找到关节 '{e}'。跳过移动。")
            return
            
        q_target = self.q_preferred
        self.get_logger().info(f"起始姿态 (q_start): {np.round(q_start, 3)}")
        self.get_logger().info(f"目标姿态 (q_target): {np.round(q_target, 3)}")

        duration = 5.0
        frequency = 50.0
        dt = 1.0 / frequency
        num_steps = int(duration * frequency)

        self.get_logger().info(f"将在 {duration} 秒内，以 {frequency} Hz 的频率平滑移动到准备姿态。")
        for i in range(num_steps + 1):
            if not rclpy.ok():
                self.get_logger().info("节点关闭，中断移动。")
                break
            alpha = float(i) / num_steps
            q_interpolated = (1 - alpha) * q_start + alpha * q_target
            self.publish_joint_angles(q_interpolated)
            time.sleep(dt)

        if rclpy.ok():
            self.publish_joint_angles(q_target)
            self.q = np.copy(q_target)
            self.last_published_q = np.copy(q_target)
            self.get_logger().info("✅ 已成功移动到准备姿态。")
            # --- 初始化完成，将节点标记为就绪状态 ---
            self.is_ready = True
        
    def is_in_any_collision_zone(self, point):
        x, y, z = point[0], point[1], point[2]
        for zone in self.collision_zones:
            x_in_zone = zone['x'][0] <= x <= zone['x'][1]
            y_in_zone = zone['y'][0] <= y <= zone['y'][1]
            z_in_zone = zone['z'][0] <= z <= zone['z'][1]
            if x_in_zone and y_in_zone and z_in_zone:
                return True
        return False

    def _get_end_effector_positions(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos_left = self.data.oMf[self.left_id].translation
        pos_right = self.data.oMf[self.right_id].translation
        return pos_left, pos_right

    def _publish_target_markers(self, target_left, target_right):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # 左臂目标点 Marker (绿色)
        marker_left = Marker()
        marker_left.header.frame_id = "base_link"
        marker_left.header.stamp = now
        marker_left.ns = "ik_targets"
        marker_left.id = 0
        marker_left.type = Marker.SPHERE
        marker_left.action = Marker.ADD
        marker_left.pose.position.x = float(target_left[0])
        marker_left.pose.position.y = float(target_left[1])
        marker_left.pose.position.z = float(target_left[2])
        marker_left.pose.orientation.w = 1.0
        marker_left.scale.x, marker_left.scale.y, marker_left.scale.z = 0.1, 0.1, 0.1
        marker_left.color.a, marker_left.color.r, marker_left.color.g, marker_left.color.b = 0.8, 0.0, 1.0, 0.0
        
        # 右臂目标点 Marker (蓝色)
        marker_right = Marker()
        marker_right.header.frame_id = "base_link"
        marker_right.header.stamp = now
        marker_right.ns = "ik_targets"
        marker_right.id = 1
        marker_right.type = Marker.SPHERE
        marker_right.action = Marker.ADD
        marker_right.pose.position.x = float(target_right[0])
        marker_right.pose.position.y = float(target_right[1])
        marker_right.pose.position.z = float(target_right[2])
        marker_right.pose.orientation.w = 1.0
        marker_right.scale.x, marker_right.scale.y, marker_right.scale.z = 0.1, 0.1, 0.1
        marker_right.color.a, marker_right.color.r, marker_right.color.g, marker_right.color.b = 0.8, 0.0, 0.0, 1.0

        marker_array.markers.append(marker_left)
        marker_array.markers.append(marker_right)
        self.marker_publisher.publish(marker_array)

    def target_callback(self, msg):
        # --- 安全守卫：检查节点是否已完全就绪 ---
        if not self.is_ready:
            self.get_logger().warn("节点尚未准备就绪，忽略收到的目标。")
            return

        self.get_logger().info('========= 接收到新的双臂目标位置 =========')
        
        target_position_left = np.array([msg.left_target.x, msg.left_target.y, msg.left_target.z])
        target_position_right = np.array([msg.right_target.x, msg.right_target.y, msg.right_target.z])

        # 立即发布Marker以在RViz中可视化目标
        self._publish_target_markers(target_position_left, target_position_right)

        self.get_logger().info(f"左臂目标: {np.round(target_position_left, 3)}")
        self.get_logger().info(f"右臂目标: {np.round(target_position_right, 3)}")

        try:
            start_time = time.time()
            q_opt, success = self.ik_optimize(target_position_left, target_position_right, self.q)
            
            if success:
                self.get_logger().info(f"✅ 双臂逆运动学求解成功! (耗时: {time.time() - start_time:.4f} 秒)")

                current_time = self.get_clock().now()
                dt = (current_time - self.last_publish_time).nanoseconds / 1e9
                if dt < 0.001: return 

                # 速度限制
                required_velocity = (q_opt - self.last_published_q) / dt
                velocity_ratio = np.abs(required_velocity / self.max_joint_velocity)
                max_ratio = np.max(velocity_ratio)

                if max_ratio > 1.0:
                    q_to_publish = self.last_published_q + (q_opt - self.last_published_q) / max_ratio
                    self.get_logger().warn(f"🏃‍♂️ 速度超限! 将运动按 {1/max_ratio:.2f} 的比例缩放。")
                else:
                    q_to_publish = q_opt

                # 在发布前，对将要执行的姿态进行碰撞检测
                # final_pos_left, final_pos_right = self._get_end_effector_positions(q_to_publish)
                # left_in_zone = self.is_in_any_collision_zone(final_pos_left)
                # right_in_zone = self.is_in_any_collision_zone(final_pos_right)

                # if left_in_zone or right_in_zone:
                #     if left_in_zone: self.get_logger().warn(f"⚠️ IK解算出的左臂位置 {np.round(final_pos_left, 3)} 在碰撞区域内，已取消本次发布！")
                #     if right_in_zone: self.get_logger().warn(f"⚠️ IK解算出的右臂位置 {np.round(final_pos_right, 3)} 在碰撞区域内，已取消本次发布！")
                #     self.last_publish_time = self.get_clock().now() 
                #     return

                # 检测通过，更新状态并发布
                self.q = q_to_publish
                self.publish_joint_angles(q_to_publish)
                self.last_published_q = np.copy(q_to_publish)
                self.last_publish_time = current_time

                self.verify_solution(q_to_publish, target_position_left, target_position_right)
            else:
                self.get_logger().warn("❌ 双臂逆运动学求解失败!")
                
        except Exception as e:
            self.get_logger().error(f"❌ 逆运动学计算失败: {e}", exc_info=True)

    def error_function(self, q, target_pos_left, target_pos_right):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        current_pos_left = self.data.oMf[self.left_id].translation
        current_pos_right = self.data.oMf[self.right_id].translation
        
        error_left = current_pos_left - target_pos_left
        error_right = current_pos_right - target_pos_right
        position_error = np.sum(error_left**2) + np.sum(error_right**2)

        posture_error = np.sum((q - self.q_preferred)**2)
        total_error = position_error + self.posture_weight * posture_error
        
        return total_error

    def ik_optimize(self, target_pos_left, target_pos_right, q_init):
        bounds = [(self.model.lowerPositionLimit[i], self.model.upperPositionLimit[i]) for i in range(self.model.nq)]
        result = minimize(
            fun=lambda q: self.error_function(q, target_pos_left, target_pos_right),
            x0=q_init, method='SLSQP', bounds=bounds,
            options={'disp': False, 'maxiter': 100})
        return result.x, result.success

    def publish_joint_angles(self, q_opt):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.model_joint_names
        joint_state_msg.position = q_opt.tolist()
        self.joint_angle_publisher.publish(joint_state_msg)

    def verify_solution(self, q_opt, target_left, target_right):
        pin.forwardKinematics(self.model, self.data, q_opt)
        pin.updateFramePlacements(self.model, self.data)
        final_pos_left = self.data.oMf[self.left_id].translation
        final_pos_right = self.data.oMf[self.right_id].translation
        error_left = np.linalg.norm(final_pos_left - target_left)
        error_right = np.linalg.norm(final_pos_right - target_right)
        self.get_logger().info(f"左臂最终位置误差: {error_left:.6f}")
        self.get_logger().info(f"右臂最终位置误差: {error_right:.6f}")

def main(args=None):
    rclpy.init(args=args)
    ik_node = None
    try:
        ik_node = DualArmIkNode()
        rclpy.spin(ik_node)
    except Exception as e:
        if ik_node:
            ik_node.get_logger().fatal(f"节点启动或运行期间发生致命错误: {e}", exc_info=True)
        else:
            print(f"节点实例化失败: {e}")
    finally:
        if ik_node and rclpy.ok():
            ik_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()