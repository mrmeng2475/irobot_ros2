#!/usr/bin/python3
# 当前的求解器是以左臂为先
import os
import rclpy
from rclpy.node import Node
import pinocchio as pin
import numpy as np
from scipy.optimize import minimize
import time
from sensor_msgs.msg import JointState
from irobot_interfaces.msg import DualArmPoseTargets 


class DualArmIkNode(Node):
    def __init__(self):
        super().__init__('dual_arm_ik_node')

        self.is_ready = False
        self.joint_state_subscriber = None

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
            
        self.model_joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]
        # 用来判定当前状态是在初始状态还是在桌面状态
        try:
            self.right_arm_joint4_index = self.model_joint_names.index('right_arm_joint4')
            self.left_arm_joint4_index = self.model_joint_names.index('left_arm_joint4')
            self.get_logger().info(f"✅ 成功找到条件判断所需的关节索引: right_arm_joint4 at [{self.right_arm_joint4_index}], left_arm_joint4 at [{self.left_arm_joint4_index}]")
        except ValueError:
            self.get_logger().error("❌ 致命错误: 在URDF模型中找不到 'right_arm_joint4' 或 'left_arm_joint4'。节点无法启动。")
            # 如果找不到这两个关节，后续逻辑无法执行，直接抛出异常
            raise RuntimeError("Required joints for initial check not found in model.")

        self.left_joint_names = {name for name in self.model_joint_names if 'L' in name}
        self.right_joint_names = {name for name in self.model_joint_names if 'R' in name}
        
        self._left_arm_joints_received = False
        self._right_arm_joints_received = False
        self._received_joint_map = {}

        self.left_arm_frame_name = "left_arm_link8"
        self.right_arm_frame_name = "right_arm_link8"
        self.left_id = self.model.getFrameId(self.left_arm_frame_name)
        self.right_id = self.model.getFrameId(self.right_arm_frame_name)
        
        self.q_transition = np.array([1.2, -1.3,  0, 1.5708,  0, 0, 0,       
                                      -1.2, 1.3,  0, 1.5708,  0, 0, 0,])

        # 初始化准备点
        self.q_preferred = np.array([0, 0,  0, 1.5708,  0, 0.0, 0.0,       
                                      0,  0, 0, 1.5708, 0, 0.0, 0.0,])
        # 求解过程中的期望点
        self.q_Target = np.array([0, -0.5,  0, 1.5708,  0, 0.0, 0.0,       
                                    0,  0.5, 0, 1.5708, 0, 0.0, 0.0,])
        self.q = np.copy(self.q_preferred) 
        self.last_published_q = np.copy(self.q) 
        self.last_publish_time = self.get_clock().now()
        
        # --- IK 求解相关参数 ---
        self.max_joint_velocity = 0.8
        
        # <<< NEW: 为位置误差、姿态误差和姿态维持（正则化）分别设置权重
        self.position_error_weight = 1.0  # 位置误差权重
        self.orientation_error_weight = 0.1 # 姿态误差权重
        self.posture_regularization_weight = 1e-4 # 维持q_preferred的权重（原posture_weight）

        self.get_logger().info(f"💪 已设定期望关节姿态目标 (准备姿态)，权重为: {self.posture_regularization_weight}")
        self.get_logger().info(f"💪 位置/姿态误差权重: {self.position_error_weight} / {self.orientation_error_weight}")

        self.joint_angle_publisher = self.create_publisher(JointState, '/ik/joint_states', 10)
        self.target_subscription = self.create_subscription(
            DualArmPoseTargets,
            '/dual_arm/ik_targets',
            self.target_callback,
            10)
        
        self._move_to_ready_pose()
        self.get_logger().info("✅ 节点初始化完成，等待目标位姿输入...")

    # ... _initial_joint_state_callback 和 _move_to_ready_pose 函数保持不变 ...
    def _initial_joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._received_joint_map[name] = pos
        if not self._left_arm_joints_received and all(name in self._received_joint_map for name in self.left_joint_names):
            self._left_arm_joints_received = True
            self.get_logger().info("✅ 已成功读取到左臂关节信息!")
        if not self._right_arm_joints_received and all(name in self._received_joint_map for name in self.right_joint_names):
            self._right_arm_joints_received = True
            self.get_logger().info("✅ 已成功读取到右臂关节信息!")

    def _interpolate_and_publish(self, q_start, q_end, duration, frequency):
        """
        在指定时间内，从起始关节姿态平滑插值到结束姿态，并发布关节角度。
        """
        dt = 1.0 / frequency
        num_steps = int(duration * frequency)
        self.get_logger().info(f"开始插值运动，从 {np.round(q_start, 2)} 到 {np.round(q_end, 2)}，持续 {duration} 秒...")
        
        for i in range(num_steps + 1):
            if not rclpy.ok(): 
                break
            alpha = float(i) / num_steps
            q_interpolated = (1 - alpha) * q_start + alpha * q_end
            self.publish_joint_angles(q_interpolated)
            time.sleep(dt)
        
        if rclpy.ok():
            self.publish_joint_angles(q_end)
            self.get_logger().info("插值运动完成。")

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
            
        
        # 1. 获取需要检查的关节的当前角度
        joint4_R_angle = q_start[self.right_arm_joint4_index]
        joint4_L_angle = q_start[self.left_arm_joint4_index]
        self.get_logger().info(f"进行初始姿态检查... "
                             f"right_arm_joint4 当前角度: {joint4_R_angle:.4f}, "
                             f"left_arm_joint4 当前角度: {joint4_L_angle:.4f}")

        # 2. 执行if/else逻辑
        if joint4_R_angle < -1.0 and joint4_L_angle > 1.0:
            # --- 条件满足：直接移动到最终准备姿态 ---
            self.get_logger().info("✅ 条件满足，初始姿态良好。将跳过过渡点，直接移动到准备姿态。")
            # 这里的duration可以根据需要调整，例如设为原来两段的总时长或一个合适的固定值
            self._interpolate_and_publish(q_start=q_start, q_end=self.q_preferred, duration=10.0, frequency=50.0)
        
        else:
            # --- 条件不满足：执行原来的两段式移动 ---
            self.get_logger().info("❌ 条件不满足。将通过过渡点安全移动到准备姿态。")
            
            # 第1步: 从当前姿态移动到过渡姿态
            self.get_logger().info("--- 阶段1: 移动到过渡姿态 ---")
            self._interpolate_and_publish(q_start=q_start, q_end=self.q_transition, duration=8.0, frequency=50.0)
            
            if not rclpy.ok(): return

            # 第2步: 从过渡姿态移动到最终准备姿态
            self.get_logger().info("--- 阶段2: 移动到准备姿态 ---")
            self._interpolate_and_publish(q_start=self.q_transition, q_end=self.q_preferred, duration=5.0, frequency=50.0)

        
        if rclpy.ok():
            self.q = np.copy(self.q_preferred)
            self.last_published_q = np.copy(self.q_preferred)
            self.get_logger().info("✅ 已成功移动到最终准备姿态。")
            self.is_ready = True
            
    def target_callback(self, msg):
        if not self.is_ready:
            self.get_logger().warn("节点尚未准备就绪，忽略收到的目标。")
            return

        self.get_logger().info('========= 接收到新的双臂目标位姿 =========')
        
        # <<< MODIFIED: 从Pose消息中解析目标位置和姿态，并转换为Pinocchio SE3对象
        # 左臂
        left_pos = np.array([msg.left_target.position.x, msg.left_target.position.y, msg.left_target.position.z])
        left_quat = pin.Quaternion(msg.left_target.orientation.w, msg.left_target.orientation.x, msg.left_target.orientation.y, msg.left_target.orientation.z)
        target_pose_left = pin.SE3(left_quat, left_pos)
        
        # 右臂
        right_pos = np.array([msg.right_target.position.x, msg.right_target.position.y, msg.right_target.position.z])
        right_quat = pin.Quaternion(msg.right_target.orientation.w, msg.right_target.orientation.x, msg.right_target.orientation.y, msg.right_target.orientation.z)
        target_pose_right = pin.SE3(right_quat, right_pos)

        self.get_logger().info(f"左臂目标 (Pos): {np.round(left_pos, 3)}")
        self.get_logger().info(f"右臂目标 (Pos): {np.round(right_pos, 3)}")

        try:
            start_time = time.time()
            # <<< MODIFIED: 将完整的位姿目标传入优化器
            q_opt, success = self.ik_optimize(target_pose_left, target_pose_right, self.q)
            
            if success:
                self.get_logger().info(f"✅ 双臂逆运动学求解成功! (耗时: {time.time() - start_time:.4f} 秒)")
                
                # 速度限制部分保持不变
                current_time = self.get_clock().now()
                dt = (current_time - self.last_publish_time).nanoseconds / 1e9
                if dt < 0.001: return 

                required_velocity = (q_opt - self.last_published_q) / dt
                max_ratio = np.max(np.abs(required_velocity / self.max_joint_velocity))

                if max_ratio > 1.0:
                    q_to_publish = self.last_published_q + (q_opt - self.last_published_q) / max_ratio
                    self.get_logger().warn(f"🏃‍♂️ 速度超限! 将运动按 {1/max_ratio:.2f} 的比例缩放。")
                else:
                    q_to_publish = q_opt

                self.q = q_to_publish
                self.publish_joint_angles(q_to_publish)
                self.last_published_q = np.copy(q_to_publish)
                self.last_publish_time = current_time

                # <<< MODIFIED: 验证函数也需要传入完整位姿
                self.verify_solution(q_to_publish, target_pose_left, target_pose_right)
            else:
                self.get_logger().warn("❌ 双臂逆运动学求解失败!")
                
        except Exception as e:
            self.get_logger().error(f"❌ 逆运动学计算失败: {e}", exc_info=True)

    # <<< MODIFIED: 全面重写误差函数以计算6D位姿误差 >>>
    def error_function(self, q, target_pose_left, target_pose_right):
        # 1. 正向运动学
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 2. 计算左右臂的6D位姿误差
        # 计算从当前位姿到目标位姿的变换矩阵
        left_error_pose = self.data.oMf[self.left_id].inverse() * target_pose_left
        right_error_pose = self.data.oMf[self.right_id].inverse() * target_pose_right
        
        # 使用pin.log6将变换矩阵转换为6D误差向量(dx, dy, dz, d_roll, d_pitch, d_yaw)
        # .np 属性获取其numpy数组表示
        left_pose_error_vec = pin.log6(left_error_pose).np
        right_pose_error_vec = pin.log6(right_error_pose).np
        
        # 分别提取位置误差和姿态误差
        left_pos_err = left_pose_error_vec[:3]
        left_ori_err = left_pose_error_vec[3:]
        right_pos_err = right_pose_error_vec[:3]
        right_ori_err = right_pose_error_vec[3:]
        
        # 加权计算总的位姿误差（L2范数的平方）
        pose_error = (self.position_error_weight * (np.sum(left_pos_err**2) + np.sum(right_pos_err**2)) +
                      self.orientation_error_weight * (np.sum(left_ori_err**2) + np.sum(right_ori_err**2)))

        # 3. 计算姿态维持误差（正则化项）
        posture_error = np.sum((q - self.q_Target)**2)
        
        # 4. 计算总误差
        total_error = pose_error + self.posture_regularization_weight * posture_error
        
        return total_error

    # <<< MODIFIED: 修改ik_optimize的调用签名
    def ik_optimize(self, target_pose_left, target_pose_right, q_init):
        bounds = [(self.model.lowerPositionLimit[i], self.model.upperPositionLimit[i]) for i in range(self.model.nq)]
        
        result = minimize(
            fun=lambda q: self.error_function(q, target_pose_left, target_pose_right), # 传递位姿目标
            x0=q_init, method='SLSQP', bounds=bounds,
            options={'disp': False, 'maxiter': 100, 'ftol': 1e-6}) # 增加ftol容差
        return result.x, result.success

    def publish_joint_angles(self, q_opt):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.model_joint_names
        joint_state_msg.position = q_opt.tolist()
        self.joint_angle_publisher.publish(joint_state_msg)

    # <<< MODIFIED: 修改验证函数以处理6D位姿
    def verify_solution(self, q_opt, target_left, target_right):
        pin.forwardKinematics(self.model, self.data, q_opt)
        pin.updateFramePlacements(self.model, self.data)
        
        final_pose_left = self.data.oMf[self.left_id]
        final_pose_right = self.data.oMf[self.right_id]
        
        # 使用pin.log6来计算最终的6D误差
        error_vec_left = pin.log6(final_pose_left.inverse() * target_left).np
        error_vec_right = pin.log6(final_pose_right.inverse() * target_right).np
        
        pos_error_left = np.linalg.norm(error_vec_left[:3])
        ori_error_left = np.linalg.norm(error_vec_left[3:]) # 误差单位是弧度
        
        pos_error_right = np.linalg.norm(error_vec_right[:3])
        ori_error_right = np.linalg.norm(error_vec_right[3:])
        
        self.get_logger().info(f"左臂最终误差 | 位置: {pos_error_left:.6f} m, 姿态: {ori_error_left:.6f} rad")
        self.get_logger().info(f"右臂最终误差 | 位置: {pos_error_right:.6f} m, 姿态: {ori_error_right:.6f} rad")

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