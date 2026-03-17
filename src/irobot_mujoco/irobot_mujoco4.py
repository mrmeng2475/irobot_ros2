#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
import numpy as np
import time
import pinocchio as pin
from sensor_msgs.msg import JointState


# === 导入自定义视觉消息和坐标变换库 ===
from irobot_interfaces.msg import ObjectPose
from scipy.spatial.transform import Rotation as R
from irobot_interfaces.msg import HeadCommand

class DualArmIDControllerNode(Node):
    def __init__(self):
        super().__init__('dual_arm_inverse_dynamics_node')

        # ---  加载 MuJoCo 模型 ---
        try:
            xml_path = "./asset/scene_irobot.xml" 
            self.model_mj = mujoco.MjModel.from_xml_path(xml_path)
            self.data_mj = mujoco.MjData(self.model_mj)
            self.get_logger().info(f"✅ MuJoCo 模型加载成功! Path: {xml_path}")
        except Exception as e:
            self.get_logger().error(f"❌ MuJoCo 模型加载失败: {e}")
            raise e

        # ---  加载 Pinocchio 模型 ---
        try:
            urdf_filename = os.path.expanduser("/root/ros2_ws/irobot_ros2_humble/src/irobot_mujoco/asset/urdf/irobot.urdf")
            self.model_pin = pin.buildModelFromUrdf(urdf_filename)
            self.data_pin = self.model_pin.createData()
            self.get_logger().info(f"✅ Pinocchio 模型加载成功! Path: {urdf_filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Pinocchio 模型加载失败: {e}")
            raise e
        
        # === 新增：获取 head_link3 在 Pinocchio 中的 Frame ID ===
        try:
            self.head_link3_id = self.model_pin.getFrameId('head_link3')
            self.get_logger().info(f"✅ 找到 head_link3 的 Pinocchio Frame ID: {self.head_link3_id}")
        except Exception as e:
            self.get_logger().error(f"❌ 在 URDF 中找不到 head_link3: {e}")
        # ========================================================

        self.nq_pin = self.model_pin.nq
        self.nv_pin = self.model_pin.nv
        
        self.actuated_joint_names = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6', 'right_arm_joint7',
            'right_hand_joint1', 'right_hand_joint2',
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6', 'left_arm_joint7',
            'left_hand_joint1', 'left_hand_joint2',
            'head_joint1', 'head_joint2'
        ]
        
        self.actuated_joint_indices = [mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.actuated_joint_names]

        self.Kp = np.diag([500.0] * self.nv_pin)
        self.Kd = np.diag([50.0] * self.nv_pin) 

        self.q_des = np.zeros(self.nv_pin) 
        self.v_des = np.zeros(self.nv_pin)
        self.a_des = np.zeros(self.nv_pin)
        self.is_joint_state_received = False

        # --- 订阅话题 ---
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        # --- 监听分发节点发来的头部指令 ---
        self.head_cmd_sub = self.create_subscription(
            HeadCommand,
            '/head_cmd',  # 订阅从桥接节点转发出来的话题
            self.mujoco_head_callback,
            10
        )
        
        self.latest_object_name = None
        self.latest_obj_pos_cam = None
        self.latest_obj_quat_cam = None
        
        # 相机相对 head_link3 的坐标变换: 先绕z旋转-90,再绕x旋转-90
        r_z = R.from_euler('z', -90, degrees=True).as_matrix()
        r_x = R.from_euler('x', -90, degrees=True).as_matrix()
        self.R_cam2head = r_z @ r_x 
        
        self.obj_sub = self.create_subscription(
            ObjectPose,
            '/detected_objects',
            self.object_pose_callback,
            10
        )
    def mujoco_head_callback(self, msg: HeadCommand):
        try:
            # 找到头部关节在期望位置数组中的索引并更新
            idx_head1 = self.actuated_joint_names.index('head_joint1')
            idx_head2 = self.actuated_joint_names.index('head_joint2')
            
            self.q_des[idx_head1] = msg.head_joint1
            self.q_des[idx_head2] = msg.head_joint2
            
            # 确保动力学控制循环被激活
            if not self.is_joint_state_received:
                self.get_logger().info("✅ MuJoCo接收到头部指令，激活逆动力学控制！")
                self.is_joint_state_received = True
                
        except ValueError as e:
            self.get_logger().error(f"❌ MuJoCo更新头部关节失败: {e}")

    def joint_state_callback(self, msg: JointState):
        try:
            if not self.is_joint_state_received:
                self.get_logger().info("✅ 首次接收到 /joint_states 消息，开始逆动力学控制！")
                self.is_joint_state_received = True
            
            for name in self.actuated_joint_names:
                if name in msg.name:
                    idx_in_msg = msg.name.index(name)
                    idx_in_model = self.actuated_joint_indices[self.actuated_joint_names.index(name)]
                    self.q_des[idx_in_model] = msg.position[idx_in_msg]
            
            self.v_des.fill(0.0)
            self.a_des.fill(0.0)
        except Exception as e:
            pass

    def object_pose_callback(self, msg: ObjectPose):
        self.latest_object_name = msg.object_name
        self.latest_obj_pos_cam = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.latest_obj_quat_cam = np.array([
            msg.pose.orientation.x, 
            msg.pose.orientation.y, 
            msg.pose.orientation.z, 
            msg.pose.orientation.w
        ])

    def run_simulation(self):
        with mujoco.viewer.launch_passive(self.model_mj, self.data_mj) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.8]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -20.0

            self.get_logger().info("🚀 仿真已启动。等待话题消息...")

            while viewer.is_running() and rclpy.ok():
                step_start_time = time.time()
                rclpy.spin_once(self, timeout_sec=0)

                # === 新增：获取当前所有关节角供 Pinocchio 计算正向运动学 ===
                q_pin_current = np.zeros(self.model_pin.nq)
                for name in self.actuated_joint_names:
                    mj_joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
                    if mj_joint_id != -1:
                        mj_qpos_adr = self.model_mj.jnt_qposadr[mj_joint_id]
                        pin_joint_id = self.model_pin.getJointId(name)
                        pin_idx_q = self.model_pin.joints[pin_joint_id].idx_q
                        q_pin_current[pin_idx_q] = self.data_mj.qpos[mj_qpos_adr]

                # 调用 Pinocchio 正向运动学，更新所有 Frame 的位姿
                pin.forwardKinematics(self.model_pin, self.data_pin, q_pin_current)
                pin.updateFramePlacements(self.model_pin, self.data_pin)
                # ==========================================================

                # ================= 计算并更新纯视觉 XYZ 坐标系 =================
                if self.latest_object_name is not None:
                    try:
                        # 使用 Pinocchio 获取 head_link3 的平移和旋转矩阵
                        oMf_head = self.data_pin.oMf[self.head_link3_id]
                        pos_head_world = oMf_head.translation
                        mat_head_world = oMf_head.rotation

                        # 计算世界系位姿
                        mat_cam_world = mat_head_world @ self.R_cam2head
                        pos_cam_world = pos_head_world 

                        mat_obj_cam = R.from_quat(self.latest_obj_quat_cam).as_matrix()
                        pos_obj_world = mat_cam_world @ self.latest_obj_pos_cam + pos_cam_world
                        mat_obj_world = mat_cam_world @ mat_obj_cam

                        quat_obj_world_scipy = R.from_matrix(mat_obj_world).as_quat()
                        quat_obj_mj = [quat_obj_world_scipy[3], quat_obj_world_scipy[0], quat_obj_world_scipy[1], quat_obj_world_scipy[2]]

                        mocap_body_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_BODY, 'target_frame')
                        if mocap_body_id != -1:
                            mocap_idx = self.model_mj.body_mocapid[mocap_body_id]
                            if mocap_idx != -1:
                                self.data_mj.mocap_pos[mocap_idx] = pos_obj_world
                                self.data_mj.mocap_quat[mocap_idx] = quat_obj_mj

                    except Exception as e:
                        self.get_logger().error(f"可视化坐标系计算失败: {e}", throttle_duration_sec=2)
                # =========================================================================

                # 以下为原有的逆动力学控制逻辑
                if self.is_joint_state_received:
                    v_pin = np.zeros(self.model_pin.nv)
                    
                    for name in self.actuated_joint_names:
                        mj_joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
                        mj_dof_adr = self.model_mj.jnt_dofadr[mj_joint_id]
                        
                        pin_joint_id = self.model_pin.getJointId(name)
                        pin_idx_v = self.model_pin.joints[pin_joint_id].idx_v
                        v_pin[pin_idx_v] = self.data_mj.qvel[mj_dof_adr]

                    # q_pin_current 已经在上面更新过了，直接用于动力学计算
                    tau_ff_pin = pin.rnea(self.model_pin, self.data_pin, q_pin_current, v_pin, self.a_des)

                    for i, name in enumerate(self.actuated_joint_names):
                        mj_joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
                        mj_dof_adr = self.model_mj.jnt_dofadr[mj_joint_id]
                        
                        pin_joint_id = self.model_pin.getJointId(name)
                        pin_idx_v = self.model_pin.joints[pin_joint_id].idx_v
                        
                        q_curr_single = self.data_mj.qpos[self.model_mj.jnt_qposadr[mj_joint_id]]
                        v_curr_single = self.data_mj.qvel[mj_dof_adr]
                        
                        q_err = self.q_des[i] - q_curr_single
                        v_err = self.v_des[i] - v_curr_single
                        
                        tau_fb_single = self.Kp[i, i] * q_err + self.Kd[i, i] * v_err
                        tau_total_single = tau_ff_pin[pin_idx_v] + tau_fb_single
                        
                        mj_actuator_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                        self.data_mj.ctrl[mj_actuator_id] = tau_total_single

                    mujoco.mj_step(self.model_mj, self.data_mj)

                viewer.sync()

                time_until_next_step = self.model_mj.opt.timestep - (time.time() - step_start_time)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DualArmIDControllerNode()
        node.run_simulation()
    except (ValueError, Exception) as e:
        print(f"节点启动或运行时发生错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()