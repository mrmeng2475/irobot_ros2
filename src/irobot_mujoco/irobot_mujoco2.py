#!/usr/bin/python3

# 该套代码是对机器人进行力矩控制的代码

import os
import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
import numpy as np
import time
import pinocchio as pin
from sensor_msgs.msg import JointState

class DualArmIDControllerNode(Node):
    def __init__(self):
        """
        构造函数：进行所有初始化操作
        """
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
        
        # --- 获取瓶子 body 的信息 ---
        # try:
        #     self.bottle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'bottle')
        #     self.bottle_qpos_start_addr = self.model.jnt_qposadr[self.model.body_jntadr[self.bottle_id]]
        # except ValueError:
        #     self.get_logger().error("错误：在模型中找不到名为 'bottle' 的body。请检查XML文件。")
        #     raise

        # --- 3. 模型参数与验证 ---
        self.nq_pin = self.model_pin.nq
        self.nv_pin = self.model_pin.nv
        
        self.get_logger().info(f"Pinocchio (机器人)自由度: {self.nv_pin}")
        self.get_logger().info(f"MuJoCo (完整场景)自由度: {self.model_mj.nv}")
        self.get_logger().info(f"MuJoCo 执行器数量: {self.model_mj.nu}")
        
        
        self.actuated_joint_names = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6', 'right_arm_joint7',
            'right_hand_joint1', 'right_hand_joint2',
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6', 'left_arm_joint7',
            'left_hand_joint1', 'left_hand_joint2',
            'head_joint1', 'head_joint2'
                                     ]
        print(self.actuated_joint_names)
        # 获取受控关节在MuJoCo模型完整qpos/qvel向量中的索引
        self.actuated_joint_indices = [mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.actuated_joint_names]
        # print(self.actuated_joint_indices)

        # --- 4. 初始化控制器和状态变量 ---
        # PD控制器增益
        self.Kp = np.diag([500.0] * self.nv_pin)
        self.Kd = np.diag([50.0] * self.nv_pin) # 适当增加阻尼以获得更稳定的效果

        # 期望状态 (q_des, v_des, a_des) - 将由ROS话题更新
        # 我们需要为模型的全部自由度（nq）定义期望位置
        self.q_des = np.zeros(self.nv_pin) # 使用当前位置作为初始期望位置
        self.v_des = np.zeros(self.nv_pin)
        self.a_des = np.zeros(self.nv_pin)
        
        # 标志位，用于判断是否已收到第一个ROS消息
        self.is_joint_state_received = False

        # --- 5. 订阅 /joint_states 话题 ---
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("📥 订阅 '/joint_states' 以接收期望关节角度")

    def joint_state_callback(self, msg: JointState):
        """
        处理 /joint_states 消息的回调函数
        这个函数会更新我们的 *期望* 状态 (q_des)
        """
        try:
            # 标记已收到消息
            if not self.is_joint_state_received:
                self.get_logger().info("✅ 首次接收到 /joint_states 消息，开始逆动力学控制！")
                self.is_joint_state_received = True

            
            for name in self.actuated_joint_names:
                if name in msg.name:
                    idx_in_msg = msg.name.index(name)
                    idx_in_model = self.actuated_joint_indices[self.actuated_joint_names.index(name)]
                    self.q_des[idx_in_model] = msg.position[idx_in_msg]
                else:
                    self.get_logger().warn(f"期望的关节 {name} 不在收到的 /joint_states 消息中", throttle_duration_sec=5)

            
            self.v_des.fill(0.0)
            self.a_des.fill(0.0)
            
        except Exception as e:
            self.get_logger().error(f"处理关节状态失败: {e}")

    def run_simulation(self):
        """
        运行主仿真循环
        """
        with mujoco.viewer.launch_passive(self.model_mj, self.data_mj) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.8]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -20.0

            self.get_logger().info("🚀 仿真已启动。等待 '/joint_states' 话题消息...")

            while viewer.is_running() and rclpy.ok():
                step_start_time = time.time()

                # 1. 处理ROS消息：这会调用回调函数并更新 self.q_des
                rclpy.spin_once(self, timeout_sec=0)

                # 2. 只有在收到有效的期望位置后才进行控制和仿真
                if self.is_joint_state_received:
                    
                    # ---------------- A. 构建 Pinocchio 专属的 q 和 v ----------------
                    # 初始化符合 Pinocchio 内部顺序的全零数组
                    q_pin = np.zeros(self.model_pin.nq)
                    v_pin = np.zeros(self.model_pin.nv)
                    
                    # 遍历每一个受控关节，精准获取它在两个引擎中的独立索引
                    for name in self.actuated_joint_names:
                        # 获取 MuJoCo 中的索引并提取状态
                        mj_joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
                        mj_qpos_adr = self.model_mj.jnt_qposadr[mj_joint_id]
                        mj_dof_adr = self.model_mj.jnt_dofadr[mj_joint_id]
                        
                        # 获取 Pinocchio 中的索引并赋值
                        pin_joint_id = self.model_pin.getJointId(name)
                        pin_idx_q = self.model_pin.joints[pin_joint_id].idx_q
                        pin_idx_v = self.model_pin.joints[pin_joint_id].idx_v
                        
                        q_pin[pin_idx_q] = self.data_mj.qpos[mj_qpos_adr]
                        v_pin[pin_idx_v] = self.data_mj.qvel[mj_dof_adr]

                    # ---------------- B. 计算逆动力学 ----------------
                    # 此时传入的 q_pin 和 v_pin 完美符合 Pinocchio 的胃口
                    tau_ff_pin = pin.rnea(self.model_pin, self.data_pin, q_pin, v_pin, self.a_des)

                    # ---------------- C. 计算 PD 反馈并精准赋值给 MuJoCo ----------------
                    for i, name in enumerate(self.actuated_joint_names):
                        # 获取该关节的 MuJoCo 索引和 Pinocchio 速度索引
                        mj_joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
                        mj_dof_adr = self.model_mj.jnt_dofadr[mj_joint_id]
                        
                        pin_joint_id = self.model_pin.getJointId(name)
                        pin_idx_v = self.model_pin.joints[pin_joint_id].idx_v
                        
                        # 计算单个关节的 PD 误差
                        q_curr_single = self.data_mj.qpos[self.model_mj.jnt_qposadr[mj_joint_id]]
                        v_curr_single = self.data_mj.qvel[mj_dof_adr]
                        
                        # 假设此时 self.q_des 的顺序是对应 actuated_joint_names 的
                        q_err = self.q_des[i] - q_curr_single
                        v_err = self.v_des[i] - v_curr_single
                        
                        # 计算该关节的独立控制总力矩 (假设 Kp/Kd是对角阵，直接取对角线元素)
                        # 注意：如果你之前 Kp 是矩阵，这里简化为 Kp[i, i]
                        tau_fb_single = self.Kp[i, i] * q_err + self.Kd[i, i] * v_err
                        
                        # 前馈 + 反馈
                        tau_total_single = tau_ff_pin[pin_idx_v] + tau_fb_single
                        
                        # 找到该关节对应的 Motor 执行器 ID 并赋值
                        mj_actuator_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                        self.data_mj.ctrl[mj_actuator_id] = tau_total_single

                    # 3. 运行 MuJoCo 仿真一步
                    mujoco.mj_step(self.model_mj, self.data_mj)

                # 4. 同步可视化窗口
                viewer.sync()

                # 5. 控制仿真频率
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