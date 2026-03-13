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

class DualArmIDControllerNode(Node):
    def __init__(self):
        """
        构造函数：进行所有初始化操作
        """
        super().__init__('dual_arm_inverse_dynamics_node')

        # --- 1. 加载 MuJoCo 模型 ---
        try:
            # <--- 修改点 1: 加载包含所有物体的完整场景文件 ---
            # 假设你的场景文件名为 scene_irobot.xml
            xml_path = "./asset/scene_irobot.xml" 
            self.model_mj = mujoco.MjModel.from_xml_path(xml_path)
            self.data_mj = mujoco.MjData(self.model_mj)
            self.get_logger().info(f"✅ MuJoCo 场景加载成功! Path: {xml_path}")
        except Exception as e:
            self.get_logger().error(f"❌ MuJoCo 模型加载失败: {e}")
            raise e

        # --- 2. 加载 Pinocchio 模型 ---
        try:
            urdf_filename = os.path.expanduser("/home/mrmeng/work_space/irobot_ros2/src/irobot_mujoco/asset/urdf/irobot.urdf")
            self.model_pin = pin.buildModelFromUrdf(urdf_filename)
            self.data_pin = self.model_pin.createData()
            self.get_logger().info(f"✅ Pinocchio 模型加载成功! Path: {urdf_filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Pinocchio 模型加载失败: {e}")
            raise e

        # --- 3. 模型参数与验证 ---
        self.nq_pin = self.model_pin.nq
        self.nv_pin = self.model_pin.nv
        
        # <--- 修改点 2: 移除严格的自由度检查，改为信息打印 ---
        self.get_logger().info(f"Pinocchio (机器人)自由度: {self.nv_pin}")
        self.get_logger().info(f"MuJoCo (完整场景)自由度: {self.model_mj.nv}")
        self.get_logger().info(f"MuJoCo 执行器数量: {self.model_mj.nu}")

        if self.nv_pin != self.model_mj.nu:
            self.get_logger().warn(f"Pinocchio自由度 ({self.nv_pin}) 与MuJoCo执行器数量 ({self.model_mj.nu}) 不匹配. "
                                   f"假设执行器与Pinocchio模型关节一一对应。")

        self.actuated_joint_names = ['joint1_R', 'joint2_R', 'joint3_R', 'joint4_R', 'joint5_R', 'joint6_R', 'joint7_R',
                                     'joint_clip1_R','joint_clip2_R',
                                     'joint1_L', 'joint2_L', 'joint3_L', 'joint4_L', 'joint5_L', 'joint6_L', 'joint7_L',
                                     'joint_clip1_L','joint_clip2_L',
                                     'joint_neck1','joint_neck2']
        
        # <--- 修改点 3: 获取受控关节在qpos中的索引，用于提取状态 ---
        self.actuated_qpos_indices = []
        for name in self.actuated_joint_names:
            joint_id = mujoco.mj_name2id(self.model_mj, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id != -1:
                # 获取该关节在 qpos 向量中的起始地址
                self.actuated_qpos_indices.append(self.model_mj.jnt_qposadr[joint_id])
            else:
                 self.get_logger().error(f"在MuJoCo模型中找不到关节: {name}")
                 raise ValueError(f"Joint {name} not found in MuJoCo model")
        print(self.actuated_qpos_indices)
        # --- 4. 初始化控制器和状态变量 ---
        # PD控制器增益 (维度应与Pinocchio模型匹配)
        self.Kp = np.diag([100.0] * self.nv_pin)
        self.Kd = np.diag([20.0] * self.nv_pin)

        # 期望状态 (q_des, v_des) - 现在它们的维度与Pinocchio模型一致
        self.q_des_robot = np.zeros(self.nv_pin)
        self.v_des_robot = np.zeros(self.nv_pin)
        self.a_des_robot = np.zeros(self.nv_pin)
        
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
        这个函数会更新我们 *机器人部分* 的期望状态 (q_des_robot)
        """
        try:
            if not self.is_joint_state_received:
                self.get_logger().info("✅ 首次接收到 /joint_states 消息，开始逆动力学控制！")
                self.is_joint_state_received = True

            # 将收到的消息更新到机器人期望位置向量中
            for i, name in enumerate(self.actuated_joint_names):
                if name in msg.name:
                    idx_in_msg = msg.name.index(name)
                    self.q_des_robot[i] = msg.position[idx_in_msg]
                else:
                    self.get_logger().warn(f"期望的关节 {name} 不在收到的 /joint_states 消息中", throttle_duration_sec=5)
            
            # 保持期望速度和加速度为0
            self.v_des_robot.fill(0.0)
            self.a_des_robot.fill(0.0)
            
        except Exception as e:
            self.get_logger().error(f"处理关节状态失败: {e}")

    def run_simulation(self):
        """
        运行主仿真循环
        """
        with mujoco.viewer.launch_passive(self.model_mj, self.data_mj) as viewer:
            # ... (可视化设置不变)
            viewer.cam.lookat[:] = [0, 0, 0.8]
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -20.0

            self.get_logger().info("🚀 仿真已启动。等待 '/joint_states' 话题消息...")

            while viewer.is_running() and rclpy.ok():
                step_start_time = time.time()
                rclpy.spin_once(self, timeout_sec=0)

                if self.is_joint_state_received:
                    # <--- 修改点 4: 从完整的MuJoCo状态中提取机器人部分的状态 ---
                    q_full = self.data_mj.qpos
                    print('q_full', q_full)
                    v_full = self.data_mj.qvel
                    
                    # 使用我们之前计算的索引来提取机器人关节的状态
                    q_robot_current = q_full[self.actuated_qpos_indices]
                    v_robot_current = v_full[self.actuated_qpos_indices]

                    # --- 核心：逆动力学计算 (现在所有向量维度都与Pinocchio模型匹配) ---
                    # a. 前馈力矩 (Feedforward)
                    tau_ff = pin.rnea(self.model_pin, self.data_pin, self.q_des_robot, self.v_des_robot, self.a_des_robot)

                    # b. 反馈力矩 (Feedback - PD Controller)
                    q_error = self.q_des_robot - q_robot_current
                    v_error = self.v_des_robot - v_robot_current
                    tau_fb = self.Kp @ q_error + self.Kd @ v_error

                    # c. 组合控制律
                    tau_control = tau_ff + tau_fb

                    # d. 应用总力矩到 MuJoCo 执行器
                    # 假设MuJoCo中的执行器顺序与actuated_joint_names的顺序一致
                    self.data_mj.ctrl[:self.nv_pin] = tau_control

                    mujoco.mj_step(self.model_mj, self.data_mj)

                viewer.sync()

                time_until_next_step = self.model_mj.opt.timestep - (time.time() - step_start_time)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

# ... (main 函数不变)
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