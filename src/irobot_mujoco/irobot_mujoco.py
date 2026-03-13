#!/usr/bin/python3

# 该代码是进行位置控制的仿真代码，要使用该套代码需对该代码的xml文件进行更改，将irobot_pd.xml文件改为irobot.xml

import os
import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
import numpy as np
import time
from sensor_msgs.msg import JointState

class DualArmIkNode(Node):
    def __init__(self):
        super().__init__('dual_arm_ik_node')

        # --- 加载 MuJoCo 模型 ---
        try:
            self.model = mujoco.MjModel.from_xml_path("./asset/scene_irobot.xml")
            self.data = mujoco.MjData(self.model)
            self.get_logger().info("✅ 模型加载成功! Path: ./asset/scene_irobot.xml")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            raise e

        # --- 获取瓶子 body 的信息 ---
        # try:
        #     self.bottle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'bottle')
        #     self.bottle_qpos_start_addr = self.model.jnt_qposadr[self.model.body_jntadr[self.bottle_id]]
        # except ValueError:
        #     self.get_logger().error("错误：在模型中找不到名为 'bottle' 的body。请检查XML文件。")
        #     raise

        # --- 获取机器人关节信息 ---
        # 假设左右臂各有 7 个关节，名称为 joint1_L 到 joint7_L 和 joint1_R 到 joint7_R
        self.joint_names = [f'joint{i}_{side}' for side in ['L', 'R'] for i in range(1, 8)]
      
        self.joint_ids = []
        self.joint_actuator_ids = []
        for name in self.joint_names:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id < 0:
                self.get_logger().error(f"错误：找不到关节 {name}")
                raise ValueError(f"Joint {name} not found in model")
            self.joint_ids.append(jnt_id)
            self.joint_actuator_ids.append(jnt_id)
            # self.joint_qpos_addrs.append(self.model.jnt_qposadr[jnt_id])
        self.get_logger().info(f"✅ 找到关节: {self.joint_names}")

        # --- 初始化关节角度 ---
        self.qpos = np.zeros(len(self.joint_names))  # 存储最新的关节角度
        self.is_joint_state_received = False

        # --- 订阅 /joint_states 话题 ---
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("📥 订阅 '/joint_states' 以接收关节角度")

    def joint_state_callback(self, msg):
        """处理 /joint_states 话题的回调函数"""
        try:
            # 确保消息中的关节顺序与模型一致
            joint_positions = []
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_positions.append(msg.position[idx])
                else:
                    self.get_logger().warn(f"关节 {name} 不在 /joint_states 中")
                    return
            self.qpos = np.array(joint_positions)
            self.is_joint_state_received = True
            self.get_logger().info(f"✅ 接收到关节状态: {np.round(self.qpos, 4)}")
        except Exception as e:
            self.get_logger().error(f"处理关节状态失败: {e}")

    def run_simulation(self):
        """运行 MuJoCo 仿真"""
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.800]
            viewer.cam.distance = 2
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -20.0

            # self.data.ctrl[:] = 0  # 初始化控制器

            last_print_time = time.time()

            while viewer.is_running() and rclpy.ok():
                step_start = time.time()

                # --- 更新关节角度到 MuJoCo ---
                if self.is_joint_state_received:
                    # for idx, qpos_addr in enumerate(self.joint_qpos_addrs):
                    #     self.data.qpos[qpos_addr] = self.qpos[idx]
                    self.get_logger().info(f"已更新 MuJoCo 关节角度: {np.round(self.qpos, 4)}")

                # --- 运行仿真一步 ---
                
                self.data.ctrl[self.joint_actuator_ids]=self.qpos
                mujoco.mj_step(self.model, self.data)
                # --- 打印瓶子位置 ---
                # current_time = time.time()
                # if current_time - last_print_time > 0.5:
                #     current_pos = self.data.qpos[self.bottle_qpos_start_addr:self.bottle_qpos_start_addr+3]
                #     self.get_logger().info(f"仿真时间: {self.data.time:.2f}s, 瓶子当前Z轴位置: {current_pos[2]:.4f}")
                #     last_print_time = current_time

                # --- 同步 viewer ---
                viewer.sync()

                # --- 处理 ROS 2 回调 ---
                rclpy.spin_once(self, timeout_sec=0.0)

                # 更新查看器
                viewer.sync()

                # 手动控制频率 (~60Hz)
                time.sleep(1/60)

    def destroy_node(self):
        """清理节点"""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmIkNode()
    try:
        node.run_simulation()
    except KeyboardInterrupt:
        node.get_logger().info("仿真已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()