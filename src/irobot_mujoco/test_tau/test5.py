import pinocchio as pin
import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# --- 加载模型 ---
# 1. 加载 MuJoCo 模型
try:
    script_dir = os.path.dirname(__file__)
    xml_path = os.path.join(script_dir, 'asset', 'irobot.xml')
    model_mj = mujoco.MjModel.from_xml_path(xml_path)
except NameError:
    # print("在交互式环境中运行，使用相对路径 './asset/irobot.xml'")
    xml_path = './asset/irobot.xml'
    model_mj = mujoco.MjModel.from_xml_path(xml_path)
except Exception as e:
    print(f"加载 MuJoCo 模型失败: {e}")
    exit()

data_mj = mujoco.MjData(model_mj)

# 2. 加载 Pinocchio 模型（URDF）
urdf_filename = os.path.expanduser("/home/mrmeng/work_space/irobot_ros2/src/irobot_mujoco/asset/urdf/irobot.urdf")
try:
    model_pin = pin.Model()
    pin.buildModelFromUrdf(urdf_filename, model_pin)
    data_pin = model_pin.createData()
except Exception as e:
    print(f"加载 Pinocchio URDF 模型失败: {e}")
    exit()

# 3. 检查模型
nq = model_pin.nq  # 应为 20
nv = model_pin.nv  # 应为 20
print(f"Pinocchio 关节名称: {model_pin.names}")
print(f"MuJoCo 关节名称: {[model_mj.joint(i).name for i in range(model_mj.njnt)]}")

# 验证关节数量匹配
if nv != model_mj.nv:
    print(f"警告：Pinocchio 自由度 ({nv}) 与 MuJoCo 自由度 ({model_mj.nv}) 不匹配，请检查 URDF 和 MJCF 文件")
    exit()

# 4. 设置控制器参数
Kp = np.diag([100.0] * nv)
Kd = np.diag([10.0] * nv)

# 5. 仿真参数
dt = 0.001
T = 5.0
N = int(T / dt)

# 6. 初始化轨迹
q_des = np.zeros(nq)
v_des = np.zeros(nv)
a_des = np.zeros(nv)

# 7. 启动 MuJoCo 可视化
with mujoco.viewer.launch_passive(model_mj, data_mj) as viewer:
    # 设置初始视角（可选）
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -30
    viewer.cam.lookat = [0, 0, 1.0]

    # 仿真循环
    for step in range(N):
        t = step * dt

        # 设置期望轨迹
        for i in range(14):
            q_des[i] = 0.3
            v_des[i] = 0.0
            a_des[i] = 0.0
        q_des[14:20] = 0.0
        v_des[14:20] = 0.0
        a_des[14:20] = 0.0

        # 获取 MuJoCo 当前状态
        q = data_mj.qpos.copy()
        v = data_mj.qvel.copy()

        # 计算逆动力学（Pinocchio）
        tau_ff = pin.rnea(model_pin, data_pin, q_des, v_des, a_des)

        # 计算反馈项
        q_error = q_des - q
        v_error = v_des - v
        tau_fb = Kp @ q_error + Kd @ v_error

        # 组合控制律
        # tau_control = tau_ff + tau_fb
        tau_control = tau_ff + tau_fb

        # 应用到 MuJoCo
        data_mj.ctrl = tau_control

        # 仿真一步
        mujoco.mj_step(model_mj, data_mj)

        # 同步可视化
        viewer.sync()

        # 输出（每 100 步）
        if step % 100 == 0:
            print(f"时间: {t:.3f}s")
            print("实际位置 (q, 右臂示例):", q[:14])
            print("期望位置 (q_des, 右臂示例):", q_des[:14])
            print("总控制力/力矩:", tau_control)

        # 控制仿真速度（接近实时）
        time.sleep(dt)