

### 拖动示教代码

import pinocchio as pin
import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# --- 1. 加载模型 (与原代码相同) ---
try:
    script_dir = os.path.dirname(__file__)
    xml_path = os.path.join(script_dir, 'asset', 'irobot.xml')
    model_mj = mujoco.MjModel.from_xml_path(xml_path)
except NameError:
    xml_path = './asset/irobot.xml'
    model_mj = mujoco.MjModel.from_xml_path(xml_path)
except Exception as e:
    print(f"加载 MuJoCo 模型失败: {e}")
    exit()

data_mj = mujoco.MjData(model_mj)

urdf_filename = os.path.expanduser("/home/mrmeng/work_space/irobot_ros2/src/irobot_mujoco/asset/urdf/irobot.urdf")
try:
    # 重要：Pinocchio 默认会加载一个浮动基座，我们需要从URDF中构建一个固定基座的模型
    # 这样自由度才能和 MuJoCo 匹配
    model_pin = pin.buildModelFromUrdf(urdf_filename)
    data_pin = model_pin.createData()
except Exception as e:
    print(f"加载 Pinocchio URDF 模型失败: {e}")
    exit()

# --- 2. 检查模型 ---
nq = model_pin.nq
nv = model_pin.nv
print(f"Pinocchio 关节数量 nq: {nq}, 自由度 nv: {nv}")
print(f"Pinocchio 关节名称: {model_pin.names}")
print(f"MuJoCo 关节数量 njnt: {model_mj.njnt}, 自由度 nv: {model_mj.nv}")
print(f"MuJoCo 关节名称: {[model_mj.joint(i).name for i in range(model_mj.njnt)]}")

# 验证自由度数量匹配
if nv != model_mj.nv:
    print(f"警告：Pinocchio 自由度 ({nv}) 与 MuJoCo 自由度 ({model_mj.nv}) 不匹配。")
    print("这可能是因为 Pinocchio 默认加载了浮动基座。URDF 的根连杆应该是固定的。")
    # 注意：如果你的URDF根连杆是浮动的，Pinocchio的nv会比MuJoCo多6。
    # 此时需要确保你的URDF是固定基座，或者在Pinocchio加载时指定。
    # 不过，对于机械臂，通常我们期望 nv 是相等的。
    # 如果 Pinocchio 的 nv 比 MuJoCo 的 nv 多 6, 并且你知道这是浮动基座导致的，
    # 你可能需要调整切片操作，例如 q[7:], v[6:]
    exit()

# --- 3. 修改点 A: 调整控制器参数 ---
# 我们不再需要Kp。我们保留Kd作为阻尼项。
# Kd的值可以根据需要调整，值越大，拖动时感觉越“粘稠”。
Kd = np.diag([30.0] * nv) # 可以适当减小Kd的值

# --- 4. 仿真参数 ---
dt = 0.001
T = 60.0 # 延长仿真时间以便有足够的时间进行示教
N = int(T / dt)

# --- 5. 修改点 B: 初始化用于存储示教数据的列表 ---
recorded_poses = []
print("\n--- 拖动示教模式已启动 ---")
print("在仿真窗口中，按住 Ctrl + 鼠标右键 拖动机器人。")
print("按 [空格键] 记录当前姿态。")

# --- 6. 启动 MuJoCo 可视化 ---
with mujoco.viewer.launch_passive(model_mj, data_mj) as viewer:
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -30
    viewer.cam.lookat = [0, 0, 1.0]

    # 仿真循环
    for step in range(N):
        t = step * dt

        # --- 修改点 C: 移除固定的期望轨迹 ---
        # 原有的 q_des, v_des, a_des 在循环内的赋值全部删除

        # 获取 MuJoCo 当前状态
        q = data_mj.qpos.copy()
        v = data_mj.qvel.copy()

        # --- 修改点 D: 重新定义控制律为“重力补偿+阻尼” ---
        # 1. 计算重力补偿力矩
        # 调用 rnea 并将期望速度和加速度设置为0，即可得到抵消重力所需的力矩
        # 注意：这里的输入是当前的关节位置 q
        tau_gravity = pin.rnea(model_pin, data_pin, q, np.zeros(nv), np.zeros(nv))

        # 2. 计算阻尼力矩
        # 这个力矩与当前速度成反比，用于增加系统的稳定性
        tau_damping = -Kd @ v

        # 3. 组合最终的控制律
        # 最终的力矩 = 重力补偿力矩 + 阻尼力矩
        tau_control = tau_gravity + tau_damping
        print(tau_control)
        # 应用到 MuJoCo
        data_mj.ctrl = tau_control

        # 仿真一步
        mujoco.mj_step(model_mj, data_mj)

        # 同步可视化
        viewer.sync()


        # 控制仿真速度（接近实时）
        time.sleep(1/120)


# 仿真结束后，打印所有记录的姿态
print("\n--- 拖动示教结束 ---")
if recorded_poses:
    print(f"共记录了 {len(recorded_poses)} 个姿态:")
    for i, pose in enumerate(recorded_poses):
        # 为了方便查看，可以只打印部分数据或进行格式化
        print(f"  姿态 {i+1}: " + np.array2string(pose, precision=3, separator=', '))
else:
    print("没有记录任何姿态。")