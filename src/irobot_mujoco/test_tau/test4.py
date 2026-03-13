import pinocchio as pin
import numpy as np
import os

# 1. 加载 URDF 模型
urdf_filename = os.path.expanduser("/home/mrmeng/work_space/irobot_ros2/src/irobot_mujoco/asset/urdf/irobot.urdf")  # 确保 URDF 文件路径正确
model = pin.Model()
pin.buildModelFromUrdf(urdf_filename, model)  # 加载动力学模型
data = model.createData()

# # 加载几何模型（可选，用于可视化或碰撞检测）
# geom_model = pin.GeometryModel()
# pin.buildGeomFromUrdf(model, urdf_filename, pin.GeometryType.VISUAL, geom_model)  # 加载可视化几何

# 2. 检查模型
nq = model.nq  # 关节位置维度（应为 20）
nv = model.nv  # 自由度（应为 20）
print(f"关节名称: {model.names}")
print(f"关节数量: {model.njoints}, 自由度: {nv}")

# 3. 设置控制器参数
Kp = np.diag([100.0] * nv)  # 比例增益（可调）
Kd = np.diag([10.0] * nv)   # 微分增益（可调）

# 4. 设置期望轨迹和实际状态
t = 0.0  # 当前时间
dt = 0.001  # 时间步长
q_des = np.zeros(nq)  # 期望位置
v_des = np.zeros(nv)  # 期望速度
a_des = np.zeros(nv)  # 期望加速度

# 为左右臂的主要关节（joint1_R 到 joint7_R, joint1_L 到 joint7_L）设置正弦轨迹
for i in range(14):  # 前 14 个自由度（左右臂各 7 DOF）
    q_des[i] = 0.5 * np.sin(2 * np.pi * 0.1 * t)  # 振幅 0.5 rad，频率 0.1 Hz
    v_des[i] = 0.5 * 2 * np.pi * 0.1 * np.cos(2 * np.pi * 0.1 * t)
    a_des[i] = -0.5 * (2 * np.pi * 0.1)**2 * np.sin(2 * np.pi * 0.1 * t)

# 夹爪和颈部关节保持静止（可根据任务调整）
q_des[14:20] = 0.0
v_des[14:20] = 0.0
a_des[14:20] = 0.0

# 模拟实际状态（带小偏差）
q = q_des + np.random.randn(nq) * 0.01  # 实际位置（添加噪声模拟误差）
v = v_des + np.random.randn(nv) * 0.01  # 实际速度

# 5. 计算逆动力学（前馈项）
tau_ff = pin.rnea(model, data, q_des, v_des, a_des)

# 6. 计算反馈项（PD 控制）
q_error = q_des - q
v_error = v_des - v
tau_fb = Kp @ q_error + Kd @ v_error

# 7. 组合控制律
tau_control = tau_ff + tau_fb

# 8. 输出结果
print("前馈项 (tau_ff):", tau_ff)
print("反馈项 (tau_fb):", tau_fb)
print("总控制力/力矩 (tau_control):", tau_control)
print("左臂控制力/力矩 (joint1_L to joint7_L):", tau_control[7:14])
print("右臂控制力/力矩 (joint1_R to joint7_R):", tau_control[0:7])
print("夹爪控制力/力矩 (clip joints):", tau_control[14:18])
print("颈部控制力/力矩 (neck joints):", tau_control[18:20])

# 9. 可选：考虑外部力（例如夹爪抓取）
f_ext = [pin.Force.Zero() for _ in range(model.njoints)]
# 示例：右臂夹爪 (joint_clip1_R) 施加 10N 向下力
clip1_R_idx = model.getJointId("joint_clip1_R")
if clip1_R_idx < model.njoints:  # 确保索引有效
    f_ext[clip1_R_idx] = pin.Force(np.array([0.0, 0.0, -10.0, 0.0, 0.0, 0.0]))
    tau_ff_ext = pin.rnea(model, data, q_des, v_des, a_des, f_ext)
    tau_control_ext = tau_ff_ext + tau_fb
    print("考虑外部力的总控制力/力矩:", tau_control_ext)
else:
    print("警告：joint_clip1_R 未找到，请检查关节名称")