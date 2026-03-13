import pinocchio as pin
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import crocoddyl # 导入 Crocoddyl

# --- 加载模型 (与原代码相同) ---
# 1. 加载 MuJoCo 模型
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

# 2. 加载 Pinocchio 模型（URDF）
urdf_filename = os.path.expanduser("/home/mrmeng/work_space/irobot_ros2/src/irobot_mujoco/asset/urdf/irobot.urdf")
try:
    model_pin = pin.Model()
    pin.buildModelFromUrdf(urdf_filename, model_pin)
    data_pin = model_pin.createData()
except Exception as e:
    print(f"加载 Pinocchio URDF 模型失败: {e}")
    exit()

# 3. 检查模型 (与原代码相同)
nq = model_pin.nq
nv = model_pin.nv
nu = nv # 控制输入的维度等于速度的维度
print(f"Pinocchio 关节数 (nq): {nq}, 自由度 (nv): {nv}")

# --- MPC 参数设置 ---
T = 100  # 预测时域的长度 (Horizon Length)
dt = 0.01  # 每个预测步长的时间间隔
sim_dt = 0.001 # 仿真器的步长，可以比MPC步长更精细
mpc_update_rate = int(dt / sim_dt) # 每隔多少个仿真步更新一次MPC

# --- 搭建 MPC 问题 (使用 Crocoddyl) ---
# 1. 定义状态和驱动模型
# 状态向量 x = [q, v]，包含关节位置和速度
state = crocoddyl.StateMultibody(model_pin)
# 驱动模型，假设我们可以完全控制所有关节的力矩
actuation = crocoddyl.ActuationModelFull(state)

# 2. 定义代价函数 (Cost Functions)
# 这是MPC的核心：我们通过代价函数告诉求解器什么是“好”的控制
running_costs = crocoddyl.CostModelSum(state)
terminal_costs = crocoddyl.CostModelSum(state)

# 代价权重 (Weights) - 这些是需要调试的关键参数！
w_q = np.array([1.0]*7 + [1.0]*7 + [0.01]*6) # 对右臂和左臂的位置误差给予高权重
w_v = np.array([0.1] * nv) # 对速度误差给予较低权重
w_u = 1e-4 # 对控制力矩（能量消耗）给予很小的权重，鼓励平滑控制
w_q_term = np.array([100.0]*7 + [100.0]*7 + [1.0]*6) # 在时域末端，我们希望更精确地到达目标位置
w_v_term = np.array([10.0] * nv) # 在时域末端，速度也更重要

# a. 运行代价 (Running Cost): 在预测时域的每一步都会计算
#    目标: 跟踪期望状态 & 最小化控制能量
q_ref = np.zeros(nq) # 期望位置 (后面会在循环中动态更新)
v_ref = np.zeros(nv) # 期望速度
x_ref = np.concatenate([q_ref, v_ref])

# 状态代价: 惩罚 (x - x_ref)^2
state_cost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelWeightedQuad(np.concatenate([w_q, w_v])**2), x_ref, nu)
# 控制代价: 惩罚 u^2
ctrl_cost = crocoddyl.CostModelControl(state, crocoddyl.ActivationModelWeightedQuad(np.array([w_u]*nu)**2), nu)

running_costs.addCost("state_reg", state_cost, 1.0)
running_costs.addCost("ctrl_reg", ctrl_cost, 1.0)

# b. 终端代价 (Terminal Cost): 只在预测时域的最后一步计算
#    目标: 在时域末端尽可能精确地达到期望状态
x_term_ref = x_ref.copy()
terminal_state_cost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelWeightedQuad(np.concatenate([w_q_term, w_v_term])**2), x_term_ref, nu)
terminal_costs.addCost("terminal_state", terminal_state_cost, 1.0)


# 3. 创建动力学模型和求解器
# 将连续的动力学和代价函数离散化
running_model = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, running_costs), dt
)
terminal_model = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, terminal_costs), dt
)

# 4. 定义优化问题 (Shooting Problem)
x0 = np.concatenate([data_mj.qpos.copy(), data_mj.qvel.copy()]) # 初始状态
problem = crocoddyl.ShootingProblem(x0, [running_model] * T, terminal_model)

# 5. 创建求解器
# FDDP (Feasibility-Driven Differential Dynamic Programming) 是一种高效的求解器
solver = crocoddyl.SolverFDDP(problem)
# solver.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose()]) # 可选：用于调试的详细日志

# --- 仿真参数 ---
T_sim = 10.0  # 总仿真时长
N_sim = int(T_sim / sim_dt) # 总仿真步数

# --- 启动 MuJoCo 可视化 ---
with mujoco.viewer.launch_passive(model_mj, data_mj) as viewer:
    # 设置初始视角
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -30
    viewer.cam.lookat = [0, 0, 1.0]

    # 初始化MPC的猜测解 (零力矩)
    u_guess = [np.zeros(nu)] * T
    x_guess = [x0] * (T + 1)

    # 仿真循环
    for i in range(N_sim):
        t = i * sim_dt

        # --- MPC 控制更新 ---
        # MPC不需要每个仿真步都计算，根据设定好的频率更新
        if i % mpc_update_rate == 0:
            # 1. 设置当前MPC问题的初始状态
            current_x = np.concatenate([data_mj.qpos.copy(), data_mj.qvel.copy()])
            problem.x0 = current_x

            # 2. 定义并更新期望轨迹 (这里我们让目标随时间变化，以展示MPC的跟踪能力)
            # 例如：让右臂和左臂的第一个关节做一个正弦运动
            amp = 0.8
            freq = 0.5
            q_des_joint_0 = amp * np.sin(2 * np.pi * freq * t)
            v_des_joint_0 = 2 * np.pi * freq * amp * np.cos(2 * np.pi * freq * t)

            # 更新整个时域内的期望轨迹
            for k in range(T):
                t_k = t + k * dt # 预测的未来时间
                q_des_k = np.zeros(nq)
                q_des_k[0] = amp * np.sin(2 * np.pi * freq * t_k) # 右臂第1关节
                q_des_k[7] = amp * np.sin(2 * np.pi * freq * t_k + np.pi) # 左臂第1关节 (反相)

                v_des_k = np.zeros(nv)
                v_des_k[0] = 2 * np.pi * freq * amp * np.cos(2 * np.pi * freq * t_k)
                v_des_k[7] = 2 * np.pi * freq * amp * np.cos(2 * np.pi * freq * t_k + np.pi)
                
                # 更新代价函数中的参考状态
                x_ref_k = np.concatenate([q_des_k, v_des_k])
                problem.runningModels[k].cost.costs["state_reg"].cost.ref = x_ref_k

            # 更新终端代价的参考状态
            q_des_term = np.zeros(nq)
            q_des_term[0] = amp * np.sin(2 * np.pi * freq * (t + T * dt))
            q_des_term[7] = amp * np.sin(2 * np.pi * freq * (t + T * dt) + np.pi)
            v_des_term = np.zeros(nv) # 终端速度期望为0
            problem.terminalModel.cost.costs["terminal_state"].cost.ref = np.concatenate([q_des_term, v_des_term])

            # 3. 求解优化问题
            # 使用上一次的解作为本次的初始猜测 (Warm Start)，可以大大提高求解速度
            solved = solver.solve(x_guess, u_guess)
            if not solved:
                print("MPC 求解失败!")
            
            # 更新下一次迭代的猜测解
            x_guess = solver.xs.tolist()
            u_guess = solver.us.tolist()
        
        # 4. 应用MPC计算出的第一个控制指令
        tau_control = solver.us[0]
        data_mj.ctrl = tau_control

        # 仿真一步
        mujoco.mj_step(model_mj, data_mj)

        # 同步可视化
        viewer.sync()

        # 输出（每 100 步）
        if i % 100 == 0:
            q_current = data_mj.qpos
            q_target = problem.runningModels[0].cost.costs["state_reg"].cost.ref[:nq]
            print(f"Time: {t:.2f}s | Joint 0 Pos: {q_current[0]:.3f} (Target: {q_target[0]:.3f})")
            # print(f"Control Torque (Joint 0): {tau_control[0]:.3f}")

        # 控制仿真速度
        time.sleep(max(0, sim_dt - viewer.timer.seconds()))