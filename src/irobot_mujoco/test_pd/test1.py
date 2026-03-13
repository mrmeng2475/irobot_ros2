import mujoco
import mujoco.viewer
import numpy as np  # 需要导入numpy库，用于数据处理
import time

# 1. 加载模型 (返回 MjModel 对象)
try:
    model = mujoco.MjModel.from_xml_path("./asset/scene_irobot.xml")
except Exception as e:
    print(f"加载模型失败，错误: {e}")
    exit()

# 2. 创建数据实例 (返回 MjData 对象)
data = mujoco.MjData(model)

# --- 在viewer启动前，获取我们需要的信息 ---
try:
    # 获取瓶子body的ID
    bottle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'bottle')
    # 通过body_id找到它对应的第一个关节(freejoint)的地址
    # 然后通过关节地址找到它在qpos(位置)数组中的起始地址
    bottle_qpos_start_addr = model.jnt_qposadr[model.body_jntadr[bottle_id]]
except ValueError:
    print("错误：在模型中找不到名为 'bottle' 的body。请检查XML文件。")
    exit()
# --- 信息获取完毕 ---


# 3. 使用新的 mujoco.viewer 启动可视化窗口
with mujoco.viewer.launch_passive(model, data) as viewer:

    # --- 设置相机 ---
    viewer.cam.lookat[:] = [0, 0, 0.800]
    viewer.cam.distance = 2
    viewer.cam.azimuth = 180.0
    viewer.cam.elevation = -20.0
    # ---------------

    # 初始化控制器
    data.ctrl[:] = 0

    # 打印瓶子的初始位置 (x, y, z)
    # initial_pos = data.qpos[bottle_qpos_start_addr:bottle_qpos_start_addr+3]
    # print(f"仿真开始。瓶子初始位置 (x,y,z): {np.round(initial_pos, 4)}")
    # print("--------------------------------------------------")

    # 用于控制打印频率的计时器
    last_print_time = time.time()

    try:
        while viewer.is_running():
            step_start = time.time()

            # 更新物理仿真 (传入 model 和 data)
            mujoco.mj_step(model, data)

            # --- 每隔大约0.5秒，打印一次瓶子的当前位置 ---
            # current_time = time.time()
            # if current_time - last_print_time > 0.5:
            #     current_pos = data.qpos[bottle_qpos_start_addr:bottle_qpos_start_addr+3]
            #     print(f"仿真时间: {data.time:.2f}s, 瓶子当前Z轴位置: {current_pos[2]:.4f}")
            #     last_print_time = current_time
            # --- 打印代码结束 ---

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("仿真已停止")