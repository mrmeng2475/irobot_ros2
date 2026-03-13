import mujoco
import mujoco.viewer
import time

# 1. 加载模型 (返回 MjModel 对象)
model = mujoco.MjModel.from_xml_path("./asset/scene_irobot.xml")  # 替换为你的 XML 路径

# 2. 创建数据实例 (返回 MjData 对象)
data = mujoco.MjData(model)

# 3. 使用新的 mujoco.viewer 启动可视化窗口
# launch_passive 会在一个独立的进程中打开一个窗口，并返回一个viewer句柄
# 3. 使用新的 mujoco.viewer 启动可视化窗口
# launch_passive 会在一个独立的进程中打开一个窗口，并返回一个viewer句柄
with mujoco.viewer.launch_passive(model, data) as viewer:

    # --- 设置相机 (实现Z轴向下俯视) ---
    viewer.cam.lookat[:] = [0.34, -0.12, 0.0]      # 让相机聚焦于场景原点
    viewer.cam.distance = 1.5              # 调整相机与目标的距离，可以根据模型大小修改
    viewer.cam.azimuth = 90.0              # 方位角：90度会让Y轴在屏幕上朝上
    viewer.cam.elevation = -90.0             # 仰角：-90度表示从正上方往下看
    # ---------------

    # 初始化控制器
    data.ctrl[:] = 0

    try:
        while viewer.is_running():
            step_start = time.time()

            # 更新物理仿真 (传入 model 和 data)
            mujoco.mj_step(model, data)

            # 同步viewer状态
            # 在新版中，渲染是自动在后台进行的，
            # 我们只需要调用 sync() 来确保物理状态和渲染画面同步
            viewer.sync()

            # 控制循环频率 (更精确的方式)
            # 计算仿真步进花费的时间，然后只休眠剩余的时间
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("仿真已停止")