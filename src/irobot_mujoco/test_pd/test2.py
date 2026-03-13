import mujoco
import mujoco.viewer
import time
import os

# --- 加载模型 ---
# 为了让脚本在任何位置都能正确找到模型文件，建议使用文件的绝对路径
# 下面的代码会假设asset文件夹和你的python脚本在同一个目录里
try:
    script_dir = os.path.dirname(__file__)
    xml_path = os.path.join(script_dir, 'asset', 'scene_irobot.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
except NameError:
    # 如果你在交互式解释器中运行，__file__ 未定义，使用相对路径
    print("在交互式环境中运行，使用相对路径 './asset/scene_irobot.xml'")
    xml_path = './asset/scene_irobot.xml'
    model = mujoco.MjModel.from_xml_path(xml_path)
except Exception as e:
    print(f"加载模型失败: {e}")
    exit()

data = mujoco.MjData(model)

# --- 启动仿真 ---
# 使用 try...except 块来捕获 Ctrl+C (KeyboardInterrupt)
try:
    # 使用 with mujoco.viewer.launch_passive(...) 来确保窗口能被正确关闭
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置viewer的摄像头
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'head_camera')

        print("\n仿真已启动。")
        print("在仿真窗口激活的情况下，你可以进行交互。")
        print("在运行此脚本的终端中按 Ctrl+C 来关闭仿真。\n")

        while viewer.is_running():
            step_start = time.time()

            # 执行一步仿真
            mujoco.mj_step(model, data)

            # 在viewer的锁内更新可视化选项，这是线程安全的做法
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False

            # 同步viewer以显示最新状态
            viewer.sync()

            # 通过休眠来匹配模型的物理时间步，稳定仿真速度
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    # 当用户在终端按下 Ctrl+C 时，程序会跳转到这里
    print("\n检测到 Ctrl+C，正在关闭仿真窗口...")

finally:
    # with 语句会自动处理 viewer.close()，这里只是打印一条最终信息
    print("仿真已结束。")