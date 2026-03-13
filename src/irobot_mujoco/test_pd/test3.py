import mujoco
import mujoco.viewer
import time
import os
import numpy as np
import matplotlib.pyplot as plt

# --- 加载模型 ---
try:
    script_dir = os.path.dirname(__file__)
    xml_path = os.path.join(script_dir, 'asset', 'scene_irobot.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
except NameError:
    # print("在交互式环境中运行，使用相对路径 './asset/scene_irobot.xml'")
    xml_path = './asset/scene_irobot.xml'
    model = mujoco.MjModel.from_xml_path(xml_path)
except Exception as e:
    print(f"加载模型失败: {e}")
    exit()

data = mujoco.MjData(model)

# --- 初始化渲染器 ---
# 对于某些版本的mujoco，需要一个有效的GL context才能初始化Renderer
# launch_passive会创建这个context，所以我们将renderer的初始化也放入with块中
renderer = None 

# 获取摄像头的ID
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'head_camera')
if camera_id == -1:
    print("未找到名为 'head_camera' 的摄像头，请检查 XML 文件")
    exit()

# 开启Matplotlib的交互模式用于实时显示深度图
plt.ion()
fig, ax = plt.subplots()
# 使用 'viridis' colormap 更适合显示深度
image_plot = ax.imshow(np.zeros((480, 640), dtype=np.float32), cmap='viridis') 
plt.title("Depth Image (meters)")
plt.colorbar(image_plot, ax=ax, label='Distance (m)')

# --- 启动仿真 ---
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # 在viewer的context激活后，安全地初始化renderer
        if renderer is None:
            renderer = mujoco.Renderer(model, height=480, width=640)

        # 设置 viewer 的摄像头
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = camera_id

        print("\n仿真已启动。")
        print("在运行此脚本的终端中按 Ctrl+C 来关闭仿真。\n")

        while viewer.is_running():
            step_start = time.time()

            # 执行一步仿真
            mujoco.mj_step(model, data)

            # --- 获取并处理深度信息 ---
            try:
                # 更新场景
                renderer.update_scene(data, camera=camera_id)
                
                # 【修正点 1】: 直接在构造函数中传入参数来创建视口
                viewport = mujoco.MjrRect(0, 0, renderer.width, renderer.height)

                # 【修正点 2】: 使用带下划线的 _mjr_context 属性
                # 读取深度缓冲区到预先分配的 numpy 数组中
                depth = np.empty((renderer.height, renderer.width), dtype=np.float32)
                mujoco.mjr_readPixels(None, depth, viewport, renderer._mjr_context)
                
                # mjr_readPixels 返回的深度值已是归一化的
                normalized_depth = depth  

                # 将归一化的深度值转换为米
                extent = model.vis.map.zfar - model.vis.map.znear
                depth_in_meters = model.vis.map.znear + normalized_depth * extent

                # 打印深度数据（每秒一次）
                if data.time % 1 < model.opt.timestep:
                    print(f"仿真时间: {data.time:.2f}s, 图像中心点的深度: {depth_in_meters[240, 320]:.3f} 米")

                # 使用 Matplotlib 显示深度图
                image_plot.set_data(depth_in_meters)
                image_plot.set_clim(vmin=np.min(depth_in_meters), vmax=np.max(depth_in_meters))
                fig.canvas.draw()
                fig.canvas.flush_events()

            except Exception as e:
                print(f"渲染或深度处理失败: {e}")
                break

            # 在viewer的锁内更新可视化选项
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False

            # 同步viewer以显示最新状态
            viewer.sync()

            # 稳定仿真速度
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("\n检测到 Ctrl+C，正在关闭仿真窗口...")

except Exception as e:
    print(f"仿真过程中发生错误: {e}")

finally:
    plt.ioff()
    plt.close(fig)
    if renderer:
        renderer.close()
    print("仿真已结束。")