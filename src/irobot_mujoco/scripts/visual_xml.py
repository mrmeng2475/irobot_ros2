import mujoco
import mujoco.viewer

def visualize_mujoco_xml(xml_path):
    try:
        # 1. 从 XML 或 URDF 文件加载 MuJoCo 模型
        model = mujoco.MjModel.from_xml_path(xml_path)
        
        # 2. 创建与模型对应的状态数据 (包含位置、速度、受力等动态信息)
        data = mujoco.MjData(model)

        print(f"✅ 成功加载模型: {xml_path}")
        print("🚀 正在启动 MuJoCo 可视化界面...")
        print("💡 提示: 在界面中可以按 'Space' 键暂停/继续仿真，双击物体可施加外力。")

        # 3. 启动自带的交互式可视化器 (会阻塞程序直到窗口关闭)
        mujoco.viewer.launch(model, data)
        
    except Exception as e:
        print(f"❌ 加载或可视化失败，请检查文件路径或 XML 格式是否正确:\n{e}")

if __name__ == "__main__":
    # 在这里替换成你转换好的 xml 或原本的 urdf 文件路径
    # 例如: "robot_mujoco.xml" 或 "../urdf/irobot.urdf"
    xml_file_path = "/root/ros2_ws/irobot_ros2_humble/src/irobot_mujoco/asset/irobot_new.xml" 
    
    visualize_mujoco_xml(xml_file_path)