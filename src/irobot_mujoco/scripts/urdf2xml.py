import mujoco

# 1. 直接读取 URDF 文件（MuJoCo 会在底层自动做转换）
model = mujoco.MjModel.from_xml_path("/root/ros2_ws/irobot_ros2_humble/src/irobot_description/meshes/irobot_visual.urdf")

# 2. 将转换好的模型保存为标准的 MuJoCo XML (MJCF) 格式
mujoco.mj_saveLastXML("/root/ros2_ws/irobot_ros2_humble/src/irobot_mujoco/scripts/irobot_mujoco.xml", model)