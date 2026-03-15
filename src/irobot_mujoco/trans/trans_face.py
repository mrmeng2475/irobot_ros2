import trimesh
mesh = trimesh.load('/root/ros2_ws/irobot_ros2_humble/src/irobot_description/meshes/right_arm_link2.STL')
mesh = mesh.simplify_quadric_decimation(face_count=19999)  # 设置目标面数
mesh.export('/root/ros2_ws/irobot_ros2_humble/src/irobot_description/meshes/right_arm_link2.STL')