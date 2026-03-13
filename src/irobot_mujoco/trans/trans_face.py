import trimesh
mesh = trimesh.load('link_clip1_R.STL')
mesh = mesh.simplify_quadric_decimation(face_count=19999)  # 设置目标面数
mesh.export('link_clip1_R.STL')