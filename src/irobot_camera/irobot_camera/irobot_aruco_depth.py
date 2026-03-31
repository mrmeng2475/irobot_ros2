import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

from irobot_interfaces.msg import ObjectPose
from geometry_msgs.msg import Pose, Point, Quaternion

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # ================= 配置参数 =================
        self.ARUCO_DICT = cv2.aruco.DICT_6X6_250
        self.MARKER_SIZE = 0.02215
        
        self.id_to_name = {
            40: "bottle1",
            98: "cup1",
            124: "cube_orange",
            203: "cube_purple"
        }

        self.publisher_ = self.create_publisher(ObjectPose, 'detected_objects', 10)

        # ================= 初始化 RealSense (新增深度流配置) =================
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # 开启彩色流
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # 【新增】开启深度流 (必须与彩色流分辨率一致，方便对齐)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.profile = self.pipeline.start(self.config)

        # 【新增】创建对齐对象：将深度画面严丝合缝地贴合到彩色画面上
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # 获取相机内参 (既给 OpenCV 用，也给 RealSense 3D 投影用)
        color_stream = self.profile.get_stream(rs.stream.color)
        self.rs_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        self.camera_matrix = np.array([
            [self.rs_intrinsics.fx, 0, self.rs_intrinsics.ppx],
            [0, self.rs_intrinsics.fy, self.rs_intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.array(self.rs_intrinsics.coeffs)

        self.get_logger().info("相机内参与深度模块加载成功")

        # ================= 初始化 ArUco =================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        self.history_pose = {}

        half_size = self.MARKER_SIZE / 2.0
        self.obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        self.get_logger().info("开始检测... ")
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        
        # 【新增】处理流对齐：确保获取到的深度帧和彩色帧像素是一一对应的
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                
                # ================= 1. OpenCV 计算旋转姿态 (保持原有的平滑追踪) =================
                if marker_id in self.history_pose:
                    rvec_guess, tvec_guess = self.history_pose[marker_id]
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs,
                        rvec=rvec_guess, tvec=tvec_guess,
                        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE
                    )
                else:
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs, 
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                
                if not success:
                    continue
                
                self.history_pose[marker_id] = (rvec, tvec)

                # ================= 2. RealSense 硬件测距 (核心修改点) =================
                # a. 计算二维码在画面中的中心像素坐标 (u, v)
                corner_points = corners[i][0]
                center_x = int(np.mean(corner_points[:, 0]))
                center_y = int(np.mean(corner_points[:, 1]))

                # b. 从深度流中直接读取该像素点的物理距离 (Z轴深度，单位：米)
                # 注：为了防止中心点刚好是个反光导致的黑洞(深度为0)，你可以加一个容错，这里取直接读取
                depth = depth_frame.get_distance(center_x, center_y)

                # 如果测不到深度（比如太近或太远超出量程），跳过这帧
                if depth <= 0.01 or depth >= 4.0:
                    continue

                # c. 将 2D 像素坐标和深度值，反向投影为相机坐标系下的 3D 物理坐标 [X, Y, Z]
                # rs2_deproject_pixel_to_point 是 RealSense 底层极其精准的几何转换函数
                true_3d_point = rs.rs2_deproject_pixel_to_point(self.rs_intrinsics, [center_x, center_y], depth)
                
                # 将硬件算出来的真实 XYZ 覆盖掉 OpenCV 算出来的 tvec
                true_tvec = np.array([[true_3d_point[0]], [true_3d_point[1]], [true_3d_point[2]]])

                # ======================================================================

                # 绘制坐标轴 (使用真实深度的位置画轴)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, true_tvec, self.MARKER_SIZE * 1.5)

                if marker_id in self.id_to_name:
                    object_name = self.id_to_name[marker_id]
                    
                    r_mat, _ = cv2.Rodrigues(rvec)
                    quat = R.from_matrix(r_mat).as_quat() 
                    
                    msg = ObjectPose()
                    msg.object_name = object_name
                    
                    # 使用 RealSense 硬件测量出的坐标系赋值
                    msg.pose.position.x = float(true_3d_point[0])
                    msg.pose.position.y = float(true_3d_point[1])
                    msg.pose.position.z = float(true_3d_point[2])
                    
                    print(f"[{object_name}] 硬件实测深度(Z): {true_3d_point[2]:.3f} 米")
                    
                    msg.pose.orientation.x = float(quat[0])
                    msg.pose.orientation.y = float(quat[1])
                    msg.pose.orientation.z = float(quat[2])
                    msg.pose.orientation.w = float(quat[3])

                    self.publisher_.publish(msg)

        cv2.imshow('RealSense ArUco Pose Estimation', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()