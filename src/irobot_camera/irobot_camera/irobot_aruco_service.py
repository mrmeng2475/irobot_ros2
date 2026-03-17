import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# 导入新建的服务类型和 ROS 2 官方的 Pose 消息
from irobot_interfaces.srv import GetObjectPose
from geometry_msgs.msg import Pose

class ArucoDetectorServiceNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_service_node')

        # ================= 配置参数 =================
        self.ARUCO_DICT = cv2.aruco.DICT_6X6_250
        self.MARKER_SIZE = 0.02
        
        # 定义 ID 到物体名称的映射字典
        self.id_to_name = {
            40: "bottle1",
            98: "cup1"
        }

        # 用一个字典来存储当前画面中最新识别到的物体位姿
        self.current_detected_poses = {}

        # 1. 创建 ROS 2 服务端，服务名为 get_object_pose
        self.srv = self.create_service(GetObjectPose, 'get_object_pose', self.handle_get_object_pose)

        # ================= 初始化 RealSense =================
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)

        # 获取相机内参
        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.array(intrinsics.coeffs)

        self.get_logger().info(f"相机内参加载成功，服务 'get_object_pose' 已启动")

        # ================= 初始化 ArUco =================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 依然需要定时器来不断读取相机画面，防止缓存溢出并更新 OpenCV 窗口
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 1. 获取图像帧
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. 检测标记
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # 每次刷新画面时，清空旧的识别记录
        latest_poses = {}

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 3. 对每个检测到的标记进行位姿估计
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
                )
                
                # 绘制坐标轴
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.MARKER_SIZE * 1.5)

                marker_id = int(ids[i][0])

                # 判断 ID 是否在我们的目标字典中
                if marker_id in self.id_to_name:
                    object_name = self.id_to_name[marker_id]
                    
                    t_x, t_y, t_z = tvec[i][0]
                    r_mat, _ = cv2.Rodrigues(rvec[i][0])
                    quat = R.from_matrix(r_mat).as_quat()

                    # 组装 Pose
                    pose_msg = Pose()
                    pose_msg.position.x = float(t_x)
                    pose_msg.position.y = float(t_y)
                    pose_msg.position.z = float(t_z)
                    pose_msg.orientation.x = float(quat[0])
                    pose_msg.orientation.y = float(quat[1])
                    pose_msg.orientation.z = float(quat[2])
                    pose_msg.orientation.w = float(quat[3])

                    # 保存最新位姿到字典中
                    latest_poses[object_name] = pose_msg

        # 更新全局记录（确保服务回调读取的是这一帧的结果）
        self.current_detected_poses = latest_poses

        # OpenCV 窗口显示实时画面
        cv2.imshow('RealSense ArUco Service', frame)
        cv2.waitKey(1)

    # ================= 服务回调函数 =================
    def handle_get_object_pose(self, request, response):
        target_name = request.object_name
        self.get_logger().info(f"收到查询请求: {target_name}")

        # 检查请求的物体当前是否在视野中
        if target_name in self.current_detected_poses:
            response.success = True
            response.pose = self.current_detected_poses[target_name]
            self.get_logger().info(f"成功获取 {target_name} 的位姿，准备返回")
        else:
            response.success = False
            self.get_logger().warning(f"当前画面未识别到 {target_name}")

        return response

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()