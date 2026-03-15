import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# 导入我们刚刚创建的新消息类型以及 ROS 2 官方的 Pose 消息
from irobot_interfaces.msg import ObjectPose
from geometry_msgs.msg import Pose, Point, Quaternion

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # ================= 配置参数 =================
        self.ARUCO_DICT = cv2.aruco.DICT_6X6_250
        self.MARKER_SIZE = 0.02
        
        # 定义 ID 到物体名称的映射字典
        self.id_to_name = {
            40: "bottle1",
            98: "cup1"
        }

        # 1. 创建 ROS 2 发布者，话题名为 /detected_objects
        self.publisher_ = self.create_publisher(ObjectPose, 'detected_objects', 10)

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

        self.get_logger().info(f"相机内参加载成功")

        # ================= 初始化 ArUco =================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("开始检测... ")

        # 设置定时器，按 30Hz 的频率处理图像
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

                # 4. 判断 ID 是否在我们的目标字典中
                if marker_id in self.id_to_name:
                    object_name = self.id_to_name[marker_id]
                    
                    # 提取平移向量
                    t_x, t_y, t_z = tvec[i][0]
                    
                    # 将 OpenCV 的旋转向量(rvec) 转换为旋转矩阵，再转为四元数
                    r_mat, _ = cv2.Rodrigues(rvec[i][0])
                    quat = R.from_matrix(r_mat).as_quat() # 格式为 [x, y, z, w]

                    # 5. 组装 ROS 2 消息并发布
                    msg = ObjectPose()
                    msg.object_name = object_name
                    
                    msg.pose.position.x = float(t_x)
                    msg.pose.position.y = float(t_y)
                    msg.pose.position.z = float(t_z)
                    
                    msg.pose.orientation.x = float(quat[0])
                    msg.pose.orientation.y = float(quat[1])
                    msg.pose.orientation.z = float(quat[2])
                    msg.pose.orientation.w = float(quat[3])

                    self.publisher_.publish(msg)
                    self.get_logger().info(f"发布识别结果: {object_name} (ID: {marker_id}), 距离Z: {t_z:.3f}m")

        # OpenCV 窗口显示
        cv2.imshow('RealSense ArUco Pose Estimation', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # 节点销毁时关闭相机和窗口
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