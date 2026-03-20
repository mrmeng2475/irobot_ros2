#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
import math
import time
import threading
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

# ================= 新增：引入多线程和回调组 =================
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose

from irobot_interfaces.msg import ObjectPose, HeadCommand
# 注意：这里导入了刚刚让你新建的自定义服务
from irobot_interfaces.srv import GetTargetPose 
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class HeadTrackServiceNode(Node):
    STATE_IDLE = 0       
    STATE_SEARCHING = 1  
    STATE_TRACKING = 2   

    def __init__(self):
        super().__init__('head_track_service_node')
        
        # 使用可重入回调组，保证服务阻塞时，定时器和话题回调能继续在后台线程运行
        self.cb_group = ReentrantCallbackGroup()

        self.state = self.STATE_IDLE
        self.search_phase = 0  
        
        self.target_name = ""        
        self.is_centered = False     
        self.error_threshold = 0.01 
        
        self.smooth_factor_pan = 0.1 
        self.smooth_factor_tilt = 0.05

        self.pan_limit = [math.radians(-90.0), math.radians(90.0)]
        self.tilt_limit = [math.radians(0.0), math.radians(60.0)]
        self.sweep_limit = math.radians(45.0) 
        self.sweep_speed = math.radians(10.0) * 0.05 

        self.cmd_pan = 0.0
        self.cmd_tilt = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.last_seen_time = time.time()  
        self.last_raw_x = 0.0              
        self.last_raw_y = 0.0              
        
        self.startup_done = False
        self.is_target_lost = False

        # --- 服务同步相关变量 ---
        self.task_event = threading.Event()  # 用于阻塞服务的事件锁
        self.result_pose = None              # 存储计算好的结果
        self.search_success = False          # 标记搜寻是否成功

        # --- Pinocchio 初始化 ---
        try:
            urdf_filename = os.path.expanduser("/root/ros2_ws/irobot_ros2_humble/src/irobot_description/urdf/irobot.urdf")
            self.model_pin = pin.buildModelFromUrdf(urdf_filename)
            self.data_pin = self.model_pin.createData()
            self.head_link3_id = self.model_pin.getFrameId('head_link3')
            self.get_logger().info(f"✅ Pinocchio 加载成功，head_link3 ID: {self.head_link3_id}")
        except Exception as e:
            self.get_logger().error(f"❌ Pinocchio 加载失败: {e}")
            raise e

        self.q_pin_current = np.zeros(self.model_pin.nq)
        r_z = R.from_euler('z', -90, degrees=True).as_matrix()
        r_x = R.from_euler('x', -90, degrees=True).as_matrix()
        self.R_cam2head = r_z @ r_x 

        # --- 通信接口配置 (全部挂载到 cb_group 允许多线程并发) ---
        self.control_timer = self.create_timer(0.05, self.control_loop, callback_group=self.cb_group)
        self.sub_obj = self.create_subscription(ObjectPose, '/detected_objects', self.object_callback, 10, callback_group=self.cb_group)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10, callback_group=self.cb_group)
        
        self.pub_head = self.create_publisher(HeadCommand, '/head/cmd_joints', 10)
        
        # 创建核心服务
        self.srv = self.create_service(GetTargetPose, 'get_target_pose', self.get_target_pose_callback, callback_group=self.cb_group)

        self.get_logger().info("🎯 追踪服务节点启动完毕！等待提供 /get_target_pose 服务...")

    def publish_cmd(self):
        self.cmd_pan = max(min(self.cmd_pan, self.pan_limit[1]), self.pan_limit[0])
        self.cmd_tilt = max(min(self.cmd_tilt, self.tilt_limit[1]), self.tilt_limit[0])
        
        cmd_msg = HeadCommand()
        cmd_msg.head_joint1 = self.cmd_pan
        cmd_msg.head_joint2 = self.cmd_tilt
        self.pub_head.publish(cmd_msg)

    # ================= 核心服务回调机制 =================
    def get_target_pose_callback(self, request, response):
        """当外部节点请求寻找物体时，触发此服务"""
        self.get_logger().info(f"🛎️ 收到服务请求：寻找并测算 [{request.object_name}] 的位置...")
        
        # 1. 设置状态，触发后台的主循环去寻找
        self.target_name = request.object_name
        self.state = self.STATE_SEARCHING
        self.search_phase = 0
        self.is_centered = False
        self.search_success = False
        self.result_pose = None
        
        # 清空事件锁，开始阻塞当前服务线程
        self.task_event.clear()
        
        # 2. 阻塞等待！(因为用了多线程，这里不会卡死 ROS 节点，控制循环会继续跑)
        self.task_event.wait() 
        
        # 3. 被唤醒后，检查结果
        if self.search_success and self.result_pose is not None:
            self.get_logger().info(f"✅ 服务完成！已成功获取 [{request.object_name}] 的 6D 位姿。")
            response.success = True
            response.pose = self.result_pose
        else:
            self.get_logger().warn(f"❌ 服务完成：未能在环境中找到 [{request.object_name}]。")
            response.success = False
            response.pose = Pose() # 返回一个空位姿

        # 任务结束，机器人进入发呆状态
        self.state = self.STATE_IDLE
        self.target_name = ""
        
        return response
    # =====================================================

    def joint_state_callback(self, msg: JointState):
        try:
            if 'head_joint1' in msg.name:
                self.current_pan = msg.position[msg.name.index('head_joint1')]
            if 'head_joint2' in msg.name:
                self.current_tilt = msg.position[msg.name.index('head_joint2')]
            
            for i, name in enumerate(msg.name):
                if self.model_pin.existJointName(name):
                    pin_joint_id = self.model_pin.getJointId(name)
                    pin_idx_q = self.model_pin.joints[pin_joint_id].idx_q
                    self.q_pin_current[pin_idx_q] = msg.position[i]
        except ValueError:
            pass

    def control_loop(self):
        if not self.startup_done:
            self.cmd_pan = 0.0
            self.cmd_tilt = math.radians(45.0)
            self.publish_cmd()
            self.startup_done = True
            return

        if self.state == self.STATE_SEARCHING:
            self.cmd_tilt = math.radians(45.0) 
            if self.search_phase == 0:
                self.cmd_pan -= self.sweep_speed
                if self.cmd_pan <= -self.sweep_limit:
                    self.cmd_pan = -self.sweep_limit
                    self.search_phase = 1 
            elif self.search_phase == 1:
                self.cmd_pan += self.sweep_speed
                if self.cmd_pan >= self.sweep_limit:
                    self.cmd_pan = self.sweep_limit
                    self.search_phase = 2 
            elif self.search_phase == 2:
                self.cmd_pan -= self.sweep_speed
                if self.cmd_pan <= 0.0:
                    self.cmd_pan = 0.0
                    self.publish_cmd()
                    
                    # --- 巡视完毕没找到，释放服务锁 ---
                    self.search_success = False
                    self.task_event.set() 
                    return
            self.publish_cmd()

        elif self.state == self.STATE_TRACKING:
            if time.time() - self.last_seen_time > 0.1:
                if not self.is_target_lost:
                    self.is_target_lost = True
                    self.is_centered = False
                    self.cmd_pan = self.current_pan
                    self.cmd_tilt = self.current_tilt
                    self.publish_cmd()

    def compute_target_base_pose(self, msg: ObjectPose) -> Pose:
        """计算基座坐标系并直接返回标准的 Pose 消息类型"""
        try:
            pin.forwardKinematics(self.model_pin, self.data_pin, self.q_pin_current)
            pin.updateFramePlacements(self.model_pin, self.data_pin)

            oMf_head = self.data_pin.oMf[self.head_link3_id]
            pos_head_world = oMf_head.translation
            mat_head_world = oMf_head.rotation

            mat_cam_world = mat_head_world @ self.R_cam2head
            pos_cam_world = pos_head_world 

            pos_obj_cam = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            quat_obj_cam = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            mat_obj_cam = R.from_quat(quat_obj_cam).as_matrix()

            pos_obj_world = mat_cam_world @ pos_obj_cam + pos_cam_world
            mat_obj_world = mat_cam_world @ mat_obj_cam
            quat_obj_world = R.from_matrix(mat_obj_world).as_quat()
            
            # 打包为标准的 Pose 消息
            final_pose = Pose()
            final_pose.position.x = float(pos_obj_world[0])
            final_pose.position.y = float(pos_obj_world[1])
            final_pose.position.z = float(pos_obj_world[2])
            final_pose.orientation.x = float(quat_obj_world[0])
            final_pose.orientation.y = float(quat_obj_world[1])
            final_pose.orientation.z = float(quat_obj_world[2])
            final_pose.orientation.w = float(quat_obj_world[3])
            
            return final_pose
            
        except Exception as e:
            self.get_logger().error(f"坐标系转换失败: {e}")
            return None

    def object_callback(self, msg: ObjectPose):
        if self.state == self.STATE_IDLE or msg.object_name != self.target_name:
            return

        error_x = msg.pose.position.x
        error_y = msg.pose.position.y
        depth_z = msg.pose.position.z 

        if error_x == self.last_raw_x and error_y == self.last_raw_y:
            return
            
        self.last_raw_x = error_x
        self.last_raw_y = error_y
        self.last_seen_time = time.time()

        if self.state == self.STATE_TRACKING and self.is_target_lost:
            self.is_target_lost = False

        if self.state == self.STATE_SEARCHING:
            self.state = self.STATE_TRACKING
            self.is_target_lost = False

        if depth_z <= 0.01:
            depth_z = 0.01

        # ================= 居中并完成任务 =================
        if abs(error_x) < self.error_threshold and abs(error_y) < self.error_threshold:
            if not self.is_centered:
                self.is_centered = True
                
                # 计算位姿
                pose_result = self.compute_target_base_pose(msg)
                
                if pose_result is not None:
                    # 将结果塞入变量，通知服务线程放行！
                    self.result_pose = pose_result
                    self.search_success = True
                    self.task_event.set() 
            return  
        # ==================================================
        
        self.is_centered = False

        exact_angle_pan = math.atan2(error_x, depth_z)
        exact_angle_tilt = math.atan2(error_y, depth_z)

        delta_pan = -exact_angle_pan * self.smooth_factor_pan
        delta_tilt = exact_angle_tilt * self.smooth_factor_tilt

        max_step = 0.05 
        delta_pan = max(min(delta_pan, max_step), -max_step)
        delta_tilt = max(min(delta_tilt, max_step), -max_step)

        self.cmd_pan += delta_pan
        self.cmd_tilt += delta_tilt

        self.publish_cmd()

def main(args=None):
    rclpy.init(args=args)
    node = HeadTrackServiceNode()
    
    # 【必须使用 MultiThreadedExecutor】：否则服务阻塞时，话题将无法接收数据！
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()