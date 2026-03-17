#!/usr/bin/python3
# ros2 topic pub --once /head/cmd_joints irobot_interfaces/msg/HeadCommand "{head_joint1: 0.0, head_joint2: 0.0}"
# ros2 topic pub --once /track_cmd std_msgs/msg/String "{data: 'cup1'}"


import rclpy
from rclpy.node import Node
import math
import time

from irobot_interfaces.msg import ObjectPose, HeadCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

class HeadTrackNode(Node):
    # --- 定义机器人的三种状态 ---
    STATE_IDLE = 0       # 待机/发呆
    STATE_SEARCHING = 1  # 左右巡视寻找
    STATE_TRACKING = 2   # 盯住目标居中

    def __init__(self):
        super().__init__('head_track_node')

        # 状态机变量
        self.state = self.STATE_IDLE
        self.search_phase = 0  # 搜寻阶段：0(向左-45), 1(向右+45), 2(回正0)
        
        self.target_name = ""        
        self.is_centered = False     
        self.error_threshold = 0.01 
        
        self.smooth_factor_pan = 0.1 
        self.smooth_factor_tilt = 0.05

        # 关节限位
        self.pan_limit = [math.radians(-90.0), math.radians(90.0)]
        self.tilt_limit = [math.radians(0.0), math.radians(60.0)]

        # 巡视相关配置
        self.sweep_limit = math.radians(45.0) # 搜寻时的左右最大边界
        self.sweep_speed = math.radians(10.0) * 0.05 # 搜寻速度：10度/秒 (在20Hz下的单步步长)

        # 内部控制指令缓存
        self.cmd_pan = 0.0
        self.cmd_tilt = 0.0
        
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # 防僵死机制
        self.last_seen_time = time.time()  
        self.last_raw_x = 0.0              
        self.last_raw_y = 0.0              
        
        # 初始位姿标志位
        self.startup_done = False
        
        # 新增：用于标记在追踪过程中是否暂时丢失视野
        self.is_target_lost = False

        # ================= 核心改进：20Hz 控制主循环 =================
        # 原本的看门狗升级为主循环，负责初始化、自动搜寻和超时保护
        self.control_timer = self.create_timer(0.05, self.control_loop)
        # ============================================================

        self.sub_cmd = self.create_subscription(String, '/track_cmd', self.track_cmd_callback, 10)
        self.sub_obj = self.create_subscription(ObjectPose, '/detected_objects', self.object_callback, 10)
        self.sub_joints = self.create_subscription(JointState, '/head/joint_states', self.joint_state_callback, 10)

        self.pub_head = self.create_publisher(HeadCommand, '/head/cmd_joints', 10)
        self.pub_status = self.create_publisher(Bool, '/track_status', 10)

        self.get_logger().info("🎯 追踪节点启动完毕！准备进入初始姿态...")

    def publish_cmd(self):
        """统一的指令下发工具函数"""
        # 下发前做最后一次绝对安全限幅
        self.cmd_pan = max(min(self.cmd_pan, self.pan_limit[1]), self.pan_limit[0])
        self.cmd_tilt = max(min(self.cmd_tilt, self.tilt_limit[1]), self.tilt_limit[0])
        
        cmd_msg = HeadCommand()
        cmd_msg.head_joint1 = self.cmd_pan
        cmd_msg.head_joint2 = self.cmd_tilt
        self.pub_head.publish(cmd_msg)

    def track_cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd.lower() == 'stop':
            self.target_name = ""
            self.state = self.STATE_IDLE
            self.is_centered = False
            self.get_logger().info("🛑 停止任务，进入待机模式。")
        else:
            self.target_name = cmd
            self.is_centered = False
            self.state = self.STATE_SEARCHING # 收到指令，默认先进入搜寻模式
            self.search_phase = 0             # 从头开始巡视
            self.get_logger().info(f"👀 开始寻找目标: {self.target_name}，启动全景扫描！")

    def joint_state_callback(self, msg: JointState):
        try:
            if 'head_joint1' in msg.name:
                idx1 = msg.name.index('head_joint1')
                self.current_pan = msg.position[idx1]
            if 'head_joint2' in msg.name:
                idx2 = msg.name.index('head_joint2')
                self.current_tilt = msg.position[idx2]
        except ValueError:
            pass

    def control_loop(self):
        """20Hz 控制主循环：处理巡视、看门狗和初始化"""
        
        # 1. 启动初始化：让头抬起来到 (0, 45度)
        if not self.startup_done:
            self.cmd_pan = 0.0
            self.cmd_tilt = math.radians(45.0)
            self.publish_cmd()
            self.startup_done = True
            self.get_logger().info("🤖 初始位姿已下发: Pan=0°, Tilt=45°")
            return

        # 2. 状态机：如果正在巡视
        if self.state == self.STATE_SEARCHING:
            self.cmd_tilt = math.radians(45.0) # 巡视时强制保持抬头45度
            
            # 第一阶段：向 -45度 转动
            if self.search_phase == 0:
                self.cmd_pan -= self.sweep_speed
                if self.cmd_pan <= -self.sweep_limit:
                    self.cmd_pan = -self.sweep_limit
                    self.search_phase = 1 # 触底反弹，进入第二阶段
            
            # 第二阶段：向 +45度 转动
            elif self.search_phase == 1:
                self.cmd_pan += self.sweep_speed
                if self.cmd_pan >= self.sweep_limit:
                    self.cmd_pan = self.sweep_limit
                    self.search_phase = 2 # 触顶反弹，准备回正
            
            # 第三阶段：回正到 0度
            elif self.search_phase == 2:
                self.cmd_pan -= self.sweep_speed
                if self.cmd_pan <= 0.0:
                    self.cmd_pan = 0.0
                    # 巡视结束，一无所获
                    self.state = self.STATE_IDLE
                    self.target_name = ""
                    self.get_logger().warn("❌ 搜寻一圈完毕，环境中无此目标。")
            
            self.publish_cmd()

        
        # 3. 状态机：如果正在锁定追踪，执行看门狗逻辑
        elif self.state == self.STATE_TRACKING:
            # 放宽超时到 500ms，且只在第一次发现丢失时刹车，防止重复发指令
            if time.time() - self.last_seen_time > 0.1:
                if not self.is_target_lost:
                    self.get_logger().warn(f"⚠️ 目标丢失超 100ms！固定头部位置，等待目标重新出现...")
                    self.is_target_lost = True
                    self.is_centered = False
                    
                    # 刹车切断惯性，强行把指令拉回当前的物理真实位置并保持
                    self.cmd_pan = self.current_pan
                    self.cmd_tilt = self.current_tilt
                    self.publish_cmd()

    def object_callback(self, msg: ObjectPose):
        # 没接到指令，或者看到的不是目标，直接无视
        if self.state == self.STATE_IDLE or msg.object_name != self.target_name:
            return

        error_x = msg.pose.position.x
        error_y = msg.pose.position.y
        depth_z = msg.pose.position.z 

        # 防僵死过滤
        if error_x == self.last_raw_x and error_y == self.last_raw_y:
            return
            
        self.last_raw_x = error_x
        self.last_raw_y = error_y
        self.last_seen_time = time.time()

        # 【新增】：如果之前因为掉帧挂起了，现在重新看到了，解除挂起
        if self.state == self.STATE_TRACKING and self.is_target_lost:
            self.get_logger().info(f"🔄 重新捕获目标 [{self.target_name}]，继续追踪！")
            self.is_target_lost = False

        # 如果原本在搜索，看到目标立刻切换为追踪状态
        if self.state == self.STATE_SEARCHING:
            self.get_logger().info(f"🎯 巡视中发现目标 [{self.target_name}]！切入锁定追踪模式！")
            self.state = self.STATE_TRACKING
            self.is_target_lost = False

        if depth_z <= 0.01:
            depth_z = 0.01

        # 死区居中判断
        if abs(error_x) < self.error_threshold and abs(error_y) < self.error_threshold:
            if not self.is_centered:
                self.get_logger().info(f"✅ [{self.target_name}] 已绝对居中!")
                self.is_centered = True
                status_msg = Bool()
                status_msg.data = True
                self.pub_status.publish(status_msg)
            return  
        
        self.is_centered = False

        # 几何算法居中逼近
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
    node = HeadTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()