#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
# 假设您的自定义接口都在 irobot_interfaces 中
from irobot_interfaces.msg import DualArmTargets, GripperCommand

import sys, select, termios, tty
import threading

# 更新后的帮助信息
HELP_MSG = """
--------------------------------------------------
双臂及夹爪遥操作节点
--------------------------------------------------
左臂控制 (Left Arm):    右臂控制 (Right Arm):
   W: 前进 (X+)             I: 前进 (X+)
   S: 后退 (X-)             K: 后退 (X-)
   A: 左移 (Y+)             J: 左移 (Y+)
   D: 右移 (Y-)             L: 右移 (Y-)
   Q: 上升 (Z+)             U: 上升 (Z+)
   E: 下降 (Z-)             O: 下降 (Z-)

左夹爪 (Left Gripper):  右夹爪 (Right Gripper):
   R: 张开 (Open)           Y: 张开 (Open)
   F: 闭合 (Close)          H: 闭合 (Close)
--------------------------------------------------
按 CTRL-C 退出
"""

# 手臂按键与移动方向的映射
ARM_KEY_MAPPING = {
    'w': ([1, 0, 0], 'left'),   's': ([-1, 0, 0], 'left'),
    'a': ([0, 1, 0], 'left'),   'd': ([0, -1, 0], 'left'),
    'q': ([0, 0, 1], 'left'),   'e': ([0, 0, -1], 'left'),

    'i': ([1, 0, 0], 'right'),  'k': ([-1, 0, 0], 'right'),
    'j': ([0, 1, 0], 'right'),  'l': ([0, -1, 0], 'right'),
    'u': ([0, 0, 1], 'right'),  'o': ([0, 0, -1], 'right'),
}

# 夹爪按键与开合方向的映射 (1:张开, -1:闭合)
GRIPPER_KEY_MAPPING = {
    'r': (1, 'left'),    'f': (-1, 'left'),
    'y': (1, 'right'),   'h': (-1, 'right'),
}


class DualArmTeleopNode(Node):
    def __init__(self):
        super().__init__('dual_arm_teleop_node')

        # --- 发布者定义 ---
        # 1. 双臂IK目标发布者
        self.target_publisher = self.create_publisher(
            DualArmTargets,
            '/dual_arm/ik_targets',
            10)
        # 2. 左右夹爪指令发布者
        self.left_gripper_publisher = self.create_publisher(GripperCommand, '/gripper/cmd_left', 10)
        self.right_gripper_publisher = self.create_publisher(GripperCommand, '/gripper/cmd_right', 10)

        # --- 参数和状态变量 ---
        # 移动步长
        self.move_step = 0.01
        self.gripper_step = 0.1 # 夹爪每次开合的步长 (0.0 to 1.0)
        # 发布频率
        self.publish_rate = 50.0

        # 初始化目标位置
        self.left_target = np.array([0.24, 0.16, 1.0805])
        self.right_target = np.array([0.24, -0.16, 1.0805])
        self.left_gripper_position = 0.0  # 0.0=闭合, 1.0=完全张开
        self.right_gripper_position = 0.0

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("✅ 遥操作节点已启动 (含夹爪控制)")
        print(HELP_MSG)

        # 创建定时器，以固定频率发布手臂目标
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_arm_targets)
        # 初始发布一次夹爪状态
        self.publish_gripper_commands()

    def get_key(self):
        """非阻塞地获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key.lower() # 转换为小写以匹配映射

    def run_keyboard_control(self):
        """主循环，处理键盘输入并更新目标"""
        while rclpy.ok():
            key = self.get_key()

            # 处理手臂移动
            if key in ARM_KEY_MAPPING:
                direction, arm = ARM_KEY_MAPPING[key]
                direction = np.array(direction) * self.move_step

                if arm == 'left':
                    self.left_target += direction
                elif arm == 'right':
                    self.right_target += direction

            # 处理夹爪开合
            elif key in GRIPPER_KEY_MAPPING:
                direction, arm = GRIPPER_KEY_MAPPING[key]
                change = direction * self.gripper_step

                if arm == 'left':
                    self.left_gripper_position += change
                    self.left_gripper_position = np.clip(self.left_gripper_position, -1.0, 0.6)
                elif arm == 'right':
                    self.right_gripper_position += change
                    self.right_gripper_position = np.clip(self.right_gripper_position, -0.6, 1.0)
                
                # 夹爪指令是离散的，立即发布
                self.publish_gripper_commands()

            elif key == '\x03':  # CTRL-C
                break
            
            # 持续打印状态信息
            self.print_status()

    def print_status(self):
        """格式化打印当前所有目标的状态"""
        left_arm_str = f"左臂:({self.left_target[0]:.3f}, {self.left_target[1]:.3f}, {self.left_target[2]:.3f})"
        right_arm_str = f"右臂:({self.right_target[0]:.3f}, {self.right_target[1]:.3f}, {self.right_target[2]:.3f})"
        left_gripper_str = f"左爪:{self.left_gripper_position:.1f}"
        right_gripper_str = f"右爪:{self.right_gripper_position:.1f}"
        # 使用\r实现原地更新
        print(f"{left_arm_str} | {right_arm_str} | {left_gripper_str} | {right_gripper_str}    ", end='\r')

    def publish_arm_targets(self):
        """定时器回调函数，发布手臂的目标"""
        msg = DualArmTargets()
        msg.left_target.x, msg.left_target.y, msg.left_target.z = self.left_target
        msg.right_target.x, msg.right_target.y, msg.right_target.z = self.right_target
        self.target_publisher.publish(msg)

    def publish_gripper_commands(self):
        """发布左右夹爪的指令"""
        left_msg = GripperCommand()
        left_msg.position = self.left_gripper_position
        self.left_gripper_publisher.publish(left_msg)

        right_msg = GripperCommand()
        right_msg.position = self.right_gripper_position
        self.right_gripper_publisher.publish(right_msg)

    def restore_terminal_settings(self):
        """恢复终端的原始设置"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = DualArmTeleopNode()

    # 将键盘读取放在一个独立的线程中，以避免阻塞ROS的主循环
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop_node,))
    spin_thread.start()

    try:
        teleop_node.run_keyboard_control()
    except Exception as e:
        teleop_node.get_logger().error(f"键盘控制循环异常: {e}")
    finally:
        # 确保在退出时恢复终端设置并清理资源
        print("\n正在退出...")
        teleop_node.restore_terminal_settings()
        # 发送关闭信号给 rclpy.spin()
        rclpy.shutdown()
        spin_thread.join()
        teleop_node.destroy_node()

if __name__ == '__main__':
    main()