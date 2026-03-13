#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from irobot_interfaces.msg import DualArmTargets

import sys, select, termios, tty
import threading

# 帮助信息
HELP_MSG = """
--------------------------------------------------
双臂末端遥操作节点
--------------------------------------------------
左臂控制 (Left Arm):    右臂控制 (Right Arm):
   W: 前进 (X+)             I: 前进 (X+)
   S: 后退 (X-)             K: 后退 (X-)
   A: 左移 (Y+)             J: 左移 (Y+)
   D: 右移 (Y-)             L: 右移 (Y-)
   Q: 上升 (Z+)             U: 上升 (Z+)
   E: 下降 (Z-)             O: 下降 (Z-)
--------------------------------------------------
按 CTRL-C 退出
"""

# 按键与移动方向的映射
# 格式：'按键': ([x, y, z]方向向量, '控制的手臂')
KEY_MAPPING = {
    'w': ([1, 0, 0], 'left'),   's': ([-1, 0, 0], 'left'),
    'a': ([0, 1, 0], 'left'),   'd': ([0, -1, 0], 'left'),
    'q': ([0, 0, 1], 'left'),   'e': ([0, 0, -1], 'left'),

    'i': ([1, 0, 0], 'right'),  'k': ([-1, 0, 0], 'right'),
    'j': ([0, 1, 0], 'right'),  'l': ([0, -1, 0], 'right'),
    'u': ([0, 0, 1], 'right'),  'o': ([0, 0, -1], 'right'),
}

class DualArmTeleopNode(Node):
    def __init__(self):
        super().__init__('dual_arm_teleop_node')

        # 创建发布者，发布双臂目标
        self.target_publisher = self.create_publisher(
            DualArmTargets,
            '/dual_arm/ik_targets',
            10)

        # 定义移动步长和发布频率
        self.move_step = 0.01  # 每按一次键移动0.01米
        self.publish_rate = 50.0 # 每秒发布50次目标

        # 初始化左右臂的目标位置
        self.left_target = np.array([0.382, 0.16, 1.1831])
        self.right_target = np.array([0.382, -0.16, 1.1831])

        # 用于保存终端原始设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("✅ 遥操作节点已启动")
        print(HELP_MSG)

        # 创建一个定时器，以固定频率发布目标
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_targets)

    def get_key(self):
        """非阻塞地获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        """主循环，处理键盘输入并更新目标"""
        while rclpy.ok():
            key = self.get_key()
            if key in KEY_MAPPING:
                direction, arm = KEY_MAPPING[key]
                direction = np.array(direction) * self.move_step

                if arm == 'left':
                    self.left_target += direction
                elif arm == 'right':
                    self.right_target += direction

                # 打印当前的目标位置以提供反馈
                print(f"左臂目标: {self.left_target[0]:.3f}, {self.left_target[1]:.3f}, {self.left_target[2]:.3f} | "
                      f"右臂目标: {self.right_target[0]:.3f}, {self.right_target[1]:.3f}, {self.right_target[2]:.3f}", end='\r')

            elif key == '\x03':  # CTRL-C
                break

    def publish_targets(self):
        """定时器回调函数，发布当前的目标"""
        msg = DualArmTargets()
        msg.left_target.x, msg.left_target.y, msg.left_target.z = self.left_target
        msg.right_target.x, msg.right_target.y, msg.right_target.z = self.right_target
        self.target_publisher.publish(msg)

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
        # 确保在退出时恢复终端设置
        teleop_node.restore_terminal_settings()
        teleop_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()