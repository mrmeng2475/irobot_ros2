#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from irobot_interfaces.srv import GetTargetPose

class TaskCommanderNode(Node):
    def __init__(self):
        super().__init__('task_commander_node')
        
        # 1. 创建服务客户端
        self.cli = self.create_client(GetTargetPose, 'get_target_pose')
        
        # 等待服务上线
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待视觉追踪服务 /get_target_pose 上线...')

        # 2. 创建广播发布者 (使用标准的 PoseStamped 格式)
        # 修改为发布紫色和橙色方块的话题
        self.purple_pub = self.create_publisher(PoseStamped, '/target_pose/cube_purple', 10)
        self.orange_pub = self.create_publisher(PoseStamped, '/target_pose/cube_orange', 10)

        self.purple_pose = None
        self.orange_pose = None

    def get_pose_sync(self, object_name):
        """同步调用服务：发送请求并死等结果返回"""
        req = GetTargetPose.Request()
        req.object_name = object_name
        
        self.get_logger().info(f'---> 开始请求寻找并测算 [{object_name}] ...')
        future = self.cli.call_async(req)
        
        # 阻塞当前线程，直到服务节点返回结果
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

    def broadcast_poses(self):
        """定时器回调：持续广播两个物体的位姿"""
        if self.purple_pose is None or self.orange_pose is None:
            return

        now = self.get_clock().now().to_msg()
        
        # 打包 cube_purple 消息
        msg_purple = PoseStamped()
        msg_purple.header.stamp = now
        msg_purple.header.frame_id = 'base_link'  # 机器人基座坐标系的名称，视你的URDF而定
        msg_purple.pose = self.purple_pose
        
        # 打包 cube_orange 消息
        msg_orange = PoseStamped()
        msg_orange.header.stamp = now
        msg_orange.header.frame_id = 'base_link'
        msg_orange.pose = self.orange_pose
        
        # 发布
        self.purple_pub.publish(msg_purple)
        self.orange_pub.publish(msg_orange)


def main(args=None):
    rclpy.init(args=args)
    node = TaskCommanderNode()

    # ================= 任务编排逻辑 =================
    
    # 步骤 1：呼叫服务寻找 cube_purple
    res_purple = node.get_pose_sync('cube_purple')
    if not res_purple.success:
        node.get_logger().error('❌ 寻找 cube_purple 失败，任务终止。')
        node.destroy_node()
        rclpy.shutdown()
        return
        
    # # === 保留原有的偏置逻辑：给第一个物体(现为 purple) 的 x 轴增加偏置 ===
    # res_purple.pose.position.x += 0.03
    
    node.get_logger().info('✅ 成功获取 cube_purple 位姿并加上偏置！cube_purple 位姿为: ' + str(res_purple.pose))
    node.purple_pose = res_purple.pose

    # 步骤 2：呼叫服务寻找 cube_orange
    res_orange = node.get_pose_sync('cube_orange')
    if not res_orange.success:
        node.get_logger().error('❌ 寻找 cube_orange 失败，任务终止。')
        node.destroy_node()
        rclpy.shutdown()
        return
        
    # # === 保留原有的偏置逻辑：给第二个物体(现为 orange) 增加偏置 ===
    # res_orange.pose.position.x += 0.01
    # res_orange.pose.position.y += 0.03
    
    node.get_logger().info('✅ 成功获取 cube_orange 位姿并加上偏置！cube_orange 位姿为: ' + str(res_orange.pose))
    node.orange_pose = res_orange.pose

    # 步骤 3：两个都找到了，开启定时器持续广播
    node.get_logger().info('🎉 两个方块均已定位（含偏置）！正在以 10Hz 频率持续广播它们的全局坐标...')
    node.create_timer(0.1, node.broadcast_poses)
    
    # 让节点持续运行（Spin），维持定时器广播
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()