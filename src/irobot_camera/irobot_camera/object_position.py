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
        self.bottle_pub = self.create_publisher(PoseStamped, '/target_pose/bottle1', 10)
        self.cup_pub = self.create_publisher(PoseStamped, '/target_pose/cup1', 10)

        self.bottle_pose = None
        self.cup_pose = None

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
        if self.bottle_pose is None or self.cup_pose is None:
            return

        now = self.get_clock().now().to_msg()
        
        # 打包 bottle1 消息
        msg_bottle = PoseStamped()
        msg_bottle.header.stamp = now
        msg_bottle.header.frame_id = 'base_link'  # 机器人基座坐标系的名称，视你的URDF而定
        msg_bottle.pose = self.bottle_pose
        
        # 打包 cup1 消息
        msg_cup = PoseStamped()
        msg_cup.header.stamp = now
        msg_cup.header.frame_id = 'base_link'
        msg_cup.pose = self.cup_pose
        
        # 发布
        self.bottle_pub.publish(msg_bottle)
        self.cup_pub.publish(msg_cup)


def main(args=None):
    rclpy.init(args=args)
    node = TaskCommanderNode()

    # ================= 任务编排逻辑 =================
    
    # 步骤 1：呼叫服务寻找 bottle1
    res_bottle = node.get_pose_sync('bottle1')
    if not res_bottle.success:
        node.get_logger().error('❌ 寻找 bottle1 失败，任务终止。')
        node.destroy_node()
        rclpy.shutdown()
        return
        
    # === 新增：给 bottle1 的 x 轴增加 2cm 偏置 ===
    res_bottle.pose.position.x += 0.03
    
    node.get_logger().info('✅ 成功获取 bottle1 位姿并加上偏置！bottle1 位姿为: ' + str(res_bottle.pose))
    node.bottle_pose = res_bottle.pose

    # 步骤 2：呼叫服务寻找 cup1
    res_cup = node.get_pose_sync('cup1')
    if not res_cup.success:
        node.get_logger().error('❌ 寻找 cup1 失败，任务终止。')
        node.destroy_node()
        rclpy.shutdown()
        return
        
    # === 新增：给 cup1 的 y 轴减去 1cm 偏置 ===
    res_cup.pose.position.x += 0.01
    res_cup.pose.position.y += 0.03
    
    node.get_logger().info('✅ 成功获取 cup1 位姿并加上偏置！cup1 位姿为: ' + str(res_cup.pose))
    node.cup_pose = res_cup.pose

    # 步骤 3：两个都找到了，开启定时器持续广播
    node.get_logger().info('🎉 两个物体均已定位（含偏置）！正在以 10Hz 频率持续广播它们的全局坐标...')
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