#!/usr/bin/env python3
import os
import json
import time
import threading
from openai import OpenAI

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 导入你自定义的接口消息，注意这里引入了 GripperControl
from irobot_interfaces.msg import DualArmPoseTargets, GripperControl, HeadCommand

class DecisionConnectNode(Node):
    def __init__(self):
        super().__init__('decision_connect_node')
        
        self.get_logger().info("🧠 iRobot 决策大脑节点正在启动...")

        # ==========================================
        # 1. 初始化 ROS2 发布者 (发送指令给刚才新写的轨迹规划层)
        # ==========================================
        # 话题名修改为与 trajectory_planner_node 订阅的一致
        self.posture_pub = self.create_publisher(DualArmPoseTargets, '/llm/posture_cmd', 10)
        self.gripper_pub = self.create_publisher(GripperControl, '/llm/gripper_cmd', 10)
        
        self.head_pub = self.create_publisher(HeadCommand, '/camera/head_cmd', 10)
        self.voice_pub = self.create_publisher(String, '/communicate/voice_cmd', 10)

        # ==========================================
        # 2. 状态缓存 (非常重要：防止单臂移动时另一只臂归零)
        # ==========================================
        # 记录大模型下发的“期望目标”，初始化为双臂的预备姿态
        self.target_left_pose = {
            "position": [0.382, 0.33394, 1.2435],
            "orientation": [0.3429, -0.000, 0.000, 0.93937]
        }
        self.target_right_pose = {
            "position": [0.382, -0.33394, 1.2435],
            "orientation": [-0.3429, -0.000, 0.000, 0.93937]
        }

        # 维护当前世界状态 (喂给大模型看)
        self.world_state = {
            "environment_info": {
                "desk_height": 1.0131  # 保持桌面高度约束
            },
            "scene_objects": [
                {"name": "purple_box", "position": [0.43, 0.15, 1.03], "orientation": [0, 0, 0, 1]}, 
                {"name": "blue_box", "position": [0.38, 0.0, 1.03], "orientation": [0, 0, 0, 1]},
                {"name": "orange_box", "position": [0.43, -0.15, 1.03], "orientation": [0, 0, 0, 1]},
            ],
            "robot_state": {
                "left_arm": {
                    "end_effector_pose": self.target_left_pose["position"] + self.target_left_pose["orientation"], 
                    "gripper_state": "open",
                    "workspace_limits": {"x": [0.0, 0.6], "y": [0.0, 0.6], "z": [1.0131, 1.5]}
                },
                "right_arm": {
                    "end_effector_pose": self.target_right_pose["position"] + self.target_right_pose["orientation"], 
                    "gripper_state": "open",
                    "workspace_limits": {"x": [0.0, 0.6], "y": [-0.6, 0.0], "z": [1.0131, 1.5]}
                }
            }
        }

        # ==========================================
        # 3. 初始化大语言模型客户端
        # ==========================================
        try:
            self.llm_client = OpenAI(
                api_key=os.getenv("DASHSCOPE_API_KEY"),
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            )
        except Exception as e:
            self.get_logger().error(f"❌ 初始化 OpenAI 客户端时出错: {e}")
            raise e

        self.conversation_history = []
        self.MAX_HISTORY_LENGTH = 5

        # 启动交互线程
        self.cli_thread = threading.Thread(target=self.terminal_interaction_loop)
        self.cli_thread.daemon = True
        self.cli_thread.start()

    def process_user_command(self, user_command):
        system_prompt = """
        你是一个人形双臂机器人(irobot)的大脑(Qwen3_decision_model)。
        你负责将人类的自然语言指令，拆解为底层控制框架能够执行的 API 指令序列。

        【核心规则】：
        1. 必须先用 `thought` 字段进行思考，严谨规划动作序列。
           - **状态前置检查**：在规划抓取动作前，必须首先检查目标手臂的当前夹爪状态（`gripper_state`），确保其处于张开（open）状态。如果闭合，需先发指令张开。
           - **抓取规范**：抓取动作必须遵循：移至上方预备 -> 下降至目标 -> 闭合夹爪 -> 抬起。
           - **放置规范**：放置动作必须遵循：移至上方预备 -> 下降至目标 -> 张开夹爪 -> 垂直抬起退刀（避免碰倒物体） -> 移回初始点。
        2. 【安全铁律】：你规划的任何机械臂末端位置 Z 轴值绝对不能低于 `desk_height`！
        3. 【运动学约束】：如果物体在左侧（Y > 0），优先使用 `left_arm`；在右侧（Y < 0），优先使用 `right_arm`。
        4. 你的输出必须是一个严格的 JSON 对象。
        5. 【格式铁律】：JSON 必须包含 `thought` 和 `action_plan`。`action_plan` 列表中的每个元素必须且只能包含 `"function"` 和 `"parameters"` 两个键。

        【完美输出示例1 - 单臂移动】：
        {
            "thought": "用户要求左臂向上移动10cm。左臂当前Z为1.24，加0.1变为1.34。符合安全高度。",
            "action_plan": [
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "left_arm", "position": [0.382, 0.33394, 1.3435], "orientation": [0, 0, 0, 1]}
                }
            ]
        }
        
        【完美输出示例2 - 右手抓取方块】：
        {
            "thought": "用户要求右手抓取橙色方块。查状态可知橙色方块位于 [0.43, -0.15, 1.03]，Y<0适合右臂操作。首先检查右臂夹爪状态：当前为 'open'，可直接抓取。规划序列：1. 右臂移动到方块上方10cm处预备；2. 右臂下降至方块实际位置；3. 闭合夹爪；4. 右臂将方块抬起。全程姿态保持为 [0, 0, 0, 1]。",
            "action_plan": [
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.13], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.03], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_gripper_cmd",
                    "parameters": {"arm": "right_arm", "state": "close"}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.23], "orientation": [0, 0, 0, 1]}
                }
            ]
        }

        【完美输出示例3 - 抓取并放置（方块堆叠）】：
        {
            "thought": "用户要求将橙色方块放到蓝色方块上。橙色方块 [0.43, -0.15, 1.03]，Y<0 必须由右臂执行。蓝色方块 [0.38, 0.0, 1.03]。检查右臂夹爪状态：当前为 'open'。规划序列：1. 右臂移至橙色方块上方预备；2. 下降至橙色方块；3. 闭合夹爪抓取；4. 垂直抬起至安全高度(Z=1.25)；5. 平移至蓝色方块上方；6. 下降放置（补偿堆叠厚度设为1.08）；7. 张开夹爪释放；8. 垂直向上退刀至安全高度(Z=1.25)避免碰倒方块；9. 移修复位至初始准备点。全程姿态保持 [0, 0, 0, 1]。",
            "action_plan": [
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.20], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.03], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_gripper_cmd",
                    "parameters": {"arm": "right_arm", "state": "close"}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.43, -0.15, 1.25], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.38, 0.0, 1.25], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.38, 0.0, 1.08], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_gripper_cmd",
                    "parameters": {"arm": "right_arm", "state": "open"}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.38, 0.0, 1.25], "orientation": [0, 0, 0, 1]}
                },
                {
                    "function": "send_posture_cmd",
                    "parameters": {"arm": "right_arm", "position": [0.382, -0.33394, 1.2435], "orientation": [0, 0, 0, 1]}
                }
            ]
        }
        """

        action_library = """
        动作库:
        1. send_posture_cmd(arm: str, position: list, orientation: list)
        2. send_gripper_cmd(arm: str, state: str)  # state: "open" 或 "close"
        3. send_voice_cmd(message: str)
        """

        world_state_str = json.dumps(self.world_state, indent=2)
        history_str_parts = [f"人类: {t['user']}\n机器人: {json.dumps(t['assistant'], ensure_ascii=False)}" for t in self.conversation_history]
        formatted_history = "\n".join(history_str_parts)
        
        full_user_prompt = f"# 当前状态\n{world_state_str}\n# API库\n{action_library}\n# 历史对话\n{formatted_history}\n# 最新指令\n人类: {user_command}\n"

        self.get_logger().info("🧠 [Qwen]: 正在思考和规划接口调用...")
        try:
            completion = self.llm_client.chat.completions.create(
                model="qwen3.5-plus", 
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": full_user_prompt}
                ],
                response_format={"type": "json_object"},
                temperature=0.1,
            )
            return json.loads(completion.choices[0].message.content)
        except Exception as e:
            self.get_logger().error(f"❌ 调用大模型失败: {e}")
            return None

    def execute_action_plan(self, action_plan):
            if not action_plan:
                return

            for i, action in enumerate(action_plan):
                func_name = action.get("function") or action.get("name") or action.get("action")
                params = action.get("parameters") or action.get("arguments") or {}
                
                self.get_logger().info(f"👉 执行步骤 [{i+1}]: {func_name}")

                if func_name == "send_voice_cmd":
                    msg_text = params.get('message', '')
                    self.get_logger().info(f"   📢 [详情] 播报内容: '{msg_text}'") # 新增打印
                    
                    msg = String()
                    msg.data = msg_text
                    self.voice_pub.publish(msg)
                    
                elif func_name == "send_posture_cmd":
                    arm = params.get('arm')
                    pos = params.get('position', [0.382, 0, 1.24])
                    ori = params.get('orientation', [0, 0, 0, 1])

                    # ================= 新增的详细参数打印 =================
                    self.get_logger().info(f"   🎯 [详情] 目标手臂: {arm}")
                    self.get_logger().info(f"   🎯 [详情] 目标位置 (X, Y, Z): {pos}")
                    self.get_logger().info(f"   🎯 [详情] 目标姿态 (四元数): {ori}")
                    # =====================================================

                    # 1. 更新内部目标缓存
                    if arm == "left_arm":
                        self.target_left_pose["position"] = pos
                        self.target_left_pose["orientation"] = ori
                    elif arm == "right_arm":
                        self.target_right_pose["position"] = pos
                        self.target_right_pose["orientation"] = ori

                    # 2. 从缓存中提取双臂坐标
                    msg = DualArmPoseTargets()
                    msg.left_target.position.x = float(self.target_left_pose["position"][0])
                    msg.left_target.position.y = float(self.target_left_pose["position"][1])
                    msg.left_target.position.z = float(self.target_left_pose["position"][2])
                    msg.left_target.orientation.x = float(self.target_left_pose["orientation"][0])
                    msg.left_target.orientation.y = float(self.target_left_pose["orientation"][1])
                    msg.left_target.orientation.z = float(self.target_left_pose["orientation"][2])
                    msg.left_target.orientation.w = float(self.target_left_pose["orientation"][3])

                    msg.right_target.position.x = float(self.target_right_pose["position"][0])
                    msg.right_target.position.y = float(self.target_right_pose["position"][1])
                    msg.right_target.position.z = float(self.target_right_pose["position"][2])
                    msg.right_target.orientation.x = float(self.target_right_pose["orientation"][0])
                    msg.right_target.orientation.y = float(self.target_right_pose["orientation"][1])
                    msg.right_target.orientation.z = float(self.target_right_pose["orientation"][2])
                    msg.right_target.orientation.w = float(self.target_right_pose["orientation"][3])

                    self.posture_pub.publish(msg)
                    
                    # 阻塞主线程 2.5 秒
                    time.sleep(3.5) 
                    
                elif func_name == "send_gripper_cmd":
                    arm = params.get('arm', '')
                    state = params.get('state', '')
                    
                    self.get_logger().info(f"   🤏 [详情] 夹爪控制: {arm} 执行 '{state}' 操作") # 新增打印

                    msg = GripperControl()
                    if arm == "left_arm":
                        msg.gripper_select = GripperControl.GRIPPER_LEFT
                    else:
                        msg.gripper_select = GripperControl.GRIPPER_RIGHT

                    if state == "open":
                        msg.command = GripperControl.COMMAND_OPEN
                    else:
                        msg.command = GripperControl.COMMAND_CLOSE

                    self.gripper_pub.publish(msg)
                    time.sleep(1.0) 
                    
            self.get_logger().info("✅ 所有指令下发执行完毕。")

    def terminal_interaction_loop(self):
        print("\n" + "="*50)
        print("🤖 iRobot 具身智能决策中枢已上线 (适配动态插值层)")
        print("指令范例：'将橙色方块放到蓝色方块上方...'")
        print("="*50 + "\n")
        
        while rclpy.ok():
            try:
                user_input = input("\n👤 [你]: ")
                if user_input.lower() in ['退出', 'exit', 'quit']:
                    rclpy.shutdown()
                    break
                
                if not user_input.strip(): continue

                model_response = self.process_user_command(user_input)
                if model_response:
                    print(f"\n🧠 [大模型思考]: {model_response.get('thought', '')}")
                    self.execute_action_plan(model_response.get("action_plan", []))
                    
                    self.conversation_history.append({"user": user_input, "assistant": model_response})
                    if len(self.conversation_history) > self.MAX_HISTORY_LENGTH:
                        self.conversation_history.pop(0)
            except EOFError: break
            except Exception as e: self.get_logger().error(f"交互循环错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionConnectNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()