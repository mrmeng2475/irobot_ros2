// 双臂合并版本：集中管理底层连接，分离上层 ROS 话题
// 右臂: ID 1-8   话题: /r_joint_cmd, /r_clip_cmd, /hardware/r_joint_states
// 左臂: ID 11-18 话题: /l_joint_cmd, /l_clip_cmd, /hardware/l_joint_states

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "irobot_interfaces/msg/clip_command.hpp"
#include "actuatorcontroller.h"
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

class MintasaDualArmController : public rclcpp::Node
{
public:
    MintasaDualArmController()
    : Node("mintasa_dual_arm_controller")
    {
        // ================= 1. 配置反向关节 =================
        r_inverted_joint_ids_ = {1, 3, 5, 6};         // 右臂反向关节
        l_inverted_joint_ids_ = {11, 13, 14, 15, 16}; // 左臂反向关节

        // ================= 2. 获取参数 =================
        this->declare_parameter<double>("arm_velocity", 800.0);
        this->declare_parameter<double>("gripper_velocity", 200.0);

        double arm_velocity = this->get_parameter("arm_velocity").as_double();
        double gripper_velocity = this->get_parameter("gripper_velocity").as_double();

        RCLCPP_INFO(this->get_logger(), "机械臂关节规划速度: %.2f", arm_velocity);
        RCLCPP_INFO(this->get_logger(), "夹爪关节规划速度: %.2f", gripper_velocity);

        // ================= 3. 底层统一初始化 (仅调用一次) =================
        pController = ActuatorController::initController();
        Actuator::ErrorsDefine ec;
        pController->lookupActuators(ec);
        if (ec != 0) { 
            RCLCPP_FATAL(this->get_logger(), "查找执行器失败，错误码: 0x%x", ec);
            rclcpp::shutdown();
            return;
        }

        // 检查所有 16 个电机是否全部在线 (右臂1-8, 左臂11-18)
        auto found_ids_vec = pController->getActuatorIdArray();
        std::vector<uint8_t> required_ids = {1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18};
        std::vector<uint8_t> missing_ids;

        for (uint8_t req_id : required_ids) {
            if (std::find(found_ids_vec.begin(), found_ids_vec.end(), req_id) == found_ids_vec.end()) {
                missing_ids.push_back(req_id);
            }
        }

        if (!missing_ids.empty()) {
            std::string missing_str;
            for(auto id : missing_ids) {
                missing_str += std::to_string(id) + " ";
            }
            RCLCPP_FATAL(this->get_logger(), "启动失败！部分执行器未找到，缺失的ID: %s", missing_str.c_str());
            rclcpp::shutdown();
            return;
        }

        actuator_ids_ = required_ids;
        RCLCPP_INFO(this->get_logger(), "✅ 已成功找到双臂全部执行器 (ID 1-8, 11-18)。");

        // 使能并设置速度
        for (const auto& id : actuator_ids_)
        {
            pController->enableActuator(id);
            pController->activateActuatorMode(id, Actuator::Mode_Profile_Pos);
            
            // 分配不同的速度规划
            if (id == 8 || id == 18) {
                pController->setProfilePositionMaxVelocity(id, gripper_velocity);
            } else {
                pController->setProfilePositionMaxVelocity(id, arm_velocity);
            }
            
            // 分配映射字典
            if (id <= 8) {
                r_joint_name_to_id_map_["joint" + std::to_string(id)] = id;
            } else if (id >= 11 && id <= 18) {
                l_joint_name_to_id_map_["joint" + std::to_string(id)] = id;
            }
        }

        // ================= 4. ROS 话题配置 (双臂解耦) =================
        
        // --- 右臂订阅与发布 ---
        r_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/r_joint_cmd", 10, std::bind(&MintasaDualArmController::rJointCommandCallback, this, _1));
        r_clip_sub_ = this->create_subscription<irobot_interfaces::msg::ClipCommand>(
            "/r_clip_cmd", 10, std::bind(&MintasaDualArmController::rClipCommandCallback, this, _1));
        r_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/r_joint_states", 10);

        // --- 左臂订阅与发布 ---
        l_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/l_joint_cmd", 10, std::bind(&MintasaDualArmController::lJointCommandCallback, this, _1));
        l_clip_sub_ = this->create_subscription<irobot_interfaces::msg::ClipCommand>(
            "/l_clip_cmd", 10, std::bind(&MintasaDualArmController::lClipCommandCallback, this, _1));
        l_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/l_joint_states", 10);
        
        // ================= 5. 统一状态反馈定时器 =================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&MintasaDualArmController::feedbackTimerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 双臂统一控制器启动完毕，监听各自的指令流！");
    }
    
    ~MintasaDualArmController()
    {
        if (pController && !actuator_ids_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "正在安全禁用双臂所有执行器...");
            for(const auto& id : actuator_ids_) {
                pController->disableActuator(id);
            }
        }
    }

private:
    double radians_to_actuator_units(double radians) const {
        return radians * (180.0 / M_PI) / 10.0;
    }

    double actuator_units_to_radians(double actuator_units) const {
        return actuator_units * 10.0 * (M_PI / 180.0);
    }

    // ----------------- 右臂指令回调 -----------------
    void rJointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& joint_name = msg->name[i];
            double position_cmd = msg->position[i];
            
            auto it = r_joint_name_to_id_map_.find(joint_name);
            if (it != r_joint_name_to_id_map_.end())
            {
                uint8_t actuator_id = it->second;
                double target_actuator_pos;

                if (actuator_id == 8) { // 右侧夹爪
                    target_actuator_pos = position_cmd;
                } else {
                    if (r_inverted_joint_ids_.count(actuator_id)) {
                        position_cmd *= -1.0;
                    }
                    target_actuator_pos = radians_to_actuator_units(position_cmd);
                }
                pController->setPosition(actuator_id, target_actuator_pos);
            }
        }
    }

    void rClipCommandCallback(const irobot_interfaces::msg::ClipCommand::SharedPtr msg)
    {
        const uint8_t CLIP_ACTUATOR_ID = 8;
        processClipCommand(CLIP_ACTUATOR_ID, msg, "右侧");
    }

    // ----------------- 左臂指令回调 -----------------
    void lJointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& joint_name = msg->name[i];
            double position_cmd = msg->position[i];
            
            auto it = l_joint_name_to_id_map_.find(joint_name);
            if (it != l_joint_name_to_id_map_.end())
            {
                uint8_t actuator_id = it->second;
                double target_actuator_pos;

                if (actuator_id == 18) { // 左侧夹爪
                    target_actuator_pos = position_cmd;
                } else {
                    if (l_inverted_joint_ids_.count(actuator_id)) {
                        position_cmd *= -1.0;
                    }
                    target_actuator_pos = radians_to_actuator_units(position_cmd);
                }
                pController->setPosition(actuator_id, target_actuator_pos);
            }
        }
    }

    void lClipCommandCallback(const irobot_interfaces::msg::ClipCommand::SharedPtr msg)
    {
        const uint8_t CLIP_ACTUATOR_ID = 18;
        processClipCommand(CLIP_ACTUATOR_ID, msg, "左侧");
    }

    // ----------------- 通用夹爪处理逻辑 -----------------
    void processClipCommand(uint8_t id, const irobot_interfaces::msg::ClipCommand::SharedPtr msg, const std::string& side)
    {
        switch (msg->mode)
        {
            case 1:
                pController->activateActuatorMode(id, Actuator::Mode_Profile_Pos);
                pController->setPosition(id, msg->value);
                break;
            case 2:
                pController->activateActuatorMode(id, Actuator::Mode_Cur);
                pController->setCurrent(id, msg->value);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "接收到未知的%s夹爪控制模式: %d", side.c_str(), msg->mode);
                break;
        }
    }

    // ----------------- 统一状态反馈回调 -----------------
    void feedbackTimerCallback()
    {
        auto r_feedback_msg = sensor_msgs::msg::JointState();
        auto l_feedback_msg = sensor_msgs::msg::JointState();
        
        auto now = this->get_clock()->now();
        
        r_feedback_msg.header.stamp = now;
        l_feedback_msg.header.stamp = now;

        for (const auto& id : actuator_ids_)
        {
            double current_actuator_pos = pController->getPosition(id, true);
            double final_position;

            // 分流处理与发布
            if (id == 8 || id == 18) {
                // 夹爪无需转换单位
                final_position = current_actuator_pos;
                if (id == 8) {
                    r_feedback_msg.name.push_back("joint" + std::to_string(id));
                    r_feedback_msg.position.push_back(final_position);
                } else {
                    l_feedback_msg.name.push_back("joint" + std::to_string(id));
                    l_feedback_msg.position.push_back(final_position);
                }
            } 
            else {
                // 机械臂处理 (转换单位并考虑正负号)
                final_position = actuator_units_to_radians(current_actuator_pos);
                if ((id <= 7 && r_inverted_joint_ids_.count(id)) || 
                    (id >= 11 && id <= 17 && l_inverted_joint_ids_.count(id))) {
                    final_position *= -1.0;
                }
                
                if (id <= 7) {
                    r_feedback_msg.name.push_back("joint" + std::to_string(id));
                    r_feedback_msg.position.push_back(final_position);
                } else if (id >= 11 && id <= 17) {
                    l_feedback_msg.name.push_back("joint" + std::to_string(id));
                    l_feedback_msg.position.push_back(final_position);
                }
            }
        }
        
        r_state_pub_->publish(r_feedback_msg);
        l_state_pub_->publish(l_feedback_msg);
    }

    // === 类成员变量 ===
    ActuatorController* pController = nullptr;
    std::vector<uint8_t> actuator_ids_;
    
    // 右臂资源
    std::set<uint8_t> r_inverted_joint_ids_;
    std::map<std::string, uint8_t> r_joint_name_to_id_map_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr r_joint_sub_;
    rclcpp::Subscription<irobot_interfaces::msg::ClipCommand>::SharedPtr r_clip_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_state_pub_;

    // 左臂资源
    std::set<uint8_t> l_inverted_joint_ids_;
    std::map<std::string, uint8_t> l_joint_name_to_id_map_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr l_joint_sub_;
    rclcpp::Subscription<irobot_interfaces::msg::ClipCommand>::SharedPtr l_clip_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_state_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MintasaDualArmController>());
    rclcpp::shutdown();
    return 0;
}