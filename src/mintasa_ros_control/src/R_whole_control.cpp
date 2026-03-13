// 最终合并版本：同时控制7个臂部关节和1个夹爪关节（ID 1-8）
// 订阅 /joint_cmd 用于统一位置控制
// 订阅 /clip_cmd 用于独立的夹爪模式（位置/电流）切换控制
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

class MintasaUnifiedController : public rclcpp::Node
{
public:
    MintasaUnifiedController()
    : Node("mintasa_unified_controller")
    {
        inverted_joint_ids_ = {1, 4, 5};

        this->declare_parameter<double>("arm_velocity", 800.0);
        this->declare_parameter<double>("gripper_velocity", 200.0);
        
        double arm_velocity = this->get_parameter("arm_velocity").as_double();
        double gripper_velocity = this->get_parameter("gripper_velocity").as_double();

        RCLCPP_INFO(this->get_logger(), "臂部关节 (ID 1-7) 规划速度: %.2f", arm_velocity);
        RCLCPP_INFO(this->get_logger(), "夹爪关节 (ID 8) 规划速度: %.2f", gripper_velocity);

        pController = ActuatorController::initController();
        Actuator::ErrorsDefine ec;
        pController->lookupActuators(ec);
        if (ec != 0) { 
            RCLCPP_FATAL(this->get_logger(), "查找执行器失败，错误码: 0x%x", ec);
            rclcpp::shutdown();
            return;
        }

        auto found_ids_vec = pController->getActuatorIdArray();
        std::vector<uint8_t> required_ids = {1, 2, 3, 4, 5, 6, 7, 8};
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
        RCLCPP_INFO(this->get_logger(), "已成功找到所有期望的执行器 (ID 1-8)。");

        for (const auto& id : actuator_ids_)
        {
            pController->enableActuator(id);
            pController->activateActuatorMode(id, Actuator::Mode_Profile_Pos);
            if (id == 8) {
                pController->setProfilePositionMaxVelocity(id, gripper_velocity);
            } else {
                pController->setProfilePositionMaxVelocity(id, arm_velocity);
            }
        }
        RCLCPP_INFO(this->get_logger(), "所有执行器均已使能、设置模式并设定各自的规划速度。");

        for(const auto& id : actuator_ids_) {
            joint_name_to_id_map_["joint" + std::to_string(id)] = id;
        }

        joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_cmd", 10, std::bind(&MintasaUnifiedController::jointCommandCallback, this, _1));
        
        clip_subscription_ = this->create_subscription<irobot_interfaces::msg::ClipCommand>(
            "/clip_cmd", 10, std::bind(&MintasaUnifiedController::clipCommandCallback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/joint_states", 10);
        
        // --- 修正点：将 create_timer 修改为 create_wall_timer ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&MintasaUnifiedController::feedbackTimerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "统一控制器初始化完成，已准备好接收 /joint_cmd 和 /clip_cmd 指令。");
    }
    
    ~MintasaUnifiedController()
    {
        if (pController && !actuator_ids_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "正在禁用所有执行器...");
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

    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& joint_name = msg->name[i];
            double position_cmd = msg->position[i];
            
            auto it = joint_name_to_id_map_.find(joint_name);
            if (it != joint_name_to_id_map_.end())
            {
                uint8_t actuator_id = it->second;
                double target_actuator_pos;

                if (actuator_id == 8) {
                    target_actuator_pos = position_cmd;
                } else {
                    if (inverted_joint_ids_.count(actuator_id)) {
                        position_cmd *= -1.0;
                    }
                    target_actuator_pos = radians_to_actuator_units(position_cmd);
                }
                
                pController->setPosition(actuator_id, target_actuator_pos);
            }
        }
    }

    void clipCommandCallback(const irobot_interfaces::msg::ClipCommand::SharedPtr msg)
    {
        const uint8_t CLIP_ACTUATOR_ID = 8;

        switch (msg->mode)
        {
            case 1:
                RCLCPP_INFO(this->get_logger(), "接收到夹爪[位置]指令: %.2f", msg->value);
                pController->activateActuatorMode(CLIP_ACTUATOR_ID, Actuator::Mode_Profile_Pos);
                pController->setPosition(CLIP_ACTUATOR_ID, msg->value);
                break;
            
            case 2:
                RCLCPP_INFO(this->get_logger(), "接收到夹爪[电流]指令: %.2f", msg->value);
                pController->activateActuatorMode(CLIP_ACTUATOR_ID, Actuator::Mode_Cur);
                pController->setCurrent(CLIP_ACTUATOR_ID, msg->value);
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "接收到未知的夹爪控制模式: %d", msg->mode);
                break;
        }
    }

    void feedbackTimerCallback()
    {
        auto feedback_msg = sensor_msgs::msg::JointState();
        feedback_msg.header.stamp = this->get_clock()->now();

        for (const auto& id : actuator_ids_)
        {
            double current_actuator_pos = pController->getPosition(id, true);
            double final_position;

            if (id == 8)
            {
                final_position = current_actuator_pos;
            }
            else
            {
                final_position = actuator_units_to_radians(current_actuator_pos);
                if (inverted_joint_ids_.count(id)) {
                    final_position *= -1.0;
                }
            }

            feedback_msg.name.push_back("joint" + std::to_string(id));
            feedback_msg.position.push_back(final_position);
        }
        
        publisher_->publish(feedback_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
    rclcpp::Subscription<irobot_interfaces::msg::ClipCommand>::SharedPtr clip_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    ActuatorController* pController = nullptr;
    std::vector<uint8_t> actuator_ids_;
    std::map<std::string, uint8_t> joint_name_to_id_map_;
    std::set<uint8_t> inverted_joint_ids_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MintasaUnifiedController>());
    rclcpp::shutdown();
    return 0;
}