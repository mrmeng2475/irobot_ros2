// 这个是最初的关节测试节点，未经角度变化，直接给关节发送实际的位置指令
// 单独的七个关节角控制
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // 使用 JointState 消息类型
#include "actuatorcontroller.h"
#include <vector>
#include <memory>
#include <shared_mutex>
#include <numeric> // for std::iota
#include <algorithm> // for std::find
#include <map>

using std::placeholders::_1;

class MintasaMultiJointController : public rclcpp::Node
{
public:
    MintasaMultiJointController()
    : Node("mintasa_multi_joint_controller")
    {
        // 1. 初始化控制器
        RCLCPP_INFO(this->get_logger(), "正在初始化 Actuator Controller...");
        pController = ActuatorController::initController();
        Actuator::ErrorsDefine ec;
        
        // 2. 检查期望的执行器是否都已连接
        RCLCPP_INFO(this->get_logger(), "正在查找所有已连接的执行器...");
        pController->lookupActuators(ec);
        if (ec != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "查找执行器失败，错误码: 0x%x", ec);
            rclcpp::shutdown();
            return;
        }

        auto found_ids_vec = pController->getActuatorIdArray();
        
        // 检查 ID 1 到 8 是否全部存在
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

        // 所有期望的执行器都已找到，存入成员变量
        actuator_ids_ = required_ids;
        RCLCPP_INFO(this->get_logger(), "已成功找到所有期望的执行器 (ID 1-8)。");

        // 3. 批量使能并设置模式
        for (const auto& id : actuator_ids_)
        {
            RCLCPP_INFO(this->get_logger(), "正在使能执行器 ID: %d...", id);
            if (!pController->enableActuator(id))
            {
                RCLCPP_FATAL(this->get_logger(), "使能执行器 %d 失败！程序退出。", id);
                rclcpp::shutdown();
                return;
            }
            pController->activateActuatorMode(id, Actuator::Mode_Profile_Pos);
        }
        RCLCPP_INFO(this->get_logger(), "所有执行器均已使能并设置为规划位置模式。");

        // 创建关节名到ID的映射
        for(const auto& id : actuator_ids_) {
            joint_name_to_id_map_["joint" + std::to_string(id)] = id;
        }

        // 4. 创建订阅者和发布者
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_cmd", 10, std::bind(&MintasaMultiJointController::commandCallback, this, _1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 5. 创建定时器用于发布反馈
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&MintasaMultiJointController::feedbackTimerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "控制器初始化完成，已准备好接收指令。");
    }

    ~MintasaMultiJointController()
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
    void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        // 遍历接收到的指令
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& joint_name = msg->name[i];
            const double target_position = msg->position[i];
            
            // 查找关节名对应的ID
            auto it = joint_name_to_id_map_.find(joint_name);
            if (it != joint_name_to_id_map_.end())
            {
                uint8_t actuator_id = it->second;
                RCLCPP_INFO(this->get_logger(), "接收到指令 -> 设置关节 %s (ID: %d) 的位置为: %.2f", 
                            joint_name.c_str(), actuator_id, target_position);
                pController->setPosition(actuator_id, target_position);
            }
        }
    }

    void feedbackTimerCallback()
    {
        auto feedback_msg = sensor_msgs::msg::JointState();
        feedback_msg.header.stamp = this->get_clock()->now();

        // 遍历所有执行器，获取位置并填充消息
        for (const auto& id : actuator_ids_)
        {
            double current_position = pController->getPosition(id, true);
            feedback_msg.name.push_back("joint" + std::to_string(id));
            feedback_msg.position.push_back(current_position);
        }
        
        publisher_->publish(feedback_msg);
    }

    // ROS2 成员
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // SDK 成员
    ActuatorController* pController = nullptr;
    std::vector<uint8_t> actuator_ids_;
    std::map<std::string, uint8_t> joint_name_to_id_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MintasaMultiJointController>());
    rclcpp::shutdown();
    return 0;
}