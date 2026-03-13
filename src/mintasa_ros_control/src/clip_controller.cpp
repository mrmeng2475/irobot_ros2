
// 单独的电机控制
#include "rclcpp/rclcpp.hpp"
#include "actuatorcontroller.h"
#include "irobot_interfaces/msg/clip_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>
#include <algorithm> // 为了 std::find

// 定义夹爪的电机ID
const uint8_t CLIP_ACTUATOR_ID = 8;

class ClipController : public rclcpp::Node
{
public:
    ClipController()
    : Node("clip_controller")
    {
        // 1. 初始化控制器
        RCLCPP_INFO(this->get_logger(), "正在初始化 Actuator Controller...");
        pController = ActuatorController::initController();
        Actuator::ErrorsDefine ec;
        
        // 2. 查找并检查夹爪电机
        RCLCPP_INFO(this->get_logger(), "正在查找执行器...");
        pController->lookupActuators(ec);
        if (ec != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "查找执行器失败，错误码: 0x%x", ec);
            rclcpp::shutdown();
            return;
        }

        auto found_ids_vec = pController->getActuatorIdArray();
        if (std::find(found_ids_vec.begin(), found_ids_vec.end(), CLIP_ACTUATOR_ID) == found_ids_vec.end()) {
            RCLCPP_FATAL(this->get_logger(), "启动失败！未找到夹爪执行器，ID: %d", CLIP_ACTUATOR_ID);
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "已成功找到夹爪执行器 (ID: %d)。", CLIP_ACTUATOR_ID);

        // 3. 使能夹爪电机
        RCLCPP_INFO(this->get_logger(), "正在使能执行器 ID: %d...", CLIP_ACTUATOR_ID);
        if (!pController->enableActuator(CLIP_ACTUATOR_ID))
        {
            RCLCPP_FATAL(this->get_logger(), "使能执行器 %d 失败！程序退出。", CLIP_ACTUATOR_ID);
            rclcpp::shutdown();
            return;
        }

        // 默认设置为规划位置模式
        pController->activateActuatorMode(CLIP_ACTUATOR_ID, Actuator::Mode_Profile_Pos);
        RCLCPP_INFO(this->get_logger(), "执行器已使能并默认设置为规划位置模式。");

        // 设置位置模式下的初始（最大）速度
        RCLCPP_INFO(this->get_logger(), "正在设置位置模式下的最大速度为 200...");
        pController->setProfilePositionMaxVelocity(CLIP_ACTUATOR_ID, 200.0);

        // 4. 创建订阅者和发布者
        subscription_ = this->create_subscription<irobot_interfaces::msg::ClipCommand>(
            "/clip_cmd", 10, std::bind(&ClipController::commandCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/clip_states", 10);
        
        // 5. 创建定时器用于发布反馈
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&ClipController::feedbackTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "夹爪控制器初始化完成，已准备好接收指令。");
    }

    ~ClipController()
    {
        if (pController)
        {
            RCLCPP_INFO(this->get_logger(), "正在禁用夹爪执行器...");
            pController->disableActuator(CLIP_ACTUATOR_ID);
        }
    }

private:
    void commandCallback(const irobot_interfaces::msg::ClipCommand::SharedPtr msg)
    {
        switch (msg->mode)
        {
            case 1: // 位置模式
                RCLCPP_INFO(this->get_logger(), "接收到位置指令: %.2f", msg->value);
                // 确保处于位置模式
                pController->activateActuatorMode(CLIP_ACTUATOR_ID, Actuator::Mode_Profile_Pos);
                // 设置目标位置
                pController->setPosition(CLIP_ACTUATOR_ID, msg->value);
                break;
            
            case 2: // 电流模式
                RCLCPP_INFO(this->get_logger(), "接收到电流指令: %.2f", msg->value);
                // 切换到电流模式 (已修正)
                pController->activateActuatorMode(CLIP_ACTUATOR_ID, Actuator::Mode_Cur);
                // 设置目标电流
                pController->setCurrent(CLIP_ACTUATOR_ID, msg->value);
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "接收到未知的控制模式: %d", msg->mode);
                break;
        }
    }

    void feedbackTimerCallback()
    {
        auto feedback_msg = sensor_msgs::msg::JointState();
        feedback_msg.header.stamp = this->get_clock()->now();

        // 获取当前模式并发布相应的数据 (已修正)
        Actuator::ActuatorMode current_mode = pController->getActuatorMode(CLIP_ACTUATOR_ID);

        feedback_msg.name.push_back("clip_joint");
        
        if (current_mode == Actuator::Mode_Profile_Pos) {
            double current_position = pController->getPosition(CLIP_ACTUATOR_ID, true);
            feedback_msg.position.push_back(current_position);
        } else if (current_mode == Actuator::Mode_Cur) { // (已修正)
            double current_current = pController->getCurrent(CLIP_ACTUATOR_ID, true);
            feedback_msg.effort.push_back(current_current); // 使用effort字段来发布电流
        }
        
        publisher_->publish(feedback_msg);
    }

    // ROS2 成员
    rclcpp::Subscription<irobot_interfaces::msg::ClipCommand>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // SDK 成员
    ActuatorController* pController = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClipController>());
    rclcpp::shutdown();
    return 0;
}