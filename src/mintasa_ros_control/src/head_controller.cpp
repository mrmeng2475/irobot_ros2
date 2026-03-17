#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// 假设你的自定义消息编译后生成的头文件名称如下
#include "irobot_interfaces/msg/head_command.hpp" 
#include "actuatorcontroller.h"
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

class MintasaHeadController : public rclcpp::Node
{
public:
    MintasaHeadController()
    : Node("mintasa_head_controller")
    {
        // 声明并获取头部电机的运动速度参数
        this->declare_parameter<double>("head_velocity", 400.0);
        double head_velocity = this->get_parameter("head_velocity").as_double();

        RCLCPP_INFO(this->get_logger(), "头部关节 (ID 21, 22) 规划速度: %.2f", head_velocity);

        // 初始化底层控制器
        pController = ActuatorController::initController();
        Actuator::ErrorsDefine ec;
        pController->lookupActuators(ec);
        if (ec != 0) { 
            RCLCPP_FATAL(this->get_logger(), "查找执行器失败，错误码: 0x%x", ec);
            rclcpp::shutdown();
            return;
        }

        // 检查头部需要的电机 ID 21 和 22 是否在线
        auto found_ids_vec = pController->getActuatorIdArray();
        std::vector<uint8_t> required_ids = {21, 22};
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
            RCLCPP_FATAL(this->get_logger(), "启动失败！头部电机未找全，缺失的ID: %s", missing_str.c_str());
            rclcpp::shutdown();
            return;
        }

        actuator_ids_ = required_ids;
        RCLCPP_INFO(this->get_logger(), "已成功找到头部执行器 (ID 21, 22)。");

        // 使能电机、设置模式和速度
        for (const auto& id : actuator_ids_)
        {
            pController->enableActuator(id);
            pController->activateActuatorMode(id, Actuator::Mode_Profile_Pos);
            pController->setProfilePositionMaxVelocity(id, head_velocity);
        }

        // 订阅你定义的头部控制指令
        head_subscription_ = this->create_subscription<irobot_interfaces::msg::HeadCommand>(
            "/head_cmd", 10, std::bind(&MintasaHeadController::headCommandCallback, this, _1));

        // 发布头部电机的实时状态反馈
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/hardware/head_joint_states", 10);
        
        // 设置 20ms 周期的定时器用于状态反馈 (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            std::bind(&MintasaHeadController::feedbackTimerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "头部控制器初始化完成，已准备好接收 /head_cmd 指令。");
    }
    
    ~MintasaHeadController()
    {
        if (pController && !actuator_ids_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "正在禁用头部执行器...");
            for(const auto& id : actuator_ids_) {
                pController->disableActuator(id);
            }
        }
    }

private:
    // 弧度转电机单位 (根据你的硬件特性可适当调整系数)
    double radians_to_actuator_units(double radians) const {
        return radians * (180.0 / M_PI) / 10.0;
    }

    // 电机单位转弧度
    double actuator_units_to_radians(double actuator_units) const {
        return actuator_units * 10.0 * (M_PI / 180.0);
    }

    // 处理头部指令的回调函数
    void headCommandCallback(const irobot_interfaces::msg::HeadCommand::SharedPtr msg) const
    {
        // 明确声明 ID 为 uint8_t 类型
        const uint8_t head1_id = 21;
        const uint8_t head2_id = 22;

        // 【修改点】：将 msg->head_joint1 取反后再下发
        double target_pos_21 = radians_to_actuator_units(-(msg->head_joint1));
        pController->setPosition(head1_id, target_pos_21);

        // 【修改点】：将 msg->head_joint2 取反后再下发
        double target_pos_22 = radians_to_actuator_units(-(msg->head_joint2));
        pController->setPosition(head2_id, target_pos_22);
    }

    // 定时读取电机状态并发布的反馈函数
    void feedbackTimerCallback()
    {
        auto feedback_msg = sensor_msgs::msg::JointState();
        feedback_msg.header.stamp = this->get_clock()->now();

        // 明确声明 ID 为 uint8_t 类型
        const uint8_t head1_id = 21;
        const uint8_t head2_id = 22;

        // 【修改点】：读取 ID 21 的状态后取反
        double pos_21 = -actuator_units_to_radians(pController->getPosition(head1_id, true));
        feedback_msg.name.push_back("head_joint1");
        feedback_msg.position.push_back(pos_21);

        // 【修改点】：读取 ID 22 的状态后取反
        double pos_22 = -actuator_units_to_radians(pController->getPosition(head2_id, true));
        feedback_msg.name.push_back("head_joint2");
        feedback_msg.position.push_back(pos_22);
        
        publisher_->publish(feedback_msg);
    }

    rclcpp::Subscription<irobot_interfaces::msg::HeadCommand>::SharedPtr head_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    ActuatorController* pController = nullptr;
    std::vector<uint8_t> actuator_ids_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MintasaHeadController>());
    rclcpp::shutdown();
    return 0;
}