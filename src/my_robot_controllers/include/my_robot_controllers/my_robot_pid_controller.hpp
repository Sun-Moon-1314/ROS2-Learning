#ifndef MY_ROBOT_PID_CONTROLLER_HPP
#define MY_ROBOT_PID_CONTROLLER_HPP

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"

using FloatArray = example_interfaces::msg::Float64MultiArray;

namespace my_robot_pid_controller
{

class PidController : public controller_interface::ControllerInterface
{
public:
    PidController();

    // 声明控制器需要的命令接口
    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    // 声明控制器需要的状态接口
    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // 接收目标关节位置
    void command_callback(const FloatArray::SharedPtr msg);

    // 关节名称
    std::vector<std::string> joint_names_;

    // 状态接口和命令接口名称
    std::string state_interface_name_;
    std::string command_interface_name_;

    // P 参数
    double kp_ = 10.0;
    double kd_ = 0.0;

    // 速度命令限幅，单位 rad/s
    double max_velocity_ = 0.2;

    // 在订阅回调和控制循环之间传递目标位置
    realtime_tools::RealtimeBuffer<std::vector<double>> target_buffer_;

    rclcpp::Subscription<FloatArray>::SharedPtr command_subscriber_;
};

}  // namespace my_robot_pid_controller

#endif  // MY_ROBOT_PID_CONTROLLER_HPP