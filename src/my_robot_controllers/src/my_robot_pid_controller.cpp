#include "my_robot_controllers/my_robot_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_pid_controller
{

PidController::PidController()
    : controller_interface::ControllerInterface()
{}


// 初始化参数
controller_interface::CallbackReturn PidController::on_init()
{
    try
    {
        auto_declare<std::vector<std::string>>("joints", {});

        auto_declare<std::string>(
            "state_interface", "position");

        auto_declare<std::string>(
            "command_interface", "velocity");

        auto_declare<double>("kp", 10.0);
        auto_declare<double>("kp", 0.5);

        auto_declare<double>("max_velocity", 0.2);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to declare parameters: %s",
            e.what());

        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}


// 申请命令接口
controller_interface::InterfaceConfiguration
PidController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;

    config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.reserve(joint_names_.size());

    for (const auto & joint_name : joint_names_)
    {
        config.names.push_back(
            joint_name + "/" + command_interface_name_);
    }

    return config;
}


// 申请状态接口
controller_interface::InterfaceConfiguration
PidController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;

    config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.reserve(joint_names_.size() * 2);

    for (const auto & joint_name : joint_names_)
    {
        config.names.push_back(joint_name + "/position");
        config.names.push_back(joint_name + "/velocity");
    }

    return config;
}


// 配置控制器
controller_interface::CallbackReturn PidController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    const auto node = get_node();

    joint_names_ =
        node->get_parameter("joints").as_string_array();

    state_interface_name_ =
        node->get_parameter("state_interface").as_string();

    command_interface_name_ =
        node->get_parameter("command_interface").as_string();

    kp_ = node->get_parameter("kp").as_double();
    kd_ = node->get_parameter("kd").as_double();

    max_velocity_ =
        node->get_parameter("max_velocity").as_double();

    if (joint_names_.empty())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "The joints parameter must not be empty.");

        return controller_interface::CallbackReturn::ERROR;
    }

    // 当前控制公式只实现了“位置误差 → 速度命令”
    if (state_interface_name_ != "position" ||
        command_interface_name_ != "velocity")
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "This controller requires position state and velocity command.");

        return controller_interface::CallbackReturn::ERROR;
    }

    if (!std::isfinite(kp_) || kp_ < 0.0 ||
    !std::isfinite(kd_) || kd_ < 0.0 ||
    !std::isfinite(max_velocity_) || max_velocity_ <= 0.0)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "kp and kd must be non-negative; max_velocity must be positive.");

        return controller_interface::CallbackReturn::ERROR;
    }

    command_subscriber_ = node->create_subscription<FloatArray>(
        "/joints_command",
        10,
        std::bind(
            &PidController::command_callback,
            this,
            std::placeholders::_1)
    );

    RCLCPP_INFO(
        node->get_logger(),
        "P controller configured: %zu joints, kp=%.3f, max_velocity=%.3f",
        joint_names_.size(),
        kp_,
        max_velocity_);

    return controller_interface::CallbackReturn::SUCCESS;
}


// 收到上层发送的目标位置
void PidController::command_callback(
    const FloatArray::SharedPtr msg)
{
    if (msg->data.size() != joint_names_.size())
    {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Expected %zu joint targets, received %zu.",
            joint_names_.size(),
            msg->data.size());

        return;
    }

    for (const double target : msg->data)
    {
        if (!std::isfinite(target))
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Joint target must be a finite number.");

            return;
        }
    }

    // 将目标写入缓冲区，供 update() 读取
    target_buffer_.writeFromNonRT(msg->data);
}


// 激活控制器
controller_interface::CallbackReturn PidController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (state_interfaces_.size() != joint_names_.size() * 2 ||
        command_interfaces_.size() != joint_names_.size())
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "The number of interfaces does not match the joints.");

        return controller_interface::CallbackReturn::ERROR;
    }

    std::vector<double> initial_targets;
    initial_targets.reserve(joint_names_.size());

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        // 检查收到的接口顺序，避免读错位置和速度
        if (state_interfaces_[2 * i].get_name() !=
                joint_names_[i] + "/position" ||
            state_interfaces_[2 * i + 1].get_name() !=
                joint_names_[i] + "/velocity" ||
            command_interfaces_[i].get_name() !=
                joint_names_[i] + "/velocity")
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Interface order does not match joint %s.",
                joint_names_[i].c_str());

            return controller_interface::CallbackReturn::ERROR;
        }

        const double current_position =
            state_interfaces_[2 * i].get_value();

        const double current_velocity =
            state_interfaces_[2 * i + 1].get_value();

        if (!std::isfinite(current_position) ||
            !std::isfinite(current_velocity))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Initial joint state is invalid.");

            return controller_interface::CallbackReturn::ERROR;
        }

        initial_targets.push_back(current_position);
        command_interfaces_[i].set_value(0.0);
    }

    target_buffer_.writeFromNonRT(initial_targets);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "PD controller activated.");

    return controller_interface::CallbackReturn::SUCCESS;
}

// 停用控制器
controller_interface::CallbackReturn PidController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (auto & command_interface : command_interfaces_)
    {
        command_interface.set_value(0.0);
    }

    RCLCPP_INFO(
        get_node()->get_logger(),
        "P controller deactivated.");

    return controller_interface::CallbackReturn::SUCCESS;
}


// 周期性计算控制命令
controller_interface::return_type PidController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    const auto * targets = target_buffer_.readFromRT();

    if (targets == nullptr ||
        targets->size() != joint_names_.size())
    {
        for (auto & command_interface : command_interfaces_)
        {
            command_interface.set_value(0.0);
        }

        return controller_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        // 1. 读取当前位置
        const double current_position = state_interfaces_[2 * i].get_value();

        const double current_velocity = state_interfaces_[2 * i + 1].get_value();

        // 2. 获取目标位置
        const double target_position = (*targets)[i];

        if (!std::isfinite(current_position) ||
            !std::isfinite(current_velocity) ||
            !std::isfinite(target_position))
        {
            for (auto & command_interface : command_interfaces_)
            {
                command_interface.set_value(0.0);
            }

            return controller_interface::return_type::ERROR;
        }

        // 3. 计算位置误差
        const double error =
            target_position - current_position;

        // 4. P 控制：根据位置误差计算速度
        double velocity_command = kp_ * error - kd_ * current_velocity;

        // 5. 限制输出速度
        velocity_command = std::clamp(
            velocity_command,
            -max_velocity_,
            max_velocity_);

        // 6. 写入 velocity 命令接口
        command_interfaces_[i].set_value(velocity_command);
    }

    return controller_interface::return_type::OK;
}

}  // namespace my_robot_pid_controller


// 将控制器导出为插件
PLUGINLIB_EXPORT_CLASS(
    my_robot_pid_controller::PidController,
    controller_interface::ControllerInterface
)