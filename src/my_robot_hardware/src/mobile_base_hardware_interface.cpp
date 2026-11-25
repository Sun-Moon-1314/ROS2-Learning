#include "my_robot_hardware/mobile_base_hardware_interface.hpp"

#include <cmath>
#include <exception>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware
{

hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    using hardware_interface::CallbackReturn;

    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    try
    {
        const auto & params = info_.hardware_parameters;

        port_ = params.at("dynamixel_port");
        const int left_motor_id = std::stoi(params.at("left_motor_id"));
        const int right_motor_id = std::stoi(params.at("right_motor_id"));

        const auto joint_count = info_.joints.size();
        if (port_.empty() || joint_count != 2)
        {
            throw std::runtime_error("A serial port and two wheel joints are required.");
        }

        motor_ids_.resize(joint_count);
        directions_.assign(joint_count, 1.0);
        hw_positions_.assign(joint_count, 0.0);
        hw_velocities_.assign(joint_count, 0.0);
        hw_commands_.assign(joint_count, 0.0);

        for (std::size_t i = 0; i < joint_count; ++i)
        {
            const auto & joint = info_.joints[i];

            if (joint.name == "base_left_wheel_joint")
            {
                motor_ids_[i] = left_motor_id;
            }
            else if (joint.name == "base_right_wheel_joint")
            {
                motor_ids_[i] = right_motor_id;
            }
            else
            {
                throw std::runtime_error("Unexpected wheel joint: " + joint.name);
            }

            const auto direction = joint.parameters.find("direction");
            if (direction != joint.parameters.end())
            {
                directions_[i] = std::stod(direction->second);
            }

            if (directions_[i] != 1.0 && directions_[i] != -1.0)
            {
                throw std::runtime_error(joint.name + ": direction must be 1 or -1.");
            }
        }

        driver_ = std::make_shared<XL330Driver>(port_);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MobileBaseHardwareInterface"),
            "Hardware initialization failed: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

// 导出状态接口，绑定每个关节对应的状态变量。
std::vector<hardware_interface::StateInterface>
MobileBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]);

        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_velocities_[i]);
    }

    return interfaces;
}


// 导出速度命令接口，控制器通过它更新 hw_commands_。
std::vector<hardware_interface::CommandInterface>
MobileBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_commands_[i]);
    }

    return interfaces;
}


// 打开串口并设置波特率。
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    const auto logger =
        rclcpp::get_logger("MobileBaseHardwareInterface");

    // 打印 URDF 中声明的命令接口和状态接口
    for (const auto & joint : info_.joints)
    {
        for (const auto & interface : joint.command_interfaces)
        {
            RCLCPP_INFO(
                logger,
                "COMMAND INTERFACE NAME: %s/%s",
                joint.name.c_str(),
                interface.name.c_str());
        }

        for (const auto & interface : joint.state_interfaces)
        {
            RCLCPP_INFO(
                logger,
                "STATE INTERFACE NAME: %s/%s",
                joint.name.c_str(),
                interface.name.c_str());
        }
    }

    // 初始化电机驱动
    if (driver_->init() != 0)
    {
        RCLCPP_ERROR(logger, "Failed to initialize driver.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger, "Driver initialized successfully.");

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 激活所有电机，使用速度控制模式。
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        hw_commands_[i] = 0.0;

        // 修改电机工作模式前，先关闭力矩。
        driver_->deactivate(motor_ids_[i]);
        driver_->activateWithVelocityMode(motor_ids_[i]);

        driver_->setTargetVelocityRadianPerSec(
            motor_ids_[i], 0.0);
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MobileBaseHardwareInterface"),
        "Hardware activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 发送零速度，然后停用所有电机。
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        hw_commands_[i] = 0.0;

        driver_->setTargetVelocityRadianPerSec(
            motor_ids_[i], 0.0);

        driver_->deactivate(motor_ids_[i]);
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MobileBaseHardwareInterface"),
        "Hardware deactivated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 电机反馈 → ROS 关节状态。
hardware_interface::return_type
MobileBaseHardwareInterface::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;

    const auto logger =
        rclcpp::get_logger("MobileBaseHardwareInterface");

    const double dt = period.seconds();

    if (!std::isfinite(dt) || dt < 0.0)
    {
        RCLCPP_ERROR(logger, "Invalid read period: %f", dt);
        return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        double motor_velocity =
            driver_->getVelocityRadianPerSec(motor_ids_[i]);

        // if (abs(motor_velocity) < 0.03) { motor_velocity = 0.0; }

        if (!std::isfinite(motor_velocity))
        {
            RCLCPP_ERROR(
                logger,
                "Invalid velocity from motor %d: %f",
                motor_ids_[i],
                motor_velocity);

            return hardware_interface::return_type::ERROR;
        }

        hw_velocities_[i] = directions_[i] * motor_velocity;

        // 从初始值 0 积分，得到相对位置估计。
        hw_positions_[i] += hw_velocities_[i] * dt;

        // RCLCPP_INFO(
        //     logger,
        //     "Motor ID: %d, velocity: %.4f rad/s, position: %.4f rad",
        //     motor_ids_[i],
        //     hw_velocities_[i],
        //     hw_positions_[i]);
    }

    return hardware_interface::return_type::OK;
}

// ROS 关节命令 → 电机命令。
hardware_interface::return_type
MobileBaseHardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 先检查全部命令，避免非有限值被转换成电机数值。
    for (const double command : hw_commands_)
    {
        if (!std::isfinite(command))
        {
            return hardware_interface::return_type::ERROR;
        }
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        driver_->setTargetVelocityRadianPerSec(
            motor_ids_[i],
            directions_[i] * hw_commands_[i]);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace mobile_base_hardware

PLUGINLIB_EXPORT_CLASS(
    mobile_base_hardware::MobileBaseHardwareInterface,
    hardware_interface::SystemInterface
)