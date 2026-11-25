// arm_hardware_interface.cpp
#include "my_robot_hardware/arm_hardware_interface.hpp"

#include <cmath>
#include <cstddef>
#include <exception>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    using hardware_interface::CallbackReturn;

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    try
    {
        const auto & params = info_.hardware_parameters;
        port_ = params.at("dynamixel_port");

        const int arm_base_motor_id = std::stoi(params.at("arm_base_motor_id"));
        const int forearm_motor_id = std::stoi(params.at("forearm_motor_id"));
        const std::size_t joint_count = info_.joints.size();

        if (port_.empty() || joint_count != 2)
        {
            throw std::runtime_error("A serial port and two arm joints are required.");
        }

        joints_ids_.resize(joint_count);
        directions_.assign(joint_count, 1.0);
        hw_positions_.assign(joint_count, 0.0);
        hw_commands_.assign(joint_count, 0.0);

        for (std::size_t i = 0; i < joint_count; ++i)
        {
            const auto & joint = info_.joints[i];

            if (joint.name == "arm_base_forearm_joint")
            {
                joints_ids_[i] = arm_base_motor_id;
            }
            else if (joint.name == "forearm_hand_joint")
            {
                joints_ids_[i] = forearm_motor_id;
            }
            else
            {
                throw std::runtime_error("Unexpected arm joint: " + joint.name);
            }

            // Xacro 未填写 direction 时，默认使用 +1。
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

        if (joints_ids_[0] == joints_ids_[1])
        {
            throw std::runtime_error("The two arm joints must use different motor IDs.");
        }

        driver_ = std::make_shared<XL330Driver>(port_);
    }
    catch (const std::exception & error)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("ArmHardwareInterface"),
            "Hardware initialization failed: %s", error.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]);
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]);
    }

    return interfaces;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /* previous_state */)
{
    const auto logger = rclcpp::get_logger("ArmHardwareInterface");

    for (const auto & joint : info_.joints)
    {
        for (const auto & interface : joint.command_interfaces)
        {
            RCLCPP_INFO(logger, "COMMAND INTERFACE NAME: %s/%s",
                joint.name.c_str(), interface.name.c_str());
        }

        for (const auto & interface : joint.state_interfaces)
        {
            RCLCPP_INFO(logger, "STATE INTERFACE NAME: %s/%s",
                joint.name.c_str(), interface.name.c_str());
        }
    }

    if (driver_->init() != 0)
    {
        RCLCPP_ERROR(logger, "Failed to initialize driver.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    // 现有驱动在 activateWithPositionMode() 中切换模式并开启力矩。
    for (const int motor_id : joints_ids_)
    {
        driver_->deactivate(motor_id);
        driver_->activateWithPositionMode(motor_id);
    }

    // 读取切换模式后的关节角度，用当前位置初始化软件目标。
    if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) !=
        hardware_interface::return_type::OK)
    {
        on_deactivate(previous_state);
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < joints_ids_.size(); ++i)
    {
        hw_commands_[i] = hw_positions_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */)
{
    for (const int motor_id : joints_ids_)
    {
        driver_->deactivate(motor_id);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time & /* time */,
    const rclcpp::Duration & /* period */)
{
    for (std::size_t i = 0; i < joints_ids_.size(); ++i)
    {
        const double motor_position = driver_->getPositionRadian(joints_ids_[i]);

        if (!std::isfinite(motor_position))
        {
            return hardware_interface::return_type::ERROR;
        }

        hw_positions_[i] = directions_[i] * motor_position;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time & /* time */,
    const rclcpp::Duration & /* period */)
{
    for (const double command : hw_commands_)
    {
        if (!std::isfinite(command))
        {
            return hardware_interface::return_type::ERROR;
        }
    }

    for (std::size_t i = 0; i < joints_ids_.size(); ++i)
    {
        driver_->setTargetPositionRadian(
            joints_ids_[i],
            directions_[i] * hw_commands_[i]);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware

// Arm 的 Xacro 目前指向底盘插件。
// 将 <plugin> 改成插件描述 XML 为下面这个机械臂类注册的名称。
PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardwareInterface,
    hardware_interface::SystemInterface)