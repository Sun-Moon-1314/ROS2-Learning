#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/xl330_driver.hpp"

namespace mobile_base_hardware
{

class MobileBaseHardwareInterface
    : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    std::shared_ptr<XL330Driver> driver_;
    std::string port_;

    // 同一个下标对应同一个关节。
    std::vector<int> motor_ids_;
    std::vector<double> directions_;

    // 状态：rad、rad/s。
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    // 目标速度：rad/s。
    std::vector<double> hw_commands_;
};

}  // namespace mobile_base_hardware

#endif  // MOBILE_BASE_HARDWARE_INTERFACE_HPP