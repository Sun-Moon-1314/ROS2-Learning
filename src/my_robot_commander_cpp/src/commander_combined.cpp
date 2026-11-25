#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/pose_target.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool=example_interfaces::msg::Bool;
using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using PoseTarget = my_robot_interfaces::msg::PoseTarget;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(0.1);
        arm_->setMaxAccelerationScalingFactor(0.1);


        goto_joint_target_sub_ = node_->create_subscription<Float64MultiArray>(
            "joint_target_command",
            10,
            std::bind(&Commander::gotoJointTargetCallback, this, _1)
        );

        mobile_robot_cmd_sub_ = node_->create_subscription<TwistStamped>(
            "mobile_robot_command",
            10,
            std::bind(&Commander::gotoMobileRobotCallback, this, _1)
        );

        mobile_robot_cmd_pub_ = node_->create_publisher<TwistStamped>(
            "/diff_drive_controller/cmd_vel",
            10
        );
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = 
            (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed");
        }
    }

    void gotoJointTargetCallback(const Float64MultiArray &msg)
    {
        const auto &joints = msg.data;

        if (joints.size() == 2) {
            goToJointTarget(joints);
        }

    }

    void gotoMobileRobotCallback(const TwistStamped &msg)
    {
        TwistStamped command = msg;

        command.header.stamp = node_->now();

        mobile_robot_cmd_pub_->publish(command);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;

    rclcpp::Subscription<Float64MultiArray>::SharedPtr goto_joint_target_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr mobile_robot_cmd_sub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr mobile_robot_cmd_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}