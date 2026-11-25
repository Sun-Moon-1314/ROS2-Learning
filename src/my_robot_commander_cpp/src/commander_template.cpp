#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/pose_target.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool=example_interfaces::msg::Bool;
using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using PoseTarget = my_robot_interfaces::msg::PoseTarget;

using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper_command", 
            10,
            std::bind(&Commander::openGripperCallback, this, _1)
        );

        goto_joint_target_sub_ = node_->create_subscription<Float64MultiArray>(
            "joint_target_command",
            10,
            std::bind(&Commander::gotoJointTargetCallback, this, _1)
        );

        goto_pose_target_sub_ = node_->create_subscription<PoseTarget>(
            "pose_target_command",
            10,
            std::bind(&Commander::gotoPoseTargetCallback, this, _1)
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

    void goToPoseTarget(
        double x, 
        double y, 
        double z, 
        double roll, 
        double pitch, 
        double yaw,
        bool cartesian_path=false
    )
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartesian_path) 
        {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            if (fraction > 0.99)
            {
                arm_->execute(trajectory);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Cartesian planning failed.");
            }
        }

    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
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

    void openGripperCallback(const Bool &msg)
    {
        if (msg.data) {
            openGripper();
        }
        else{
            closeGripper();
        }
    }

    void gotoJointTargetCallback(const Float64MultiArray &msg)
    {
        const auto &joints = msg.data;

        if (joints.size() == 6) {
            goToJointTarget(joints);
        }

    }

    void gotoPoseTargetCallback(const PoseTarget &msg)
    {
        goToPoseTarget(
            msg.x,
            msg.y,
            msg.z,
            msg.roll,
            msg.pitch,
            msg.yaw, 
            msg.cartesian_path
        );
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr goto_joint_target_sub_;
    rclcpp::Subscription<PoseTarget>::SharedPtr goto_pose_target_sub_;

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