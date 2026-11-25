#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/robot_control.hpp"
#include "example_interfaces/msg/empty.hpp"

using RobotControl = my_robot_interfaces::action::RobotControl;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<RobotControl>;
using namespace std::placeholders;

class RobotControlClientNode : public rclcpp::Node{

public:
    RobotControlClientNode() : Node("node_name"){
        robot_control_client_ = 
            rclcpp_action::create_client<RobotControl>(this, "robot_control");
        cancel_sending_subscriber_ = 
            this->create_subscription<example_interfaces::msg::Empty>(
                "cancel_sending", 
                10, 
                std::bind(&RobotControlClientNode::cancel_callback, this, _1));
    }

    void send_goal(int target_position, int velocity)
    {
        robot_control_client_->wait_for_action_server();

        auto goal = RobotControl::Goal();
        goal.target_position = target_position;
        goal.velocity = velocity;

        auto options = rclcpp_action::Client<RobotControl>::SendGoalOptions();
        options.result_callback = 
            std::bind(&RobotControlClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = 
            std::bind(&RobotControlClientNode::goal_response_callback, this, _1);
        options.feedback_callback = 
            std::bind(&RobotControlClientNode::goal_feedback_callback, this, _1, _2);
        
        RCLCPP_INFO(get_logger(), "Sending a goal.");
        robot_control_client_->async_send_goal(goal, options);
    }

private:

    void cancel_callback(const example_interfaces::msg::Empty &msg){
        (void)msg;
        this->cancel_goal();
    }

    void cancel_goal()
    {
        if (!goal_handle_)
        {
            RCLCPP_WARN(
                get_logger(),
                "There is no active goal to cancel."
            );
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Sending a cancel request."
        );

        robot_control_client_->async_cancel_goal(goal_handle_);
    }

    void goal_response_callback(const ClientGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle){
            RCLCPP_INFO(get_logger(), "goal is rejected.");
        }
        else { 
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(get_logger(), "goal is received.");
        }
    }

    void goal_result_callback(const ClientGoalHandle::WrappedResult &result)
    {
        auto status = result.code;

        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Succeed.");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(get_logger(), "Aborted.");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(get_logger(), "Canceled.");
        }

        int real_position = result.result->real_position;
        RCLCPP_INFO(get_logger(), "Result: %d", real_position);
    }

    void goal_feedback_callback(
        const ClientGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const RobotControl::Feedback> feedback
    )
    {
        (void)goal_handle;
        int current_position = feedback->current_position;
        RCLCPP_INFO(get_logger(), "Got feedback: %d", current_position);
    }

    rclcpp_action::Client<RobotControl>::SharedPtr robot_control_client_;
    ClientGoalHandle::SharedPtr goal_handle_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr cancel_sending_subscriber_;
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlClientNode>();
    node->send_goal(70, 1);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}