#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/robot_control.hpp"

using RobotControl = my_robot_interfaces::action::RobotControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<RobotControl>;
using namespace std::placeholders;

class RobotControlServerNode : public rclcpp::Node{

public:
    RobotControlServerNode() : Node("robot_control_server"){
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant );
        robot_control_server_ = rclcpp_action::create_server<RobotControl>(
            this,
            "robot_control",
            std::bind(&RobotControlServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotControlServerNode::cancel_callback, this, _1),
            std::bind(&RobotControlServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_ 
        );
        RCLCPP_INFO(get_logger(), "robot control server has been started.");
    }
 
private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const RobotControl::Goal> goal
    ){
        (void)uuid;
        RCLCPP_INFO(get_logger(), "Received a goal.");

        int target_position = goal->target_position;
        int velocity = goal->velocity;
        
        if (target_position < 0 || target_position > 100 || velocity <= 0)
        {
            RCLCPP_INFO(get_logger(), "Rejecting the goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Policy: preempt the goal if a new goal accepted.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (this->goal_handle_) {
                if (goal_handle_->is_active()) {
                    RCLCPP_INFO(get_logger(), 
                        "Abort current goal and receive new goal.");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Accepting the goal.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Receive a cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        execute_callback(goal_handle);
    }

    void execute_callback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        int target_position = goal_handle->get_goal()->target_position;
        int velocity = goal_handle->get_goal()->velocity;

        auto feedback = std::make_shared<RobotControl::Feedback>();
        auto result = std::make_shared<RobotControl::Result>();

        RCLCPP_INFO(get_logger(), "Start executing");
        rclcpp::Rate loop_rate(1.0);
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_)
                {
                    result->real_position = robot_position_;
                    result->message = "Aborted";
                    goal_handle->abort(result);
                    return;
                }
            }

            if (goal_handle->is_canceling()){
                result->message = "Canceled.";
                result->real_position = robot_position_;
                goal_handle->canceled(result);
                return;
            }

            int diff = target_position - robot_position_;

            if (diff == 0)
            {
                result->message = "Success.";
                result->real_position = target_position;
                goal_handle->succeed(result);
                return;
            }
            else if (diff > 0)
            {
                if (diff >= velocity)
                {
                    robot_position_ += velocity;
                }
                else
                {
                    robot_position_ += diff;
                }
            }
            else
            {
                if (std::abs(diff) >= velocity)
                {
                    robot_position_ -= velocity;
                }
                else
                {
                    robot_position_ += diff;
                }
            }
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(get_logger(), "Robot position: %d", robot_position_);
            loop_rate.sleep();
        }
        
    }

    rclcpp_action::Server<RobotControl>::SharedPtr robot_control_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    std::shared_ptr<GoalHandle> goal_handle_;
    int robot_position_ = 50; 
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}