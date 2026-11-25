#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

using FollowJointTrajectory =
    control_msgs::action::FollowJointTrajectory;

using GoalHandle =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

using FloatArray =
    example_interfaces::msg::Float64MultiArray;

using JointState =
    sensor_msgs::msg::JointState;


class PidTrajectoryAdapter : public rclcpp::Node
{
public:
    PidTrajectoryAdapter()
        : Node("pid_trajectory_adapter")
    {
        joints_command_pub_ = create_publisher<FloatArray>(
            "/joints_command", 10);

        joint_state_sub_ = create_subscription<JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(
                &PidTrajectoryAdapter::joint_state_callback,
                this,
                std::placeholders::_1));

        action_server_ =
            rclcpp_action::create_server<FollowJointTrajectory>(
                this,
                "/arm_controller/follow_joint_trajectory",
                std::bind(
                    &PidTrajectoryAdapter::goal_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
                std::bind(
                    &PidTrajectoryAdapter::cancel_callback,
                    this,
                    std::placeholders::_1),
                std::bind(
                    &PidTrajectoryAdapter::accepted_callback,
                    this,
                    std::placeholders::_1));

        // 每 50 ms 更新一次轨迹参考位置
        timer_ = create_wall_timer(
            50ms,
            std::bind(&PidTrajectoryAdapter::update, this));

        RCLCPP_INFO(get_logger(), "PID trajectory adapter started.");
    }

private:
    static constexpr std::size_t JOINT_COUNT = 2;

    const std::array<std::string, JOINT_COUNT> joint_names_ = {
        "arm_base_forearm_joint",
        "forearm_hand_joint"
    };

    // 示例中的默认位置误差限制，单位 rad
    static constexpr double START_TOLERANCE = 0.05;
    static constexpr double PATH_TOLERANCE = 0.30;
    static constexpr double GOAL_TOLERANCE = 0.03;
    static constexpr double DEFAULT_GOAL_TIME = 3.0;
    static constexpr double STATE_TIMEOUT = 1.0;

    rclcpp::Publisher<FloatArray>::SharedPtr joints_command_pub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<double, JOINT_COUNT> current_positions_{0.0, 0.0};
    bool joint_states_ready_ = false;
    std::chrono::steady_clock::time_point last_joint_state_time_;

    std::shared_ptr<GoalHandle> active_goal_;
    bool goal_reserved_ = false;

    trajectory_msgs::msg::JointTrajectory trajectory_;

    // 控制器中的关节顺序 → MoveIt 轨迹中的下标
    std::array<std::size_t, JOINT_COUNT> joint_indices_{0, 1};

    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    double goal_time_allowance_ = DEFAULT_GOAL_TIME;


    static double duration_seconds(
        const builtin_interfaces::msg::Duration & duration)
    {
        return static_cast<double>(duration.sec) +
               static_cast<double>(duration.nanosec) / 1e9;
    }


    bool joint_states_fresh() const
    {
        if (!joint_states_ready_) {
            return false;
        }

        const auto age =
            std::chrono::steady_clock::now() - last_joint_state_time_;

        return std::chrono::duration<double>(age).count() <= STATE_TIMEOUT;
    }


    void joint_state_callback(const JointState::SharedPtr msg)
    {
        std::array<double, JOINT_COUNT> positions;

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const auto it = std::find(
                msg->name.begin(),
                msg->name.end(),
                joint_names_[i]);

            if (it == msg->name.end()) {
                return;
            }

            const auto index = static_cast<std::size_t>(
                std::distance(msg->name.begin(), it));

            if (index >= msg->position.size() ||
                !std::isfinite(msg->position[index]))
            {
                return;
            }

            positions[i] = msg->position[index];
        }

        current_positions_ = positions;
        last_joint_state_time_ = std::chrono::steady_clock::now();

        if (!joint_states_ready_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Received arm states: %.3f, %.3f",
                current_positions_[0],
                current_positions_[1]);
        }

        joint_states_ready_ = true;
    }


    bool get_joint_indices(
        const trajectory_msgs::msg::JointTrajectory & trajectory,
        std::array<std::size_t, JOINT_COUNT> & indices) const
    {
        if (trajectory.joint_names.size() != JOINT_COUNT) {
            return false;
        }

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const auto it = std::find(
                trajectory.joint_names.begin(),
                trajectory.joint_names.end(),
                joint_names_[i]);

            if (it == trajectory.joint_names.end()) {
                return false;
            }

            indices[i] = static_cast<std::size_t>(
                std::distance(trajectory.joint_names.begin(), it));
        }

        return indices[0] != indices[1];
    }


    bool validate_trajectory(
        const trajectory_msgs::msg::JointTrajectory & trajectory) const
    {
        if (trajectory.points.empty()) {
            return false;
        }

        double previous_time = -1.0;

        for (const auto & point : trajectory.points)
        {
            if (point.positions.size() != JOINT_COUNT ||
                point.time_from_start.sec < 0 ||
                point.time_from_start.nanosec >= 1000000000U)
            {
                return false;
            }

            const double point_time =
                duration_seconds(point.time_from_start);

            if (!std::isfinite(point_time) ||
                point_time <= previous_time)
            {
                return false;
            }

            for (const double position : point.positions)
            {
                if (!std::isfinite(position)) {
                    return false;
                }
            }

            previous_time = point_time;
        }

        return true;
    }


    // 根据时间计算参考位置，结果顺序与 MoveIt 轨迹一致
    std::vector<double> interpolate(double elapsed) const
    {
        const auto & points = trajectory_.points;

        if (elapsed <= duration_seconds(points.front().time_from_start)) {
            return points.front().positions;
        }

        for (std::size_t i = 1; i < points.size(); ++i)
        {
            const double t0 =
                duration_seconds(points[i - 1].time_from_start);

            const double t1 =
                duration_seconds(points[i].time_from_start);

            if (elapsed <= t1)
            {
                const double ratio = (elapsed - t0) / (t1 - t0);

                std::vector<double> reference(JOINT_COUNT);

                for (std::size_t j = 0; j < JOINT_COUNT; ++j)
                {
                    const double p0 = points[i - 1].positions[j];
                    const double p1 = points[i].positions[j];

                    reference[j] = p0 + ratio * (p1 - p0);
                }

                return reference;
            }
        }

        return points.back().positions;
    }


    void publish_positions(
        const std::array<double, JOINT_COUNT> & positions)
    {
        FloatArray msg;
        msg.data.assign(positions.begin(), positions.end());

        joints_command_pub_->publish(msg);
    }


    void hold_current_position()
    {
        // 仅在反馈新鲜时发送当前位置作为保持目标
        if (joint_states_fresh()) {
            publish_positions(current_positions_);
        }
    }


    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;

        if (goal_reserved_)
        {
            RCLCPP_WARN(get_logger(), "A trajectory is already running.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!joint_states_fresh())
        {
            RCLCPP_ERROR(get_logger(), "Joint states are missing or stale.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 此示例不处理多自由度关节和自定义误差容限
        if (!goal->multi_dof_trajectory.points.empty() ||
            !goal->path_tolerance.empty() ||
            !goal->goal_tolerance.empty() ||
            !goal->component_path_tolerance.empty() ||
            !goal->component_goal_tolerance.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "This example does not support custom trajectory tolerances "
                "or multi-DOF trajectories.");

            return rclcpp_action::GoalResponse::REJECT;
        }

        std::array<std::size_t, JOINT_COUNT> indices;

        if (!get_joint_indices(goal->trajectory, indices) ||
            !validate_trajectory(goal->trajectory))
        {
            RCLCPP_ERROR(get_logger(), "Invalid trajectory or joint names.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 为简化时间处理，本示例要求轨迹头时间戳为 0
        const auto & stamp = goal->trajectory.header.stamp;

        if (stamp.sec != 0 || stamp.nanosec != 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "This example requires a zero trajectory header timestamp.");

            return rclcpp_action::GoalResponse::REJECT;
        }

        const auto & first_point = goal->trajectory.points.front();

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const double planned = first_point.positions[indices[i]];
            const double actual = current_positions_[i];

            if (std::abs(planned - actual) > START_TOLERANCE)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Start mismatch for %s: planned %.3f, actual %.3f",
                    joint_names_[i].c_str(),
                    planned,
                    actual);

                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (goal->goal_time_tolerance.sec < 0 ||
            goal->goal_time_tolerance.nanosec >= 1000000000U)
        {
            RCLCPP_ERROR(get_logger(), "Invalid goal time tolerance.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        goal_reserved_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Accepted trajectory with %zu points.",
            goal->trajectory.points.size());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        if (!active_goal_ || goal_handle != active_goal_) {
            return rclcpp_action::CancelResponse::REJECT;
        }

        RCLCPP_INFO(get_logger(), "Cancellation requested.");

        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void accepted_callback(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        active_goal_ = goal_handle;
        trajectory_ = goal_handle->get_goal()->trajectory;

        if (!get_joint_indices(trajectory_, joint_indices_))
        {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
            result->error_string = "Joint mapping failed.";

            active_goal_->abort(result);
            active_goal_.reset();
            goal_reserved_ = false;
            return;
        }

        const double requested_time = duration_seconds(
            goal_handle->get_goal()->goal_time_tolerance);

        goal_time_allowance_ =
            requested_time > 0.0 ? requested_time : DEFAULT_GOAL_TIME;

        start_time_ = now();

        RCLCPP_INFO(get_logger(), "Executing trajectory.");
    }


    void finish(
        int32_t error_code,
        const std::string & message,
        bool canceled = false)
    {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = error_code;
        result->error_string = message;

        if (canceled)
        {
            hold_current_position();
            active_goal_->canceled(result);
        }
        else if (error_code == FollowJointTrajectory::Result::SUCCESSFUL)
        {
            active_goal_->succeed(result);
        }
        else
        {
            hold_current_position();
            active_goal_->abort(result);
        }

        RCLCPP_INFO(get_logger(), "Trajectory finished: %s", message.c_str());

        active_goal_.reset();
        goal_reserved_ = false;
    }


    void update()
    {
        if (!active_goal_) {
            return;
        }

        if (active_goal_->is_canceling())
        {
            finish(
                FollowJointTrajectory::Result::SUCCESSFUL,
                "Canceled.",
                true);

            return;
        }

        if (!joint_states_fresh())
        {
            finish(
                FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                "Joint states are missing or stale.");

            return;
        }

        const double elapsed = (now() - start_time_).seconds();

        if (elapsed < 0.0)
        {
            finish(
                FollowJointTrajectory::Result::INVALID_GOAL,
                "Clock moved backwards.");

            return;
        }

        const double end_time = duration_seconds(
            trajectory_.points.back().time_from_start);

        const std::vector<double> reference = interpolate(elapsed);

        std::array<double, JOINT_COUNT> command;
        std::array<double, JOINT_COUNT> errors;

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            command[i] = reference[joint_indices_[i]];
            errors[i] = command[i] - current_positions_[i];

            if (std::abs(errors[i]) > PATH_TOLERANCE)
            {
                finish(
                    FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                    "Tracking error exceeded the path tolerance.");

                return;
            }
        }

        // 按 P 控制器要求的关节顺序发布参考位置
        publish_positions(command);

        auto feedback =
            std::make_shared<FollowJointTrajectory::Feedback>();

        feedback->header.stamp = now();
        feedback->joint_names = trajectory_.joint_names;

        feedback->desired.positions.resize(JOINT_COUNT);
        feedback->actual.positions.resize(JOINT_COUNT);
        feedback->error.positions.resize(JOINT_COUNT);

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const auto trajectory_index = joint_indices_[i];

            feedback->desired.positions[trajectory_index] = command[i];
            feedback->actual.positions[trajectory_index] = current_positions_[i];
            feedback->error.positions[trajectory_index] = errors[i];
        }

        active_goal_->publish_feedback(feedback);

        if (elapsed < end_time) {
            return;
        }

        bool goal_reached = true;

        for (const double error : errors)
        {
            if (std::abs(error) > GOAL_TOLERANCE) {
                goal_reached = false;
            }
        }

        if (goal_reached)
        {
            finish(
                FollowJointTrajectory::Result::SUCCESSFUL,
                "Goal reached.");

            return;
        }

        if (elapsed > end_time + goal_time_allowance_)
        {
            finish(
                FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
                "Goal position was not reached in time.");
        }
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PidTrajectoryAdapter>();

    // 这里使用单线程执行器，回调按顺序执行
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}