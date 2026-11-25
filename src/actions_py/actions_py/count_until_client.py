#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil



class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(
            self,
            CountUntil,
            "count_until"
        )

    def send_goal(self, target_number, period):
        # Wait for the server
        self.count_until_client_.wait_for_server()

        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending goal")
        goal_future = self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)

    # def cancel_goal(self):
    #     self.get_logger().info("Send a cancel request.")
    #     self.goal_handle_.cancel_goal_async()
    #     self.timer_.cancel()

    def goal_response_callback(self, goal_future):
        self.goal_handle_: ClientGoalHandle = goal_future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accept.")
            result_future = self.goal_handle_.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal Get rejected.")

    def goal_result_callback(self, result_future):
        result_response = result_future.result()
        result: CountUntil.Result = result_response.result
        status = result_response.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(
            f"Result: {result.reached_number}"
        )

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Got feedback: " + str(number))


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(5, 1.0) 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
