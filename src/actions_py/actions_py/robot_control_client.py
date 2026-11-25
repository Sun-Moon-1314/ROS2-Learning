#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import RobotControl
from example_interfaces.msg import Empty

class RobotControlClientNode(Node):
    def __init__(self):
        super().__init__("robot_control_client")
        self.goal_handle_ = None

        self.robot_control_client_ = ActionClient(
            self, RobotControl, "robot_control"
        )
        self.cancel_sending_subscriber_ = self.create_subscription(
            Empty, "cancel_sending", self.cancel_callback, 10
        ) 

    def cancel_callback(self, msg):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Send a cancel request.")
            self.goal_handle_.cancel_goal_async()


    def send_goal(self, target_position, velocity):
        self.robot_control_client_.wait_for_server()

        goal = RobotControl.Goal()
        goal.target_position = target_position
        goal.velocity = velocity

        self.get_logger().info("Send a goal.")
        self.robot_control_client_.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback). \
        add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, goal_future):
        self.goal_handle_: ClientGoalHandle = goal_future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accept.")
            self.goal_handle_.get_result_async().\
                add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got refused.")

    def goal_result_callback(self, result_future):
        result_response = result_future.result()

        result: RobotControl.Result = result_response.result
        status = result_response.status

        if status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")

        self.get_logger().info(
            f"Result: {result.real_position}, Message: {result.message}")

    def feedback_callback(self, feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info("Got feedback: " + str(current_position))


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlClientNode()
    node.send_goal(76, 1)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()