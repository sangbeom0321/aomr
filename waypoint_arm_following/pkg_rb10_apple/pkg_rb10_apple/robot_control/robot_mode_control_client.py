import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_rb10_apple.action import RobotModeControl
from std_msgs.msg import Int32


class RobotModeControlClient(Node):
    def __init__(self):
        super().__init__("robot_mode_control_client")
        self.action_client = ActionClient(self, RobotModeControl, "robot_mode_control")
        self.subscription = self.create_subscription(
            Int32, "mode_topic", self.mode_callback, 10
        )
        self.mode = None

    def mode_callback(self, msg):
        self.mode = msg.data
        self.send_goal(self.mode)

    def send_goal(self, mode):
        goal_msg = RobotModeControl.Goal()
        goal_msg.mode = mode
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Received feedback: {0}".format(feedback.feedback))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            "Result: {0}, {1}".format(result.success, result.message)
        )


def main(args=None):
    rclpy.init(args=args)
    robot_mode_control_client = RobotModeControlClient()
    rclpy.spin(robot_mode_control_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
