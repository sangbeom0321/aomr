import rclpy
from rclpy.node import Node
from interface_rb10_apple.srv import RobotConnectControl
from .cobot import *


class RobotService(Node):
    def __init__(self):
        super().__init__("robot_service_node")
        self.service = self.create_service(
            RobotConnectControl,
            "robot_connect_control",
            self.handle_robot_connect_control,
        )

    def handle_robot_connect_control(self, request, response):
        if request.connect:
            cobot_initialization()
            response.is_power_on = True
        else:
            cobot_deactivate()
            response.is_power_on = False
        return response


def cobot_initialization():
    # 로봇 켜기
    ip = "10.0.99.21"  # 로봇 IP
    ToCB(ip)
    CobotInit()
    SetProgramMode(PG_MODE.REAL)  # 로봇을 real mode로 만듦


def cobot_deactivate():
    # Code to deactivate the robot
    DisConnectToCB()


def main():
    rclpy.init()
    robot_service_node = RobotService()
    try:
        rclpy.spin(robot_service_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_service_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
