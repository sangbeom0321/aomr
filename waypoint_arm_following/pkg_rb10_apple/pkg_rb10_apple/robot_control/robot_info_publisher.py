import rclpy
from rclpy.node import Node
from interface_rb10_apple.msg import RobotJointPosition, RobotTcpPose
from .cobot import *
from std_msgs.msg import String

from rclpy.executors  import MultiThreadedExecutor

class RobotInfoPublisher(Node):
    def __init__(self):
        super().__init__("robot_info_publisher")

        # 기존 퍼블리셔
        self.joint_publisher = self.create_publisher(
            RobotJointPosition, "robot_joint_position", 10
        )
        self.pose_publisher = self.create_publisher(RobotTcpPose, "robot_tcp_pose", 10)

        # 새로 추가된 String 퍼블리셔
        self.status_publisher = self.create_publisher(String, "cobot_status", 10)

        # 멤버 변수 초기화
        self.previous_cobot_status = None

        # 기존 타이머(1.0초 주기) : 로봇 정보 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_robot_info)

        # 새로 추가된 타이머(1.0초 주기) : COBOT 상태 관찰
        self.status_timer = self.create_timer(0.1, self.observe_cobot_status)

        self.cobot_initialization()

    def cobot_initialization(self):
        ip = "10.0.99.21"  # 로봇 IP
        ToCB(ip)
        CobotInit()
        SendCOMMAND("pgmode real", CMD_TYPE.NONMOVE)  # 로봇을 real mode로 만듦

    def publish_robot_info(self):
        robot_info = GetCurreJP()  # (joint_data, pose_data) 튜플이라 가정
        joint_data, pose_data = robot_info

        # robot_joint_position 메시지 생성 및 할당
        joint_position_msg = RobotJointPosition()
        joint_position_msg.j0 = joint_data.j0
        joint_position_msg.j1 = joint_data.j1
        joint_position_msg.j2 = joint_data.j2
        joint_position_msg.j3 = joint_data.j3
        joint_position_msg.j4 = joint_data.j4
        joint_position_msg.j5 = joint_data.j5

        # robot_tcp_pose 메시지 생성 및 할당
        tcp_pose_msg = RobotTcpPose()
        tcp_pose_msg.x = pose_data.x
        tcp_pose_msg.y = pose_data.y
        tcp_pose_msg.z = pose_data.z
        tcp_pose_msg.rx = pose_data.rx
        tcp_pose_msg.ry = pose_data.ry
        tcp_pose_msg.rz = pose_data.rz

        # 토픽 발행
        self.joint_publisher.publish(joint_position_msg)
        self.pose_publisher.publish(tcp_pose_msg)

    def observe_cobot_status(self):
        # 현재 COBOT 상태를 enum 형태로 받아온다고 가정
        status_enum = GetCurrentCobotStatus()

        # enum -> 문자열로 매핑
        if status_enum == COBOT_STATUS.PAUSED:
            status_str = "PAUSED"
        elif status_enum == COBOT_STATUS.IDLE:
            status_str = "IDLE"
        elif status_enum == COBOT_STATUS.RUNNING:
            status_str = "RUNNING"
        else:
            status_str = "UNKNOWN"

        # String 메시지 생성 및 할당 후 발행
        status_msg = String()
        status_msg.data = status_str
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_info_publisher = RobotInfoPublisher()

    #rclpy.spin(robot_info_publisher)

    executor = MultiThreadedExecutor()
    executor.add_node(robot_info_publisher)
    executor.spin()

    robot_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
