import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from interface_rb10_apple.action import RobotTcpControl
from interface_rb10_apple.msg import RobotTcpPose
from std_msgs.msg import String
from .cobot import *  # 로봇 스크립트 함수(ManualScript 등)
from enum import Enum
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
import logging


# COBOT 상태를 나타내는 Enum
class COBOT_STATUS(Enum):
    UNKNOWN = 0
    PAUSED = 1
    IDLE = 2
    RUNNING = 3


# 내부 상태 머신을 위한 Enum
class APP_STATE(Enum):
    WAITING_FOR_IDLE = 1  # 로봇이 IDLE이 되기를 대기
    EXECUTING_SCRIPT = 2  # ManualScript 실행 중
    WAITING_FOR_RUNNING = 3  # 스크립트 실행 후 RUNNING 상태 대기
    WAITING_FOR_IDLE_AFTER_RUNNING = 4  # RUNNING 후 다시 IDLE 대기
    DONE = 5  # 액션 완료 상태


class RobotTcpControlServer(Node):
    def __init__(self):
        super().__init__("robot_tcp_control_server")

        # Set log level to DEBUG
        self.get_logger().set_level(logging.WARN)

        # 액션 서버 생성
        self._action_server = ActionServer(
            self, RobotTcpControl, "robot_tcp_control", self.execute_callback
        )

        # pose 토픽 구독
        self.subscription = self.create_subscription(
            RobotTcpPose, "robot_tcp_pose", self.pose_callback, 10
        )

        # cobot_status 토픽 구독
        self.cobot_status_subscriber = self.create_subscription(
            String, "cobot_status", self.cobot_status_callback, 10
        )

        self.current_pose = RobotTcpPose()

        # 상태 머신 관련 변수 초기화
        self.app_state = APP_STATE.DONE
        self.current_robot_status = COBOT_STATUS.UNKNOWN
        self.action_goal_handle = None
        self.action_result = None
        self.action_in_progress = False

        # 액션 완료를 신호하기 위한 Event 객체
        self.action_complete = threading.Event()

        # COBOT 초기화
        self.cobot_initialization()

    def cobot_initialization(self):
        ip = "10.0.99.21"  # 로봇 IP
        ToCB(ip)
        CobotInit()
        SendCOMMAND("pgmode real", CMD_TYPE.NONMOVE)  # 로봇을 real mode로 설정
        self.get_logger().info("COBOT 초기화 완료 및 real mode로 전환됨.")

    def pose_callback(self, msg: RobotTcpPose):
        self.current_pose = msg
        self.get_logger().debug(f"현재 포즈 업데이트: {self.current_pose}")

    def execute_callback(self, goal_handle):
        self.get_logger().info("액션 목표 수신: 실행 중...")
        # 목표 위치 계산
        new_x = self.current_pose.x + goal_handle.request.x
        new_y = self.current_pose.y + goal_handle.request.y
        new_z = self.current_pose.z + goal_handle.request.z
        new_rx = self.current_pose.rx + goal_handle.request.rx
        new_ry = self.current_pose.ry + goal_handle.request.ry
        new_rz = self.current_pose.rz + goal_handle.request.rz

        # 목표 위치를 저장
        self.target_pose = RobotTcpPose(
            x=new_x, y=new_y, z=new_z, rx=new_rx, ry=new_ry, rz=new_rz
        )

        # 액션 상태 초기화
        self.action_goal_handle = goal_handle
        self.action_result = RobotTcpControl.Result()
        self.action_in_progress = True
        self.app_state = APP_STATE.WAITING_FOR_IDLE
        self.publish_feedback("로봇이 IDLE 상태가 되기를 기다리는 중...")

        # Event 클리어 및 대기
        self.action_complete.clear()

        self.get_logger().info(
            "execute_callback: 상태 머신 시작, WAITING_FOR_IDLE 상태로 전환됨."
        )

        # 액션 완료까지 대기
        self.action_complete.wait()

        return self.action_result

    def _run_state_machine(self):
        while self.app_state != APP_STATE.DONE and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        # 상태 머신이 완료되면 스레드 종료

    def cobot_status_callback(self, msg: String):
        status_str = msg.data
        previous_status = self.current_robot_status
        # COBOT 상태 업데이트
        if status_str == "PAUSED":
            self.current_robot_status = COBOT_STATUS.PAUSED
        elif status_str == "IDLE":
            self.current_robot_status = COBOT_STATUS.IDLE
        elif status_str == "RUNNING":
            self.current_robot_status = COBOT_STATUS.RUNNING
        else:
            self.current_robot_status = COBOT_STATUS.UNKNOWN

        self.get_logger().info(
            f"COBOT 상태 변경: {previous_status.name} -> {self.current_robot_status.name}"
        )

        # 상태 머신이 동작 중일 때만 처리
        if not self.action_in_progress:
            return

        # 상태 머신 로직
        if self.app_state == APP_STATE.WAITING_FOR_IDLE:
            if self.current_robot_status == COBOT_STATUS.IDLE:
                # IDLE 상태가 되면 ManualScript 실행
                self.get_logger().info("로봇이 IDLE 상태입니다. 스크립트 실행 중...")
                spd = 0.5  # 속도
                acc = 0.1  # 가속도
                script_tcp = f"movetcp {spd}, {acc}, {self.target_pose.x}, {self.target_pose.y}, {self.target_pose.z}, {self.target_pose.rx}, {self.target_pose.ry}, {self.target_pose.rz}"
                ManualScript(script_tcp)
                self.app_state = APP_STATE.WAITING_FOR_RUNNING
                self.publish_feedback("스크립트 실행 중...")

        elif self.app_state == APP_STATE.WAITING_FOR_RUNNING:
            if self.current_robot_status == COBOT_STATUS.RUNNING:
                self.get_logger().info(
                    "스크립트 실행으로 인해 로봇이 RUNNING 상태입니다."
                )
                self.app_state = APP_STATE.WAITING_FOR_IDLE_AFTER_RUNNING
                self.publish_feedback("스크립트 실행 완료, 로봇이 RUNNING 상태임.")

        elif self.app_state == APP_STATE.WAITING_FOR_IDLE_AFTER_RUNNING:
            if self.current_robot_status == COBOT_STATUS.IDLE:
                # 스크립트 실행 후 다시 IDLE 상태가 되면 액션 성공 처리
                self.get_logger().info(
                    "스크립트 실행 완료 및 로봇이 다시 IDLE 상태입니다."
                )
                self.action_result.success = True
                self.action_result.message = "스크립트 실행 완료"
                self.action_goal_handle.succeed()
                self.action_in_progress = False
                self.app_state = APP_STATE.DONE
                self.publish_feedback(
                    "액션 성공: 스크립트 실행 완료 및 IDLE 상태 복귀."
                )
                self.action_complete.set()

    def publish_feedback(self, msg: str):
        if self.action_goal_handle:
            feedback = RobotTcpControl.Feedback()
            feedback.feedback = msg
            self.action_goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"피드백 전송: {msg}")


def main(args=None):
    rclpy.init(args=args)
    robot_tcp_control_server = RobotTcpControlServer()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_tcp_control_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_tcp_control_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
