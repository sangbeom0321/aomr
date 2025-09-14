import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from interface_rb10_apple.action import RobotModeControl
from std_msgs.msg import String
from .cobot import *  # 로봇 스크립트 함수(ManualScript 등)
from enum import Enum
import threading
from rclpy.executors import MultiThreadedExecutor
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


class RobotModeControlServer(Node):
    def __init__(self):
        super().__init__("robot_mode_control_server")

        # Set log level to DEBUG
        self.get_logger().set_level(logging.WARN)
        # 액션 서버 생성
        self.action_server = ActionServer(
            self, RobotModeControl, "robot_mode_control", self.execute_callback
        )

        # cobot_status 토픽 구독
        self.cobot_status_subscriber = self.create_subscription(
            String, "cobot_status", self.cobot_status_callback, 10
        )
        self.joint_speed = 30.0
        self.joint_accel = 20.0
        # 로봇 모드별 스크립트 정의
        self.script_poses = {
            0: f"move_j(jnt[180.0, -70.0, 160.0, -90.0, 180.0, 0.0], {self.joint_speed}, {self.joint_accel})",  # 이동자세 좌
            1: f"move_j(jnt[180.0, -70.0, 160.0, -90.0,  90.0, 0.0], {self.joint_speed}, {self.joint_accel})",  # 이동자세 정면
            2: f"move_j(jnt[180.0, -70.0, 160.0, -90.0,   0.0, 0.0], {self.joint_speed}, {self.joint_accel})",  # 이동자세 우
            3: f"move_j(jnt[89.97, -8.31, -125.52, -226.17,-89.89, 0.0], {self.joint_speed}, {self.joint_accel})",  # 판지자세 좌
            4: f"move_j(jnt[89.98, 12.02, 121.01, -133.02, 90.09, 0.0],{self.joint_speed}, {self.joint_accel})",  # 판지자세 우
        }

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

    def execute_callback(self, goal_handle):
        self.get_logger().info("액션 목표 수신: 실행 중...")
        mode = goal_handle.request.mode
        script_pose = self.script_poses.get(mode, None)

        if script_pose is None:
            self.get_logger().warn(f"잘못된 모드 요청: {mode}")
            result = RobotModeControl.Result()
            result.success = False
            result.message = "Invalid mode"
            goal_handle.abort()
            return result

        # 액션 상태 초기화
        self.action_goal_handle = goal_handle
        self.action_result = RobotModeControl.Result()
        self.action_in_progress = True
        self.app_state = APP_STATE.WAITING_FOR_IDLE
        self.publish_feedback("로봇이 IDLE 상태가 되기를 기다리는 중...")

        # ManualScript 실행할 스크립트 저장
        self.selected_script = script_pose

        # Event 클리어 및 대기
        self.action_complete.clear()

        self.get_logger().info(
            "execute_callback: 상태 머신 시작, WAITING_FOR_IDLE 상태로 전환됨."
        )

        # 비동기적으로 상태 머신을 관리하기 위해 별도의 스레드를 사용
        self.action_complete.wait()
        return self.action_result  # 현재는 아직 완료되지 않았으므로 기본값 반환

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
                ManualScript(self.selected_script)
                self.app_state = (
                    APP_STATE.WAITING_FOR_RUNNING
                )  # 스크립트 실행 후 RUNNING 상태을 기다림
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

        # 새로운 상태 전환 로직 추가
        elif self.app_state == APP_STATE.EXECUTING_SCRIPT:
            # 현재 상태는 스크립트를 실행 중
            if self.current_robot_status == COBOT_STATUS.RUNNING:
                self.get_logger().info(
                    "스크립트 실행으로 인해 로봇이 RUNNING 상태입니다."
                )
                self.app_state = APP_STATE.WAITING_FOR_IDLE_AFTER_RUNNING
                self.publish_feedback("스크립트 실행 완료, 로봇이 RUNNING 상태임.")

    def publish_feedback(self, msg: str):
        if self.action_goal_handle:
            feedback = RobotModeControl.Feedback()
            feedback.feedback = msg
            self.action_goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"피드백 전송: {msg}")


def main(args=None):
    rclpy.init(args=args)
    robot_mode_control_server = RobotModeControlServer()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_mode_control_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_mode_control_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
