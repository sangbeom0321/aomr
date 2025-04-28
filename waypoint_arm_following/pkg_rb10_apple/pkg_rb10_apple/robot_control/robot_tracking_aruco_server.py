import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from interface_rb10_apple.msg import RobotTcpPose
from interface_rb10_apple.action import RobotTrackingControl

from .cobot import *  # 로봇 스크립트 함수(ManualScript 등)
import math
import time
from tf_transformations import euler_from_quaternion
from enum import Enum
from rclpy.executors import MultiThreadedExecutor
import threading
import logging


# 실제 로봇 상태 토픽을 매핑하기 위한 enum
class COBOT_STATUS(Enum):
    UNKNOWN = 0
    PAUSED = 1
    IDLE = 2
    RUNNING = 3


# 내부 상태 머신 enum
class APP_STATE(Enum):
    WAITING_FOR_IDLE = 1  # 로봇이 IDLE이 되기를 대기
    CHECK_THRESHOLD = 2  # 오차 측정 → 임계값 확인
    WAITING_FOR_RUNNING = 3  # movetcp 명령 후 로봇이 RUNNING 될 때까지 대기
    WAITING_FOR_MOTION_COMPLETE = 4  # RUNNING → 다시 IDLE (모션 종료)
    DONE = 5  # 액션이 종료된 상태(Succeed 또는 Abort)


class ArucoTrackingServer(Node):
    def __init__(self):
        super().__init__("aruco_tracking_server")

        # Set log level to DEBUG
        self.get_logger().set_level(logging.WARN)
        # 1) 액션 서버 생성 (Non-blocking 형태로 사용할 예정)
        self._action_server = ActionServer(
            self, RobotTrackingControl, "aruco_tracking", self.execute_callback
        )

        # 2) 토픽 구독
        self.robot_tcp_pose_subscription = self.create_subscription(
            RobotTcpPose, "robot_tcp_pose", self.robot_tcp_pose_callback, 10
        )
        self.aruco_pose_subscription = self.create_subscription(
            PoseArray, "aruco_poses_rs", self.aruco_pose_callback, 10
        )
        self.cobot_status_subscriber = self.create_subscription(
            String, "cobot_status", self.cobot_status_callback, 10
        )

        # 3) 내부 데이터 초기화
        self.current_tcp_pose = RobotTcpPose()
        self.current_aruco_pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
        }
        self.last_rx = 0.0
        self.last_ry = 0.0
        self.last_rz = 0.0

        # 로봇 상태
        self.current_robot_status = COBOT_STATUS.UNKNOWN

        # 오차 임계값
        self.threshold_position = 5  # mm 단위
        self.threshold_orientation = 10  # deg 단위

        # 원하는 오프셋 (환경마다 달라질 수 있음)
        self.desired_offset = {
            "x": 0.0,
            "y": 150.0,  # 축 바뀜
            "z": 0.0,
            "rx": 180.0,
            "ry": 0.0,
            "rz": 0.0,
        }

        # 4) 액션 상태 머신 관련
        self.app_state = APP_STATE.DONE
        self.action_in_progress = False
        self.action_goal_handle = None
        self.action_result = None

        self.move_attempts_orientation = 0
        self.move_attempts_position = 0
        self.max_attempts = 20

        # 5) 디버깅용 타이머 (아루코 포즈 출력)
        self.timer = self.create_timer(5.0, self.print_aruco_pose)

        # 로봇 초기화 (예시)
        self.cobot_initialization()

        # 6) 액션 완료를 신호하기 위한 Event 객체
        self.action_complete = threading.Event()

    def cobot_initialization(self):
        ip = "10.0.99.21"
        ToCB(ip)
        CobotInit()
        SendCOMMAND("pgmode real", CMD_TYPE.NONMOVE)

    # ------------------------------------------------
    # 액션 서버 콜백 (비동기)
    # ------------------------------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("execute_callback: Received action request.")

        # 액션 처리용 변수들 초기화
        self.action_goal_handle = goal_handle
        self.action_result = RobotTrackingControl.Result()
        self.action_in_progress = True

        # move_attempts 리셋
        self.move_attempts_orientation = 0
        self.move_attempts_position = 0

        # 상태 머신 시작 상태
        self.transition_state(APP_STATE.WAITING_FOR_IDLE)
        self.publish_feedback("Action started. Waiting for robot to become IDLE...")

        # **Non-blocking** → 상태 머신이 완료될 때까지 대기
        self.action_complete.clear()
        self.action_complete.wait()

        # 최종 결과 반환
        return self.action_result

    # ------------------------------------------------
    # cobot_status 콜백 - 상태 머신 메인 로직
    # ------------------------------------------------
    def cobot_status_callback(self, msg: String):
        # 현재 로봇 상태 업데이트
        status_str = msg.data
        if status_str == "PAUSED":
            self.current_robot_status = COBOT_STATUS.PAUSED
        elif status_str == "IDLE":
            self.current_robot_status = COBOT_STATUS.IDLE
        elif status_str == "RUNNING":
            self.current_robot_status = COBOT_STATUS.RUNNING
        else:
            self.current_robot_status = COBOT_STATUS.UNKNOWN

        # 상태 머신은 액션이 진행 중일 때만
        if not self.action_in_progress:
            return

        # 비동기 상태 머신
        if self.app_state == APP_STATE.WAITING_FOR_IDLE:
            # 로봇이 IDLE 상태가 되면 → CHECK_THRESHOLD
            if self.current_robot_status == COBOT_STATUS.IDLE:
                self.transition_state(APP_STATE.CHECK_THRESHOLD)
                self.publish_feedback("Robot is IDLE. Checking threshold...")

        elif self.app_state == APP_STATE.CHECK_THRESHOLD:
            self.print_aruco_pose()
            # 오차 계산 → 임계값 확인
            pos_err, ori_err, dx, dy, dz, drx, dry, drz = self.calculate_errors()

            # 먼저 orientation 오류를 확인
            if ori_err >= self.threshold_orientation:
                if self.move_attempts_orientation >= self.max_attempts:
                    self.action_abort("Exceeded maximum orientation move attempts.")
                    return
                self.publish_feedback(
                    "Orientation error exceeds threshold. Correcting orientation..."
                )
                self.move_robot_to_correction(drx, dry, drz, orientation=True)
                self.move_attempts_orientation += 1
                self.transition_state(APP_STATE.WAITING_FOR_RUNNING)
                return

            # Orientation이 만족되면 position 오류를 확인
            if pos_err >= self.threshold_position:
                if self.move_attempts_position >= self.max_attempts:
                    self.action_abort("Exceeded maximum position move attempts.")
                    return
                self.publish_feedback(
                    "Position error exceeds threshold. Correcting position..."
                )
                self.move_robot_to_correction(dx, dy, dz, orientation=False)
                self.move_attempts_position += 1
                self.transition_state(APP_STATE.WAITING_FOR_RUNNING)
                return

            # 둘 다 만족하면 성공
            self.action_succeed("Aruco marker successfully tracked.")

        elif self.app_state == APP_STATE.WAITING_FOR_RUNNING:
            # 로봇 상태가 RUNNING이 되면 → 모션 중
            if self.current_robot_status == COBOT_STATUS.RUNNING:
                self.publish_feedback(
                    "Robot is RUNNING. Waiting for motion to complete..."
                )
                self.transition_state(APP_STATE.WAITING_FOR_MOTION_COMPLETE)

        elif self.app_state == APP_STATE.WAITING_FOR_MOTION_COMPLETE:
            # 모션 완료 → 다시 IDLE → CHECK_THRESHOLD로 돌아가 반복
            if self.current_robot_status == COBOT_STATUS.IDLE:
                self.publish_feedback("Motion complete. Checking threshold again...")
                self.transition_state(APP_STATE.CHECK_THRESHOLD)

        elif self.app_state == APP_STATE.DONE:
            # 이미 끝난 상태
            pass

    # ------------------------------------------------
    # Helper 함수들 (액션 결과, 피드백 등)
    # ------------------------------------------------
    def publish_feedback(self, msg: str):
        if self.action_goal_handle is not None:
            fb = RobotTrackingControl.Feedback()
            fb.feedback = msg
            self.action_goal_handle.publish_feedback(fb)

    def action_succeed(self, message: str):
        if self.action_goal_handle is not None:
            self.action_result.success = True
            self.action_result.message = message
            self.action_goal_handle.succeed()
        self.transition_state(APP_STATE.DONE)
        self.action_in_progress = False
        self.get_logger().info(f"[action_succeed] {message}")
        self.action_complete.set()

    def action_abort(self, message: str):
        if self.action_goal_handle is not None:
            self.action_result.success = False
            self.action_result.message = message
            self.action_goal_handle.abort()
        self.transition_state(APP_STATE.DONE)
        self.action_in_progress = False
        self.get_logger().warn(f"[action_abort] {message}")
        self.action_complete.set()

    def transition_state(self, new_state):
        self.get_logger().info(
            f"State transition: {self.app_state.name} -> {new_state.name}"
        )
        self.app_state = new_state

    # ------------------------------------------------
    # 로봇 Pose & Error 계산 콜백
    # ------------------------------------------------
    def robot_tcp_pose_callback(self, msg: RobotTcpPose):
        self.current_tcp_pose = msg

    def aruco_pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return
        first_pose = msg.poses[0]
        # position: m -> mm
        self.current_aruco_pose["x"] = first_pose.position.x * 1000
        self.current_aruco_pose["y"] = first_pose.position.y * 1000
        self.current_aruco_pose["z"] = first_pose.position.z * 1000

        # orientation: quaternion -> euler
        q = [
            first_pose.orientation.x,
            first_pose.orientation.y,
            first_pose.orientation.z,
            first_pose.orientation.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(q)

        rx_deg = math.degrees(roll)
        ry_deg = math.degrees(pitch)
        rz_deg = math.degrees(yaw)

        rx_deg = self.unwrap_angle(rx_deg, self.last_rx)
        rx_deg = (
            rx_deg + 360 if rx_deg < 0 else rx_deg
        )  # 음수 보정용, 이게 아니면 -180으로 뒤집어짐
        ry_deg = self.unwrap_angle(ry_deg, self.last_ry)
        rz_deg = self.unwrap_angle(rz_deg, self.last_rz)

        self.current_aruco_pose["rx"] = rx_deg
        self.current_aruco_pose["ry"] = ry_deg
        self.current_aruco_pose["rz"] = rz_deg

        self.last_rx = rx_deg
        self.last_ry = ry_deg
        self.last_rz = rz_deg

    def unwrap_angle(self, new_angle_deg, last_angle_deg):
        diff = new_angle_deg - last_angle_deg
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return last_angle_deg + diff

    def calculate_errors(self):
        """
        오차(dx, dy, dz, drx, dry, drz)와
        전체 오차(position_error, orientation_error) 계산
        """
        dx = self.current_aruco_pose["x"] - self.desired_offset["x"]
        dy = self.current_aruco_pose["z"] - self.desired_offset["y"]  # 축 바뀜
        dz = self.current_aruco_pose["y"] - self.desired_offset["z"]

        drx = self.current_aruco_pose["rx"] - self.desired_offset["rx"]
        dry = self.current_aruco_pose["rz"] - self.desired_offset["ry"]  # 축 바뀜
        drz = self.current_aruco_pose["ry"] - self.desired_offset["rz"]  # 축 바뀜

        pos_err = math.sqrt(dx**2 + dy**2 + dz**2)
        ori_err = math.sqrt(drx**2 + dry**2 + drz**2)

        self.get_logger().info(
            f"[calculate_errors] dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}, "
            f"drx={drx:.3f}, dry={dry:.3f}, drz={drz:.3f}, "
            f"pos_err={pos_err:.3f}, ori_err={ori_err:.3f}"
        )

        return (pos_err, ori_err, dx, dy, dz, drx, dry, drz)

    def is_within_threshold(self, position_error, orientation_error):
        return (
            position_error < self.threshold_position
            and orientation_error < self.threshold_orientation
        )

    def move_robot_to_correction(self, delta1, delta2, delta3, orientation=True):
        """
        오차를 보정하기 위한 movetcp 명령을 전송
        orientation=True: drx, dry, drz 보정
        orientation=False: dx, dy, dz 보정
        """
        spd = 0.3
        acc = 0.1

        # scaling 계산기
        scalingFactor_calculator = ScalingFactorCalculator()

        if orientation:
            attempts = self.move_attempts_orientation
            scaling_factor = scalingFactor_calculator.get_scaling_factor(
                "orientation", attempts
            )
            self.get_logger().info(f"Orientation scaling factor: {scaling_factor:.2f}")

            # Apply scaling factor to rotational deltas
            scaled_drx = delta1 * scaling_factor
            scaled_dry = delta2 * scaling_factor
            scaled_drz = delta3 * scaling_factor

            new_rx = self.current_tcp_pose.rx - scaled_drx
            new_ry = self.current_tcp_pose.ry - scaled_dry
            new_rz = self.current_tcp_pose.rz - scaled_drz

            script_tcp = (
                f"movetcp {spd}, {acc}, "
                f"{self.current_tcp_pose.x}, {self.current_tcp_pose.y}, {self.current_tcp_pose.z}, "
                f"{new_rx}, {new_ry}, {new_rz}"
            )
            self.get_logger().info(
                f"[move_robot_to_correction - Orientation] Sending script: {script_tcp}"
            )
        else:
            attempts = self.move_attempts_position

            # 개별 축에 대한 스케일링 팩터 계산
            scaling_factor_x = scalingFactor_calculator.get_scaling_factor(
                "position_x", attempts
            )
            scaling_factor_y = scalingFactor_calculator.get_scaling_factor(
                "position_y", attempts
            )
            scaling_factor_z = scalingFactor_calculator.get_scaling_factor(
                "position_z", attempts
            )

            self.get_logger().info(
                f"Position scaling factors - x: {scaling_factor_x:.2f}, y: {scaling_factor_y:.2f}, z: {scaling_factor_z:.2f}"
            )

            # Apply scaling factors to translational deltas
            scaled_dx = delta1 * scaling_factor_x
            scaled_dy = delta2 * scaling_factor_y
            scaled_dz = delta3 * scaling_factor_z

            new_x = self.current_tcp_pose.x - scaled_dx
            new_y = self.current_tcp_pose.y - scaled_dy
            new_z = self.current_tcp_pose.z - scaled_dz

            script_tcp = (
                f"movetcp {spd}, {acc}, "
                f"{new_x}, {new_y}, {new_z}, "
                f"{self.current_tcp_pose.rx}, {self.current_tcp_pose.ry}, {self.current_tcp_pose.rz}"
            )
            self.get_logger().info(
                f"[move_robot_to_correction - Position] Sending script: {script_tcp}"
            )

        ManualScript(script_tcp)

    # ------------------------------------------------
    # 디버깅용
    # ------------------------------------------------
    def print_aruco_pose(self):
        x = self.current_aruco_pose["x"]
        y = self.current_aruco_pose["y"]
        z = self.current_aruco_pose["z"]
        rx = self.current_aruco_pose["rx"]
        ry = self.current_aruco_pose["ry"]
        rz = self.current_aruco_pose["rz"]
        self.get_logger().info(
            f"[print_aruco_pose] x={x:.1f}, y={y:.1f}, z={z:.1f}, "
            f"rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}"
        )


class ScalingFactorCalculator:
    def __init__(self):
        # 각 카테고리에 대한 스케일링 파라미터를 정의
        self.scaling_params = {
            "orientation": {"min_scale": 0.1, "max_scale": 1.0, "scale_increment": 0.1},
            "position_x": {"min_scale": 0.5, "max_scale": 1.0, "scale_increment": 0.2},
            "position_y": {"min_scale": 0.1, "max_scale": 1.0, "scale_increment": 0.05},
            "position_z": {"min_scale": 0.5, "max_scale": 1.0, "scale_increment": 0.2},
        }

    def get_scaling_factor(self, category, move_attempts):
        """
        주어진 카테고리에 대한 스케일링 팩터를 계산합니다.

        Args:
            category (str): 'orientation', 'position_x', 'position_y', 'position_z' 중 하나.
            move_attempts (int): 시도 횟수.

        Returns:
            float: 계산된 스케일링 팩터.

        Raises:
            ValueError: 알 수 없는 카테고리가 입력된 경우.
        """
        if category not in self.scaling_params:
            raise ValueError(f"Unknown category: {category}")

        params = self.scaling_params[category]
        scaling_factor = params["min_scale"] + (
            move_attempts * params["scale_increment"]
        )
        scaling_factor = min(scaling_factor, params["max_scale"])  # max_scale으로 제한

        return scaling_factor


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackingServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
