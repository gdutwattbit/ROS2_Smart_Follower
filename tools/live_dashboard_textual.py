#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from smart_follower_msgs.msg import FollowCommand, PersonPoseArray, TrackedPerson

try:
    from rclpy.parameter_client import AsyncParameterClient
except ModuleNotFoundError:  # Some Humble installs do not ship this helper module
    AsyncParameterClient = None

TEXTUAL_IMPORT_ERROR = None
try:
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from textual.app import App, ComposeResult
    from textual.binding import Binding
    from textual.containers import Horizontal, Vertical
    from textual.widgets import Footer, Header, Static
except ModuleNotFoundError as exc:  # Optional dependency
    TEXTUAL_IMPORT_ERROR = exc

    class _Dummy:  # pragma: no cover - fallback only
        def __init__(self, *args, **kwargs):
            pass

    class App:  # type: ignore[override]
        pass

    ComposeResult = object  # type: ignore[assignment]
    Binding = _Dummy  # type: ignore[assignment]
    Horizontal = _Dummy  # type: ignore[assignment]
    Vertical = _Dummy  # type: ignore[assignment]
    Footer = _Dummy  # type: ignore[assignment]
    Header = _Dummy  # type: ignore[assignment]
    Static = _Dummy  # type: ignore[assignment]

    def Panel(*args, **kwargs):  # type: ignore[override]
        return None

    class Table:  # type: ignore[override]
        def __init__(self, *args, **kwargs):
            pass

    class Text(str):  # type: ignore[override]
        pass


def normalize_ns(ns: str) -> str:
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def join_topic(robot_ns: str, relative_or_absolute: str) -> str:
    if relative_or_absolute.startswith("/"):
        return relative_or_absolute
    if not robot_ns:
        return "/" + relative_or_absolute
    return f"{robot_ns}/{relative_or_absolute}"


def bool_mark(value: Optional[bool]) -> str:
    if value is None:
        return "N/A"
    return "是" if value else "否"


def safe_float(value: object) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def safe_int(value: object) -> Optional[int]:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def fmt_float(value: Optional[float], digits: int = 2, suffix: str = "") -> str:
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}{suffix}"


def fmt_hz(value: Optional[float]) -> str:
    if value is None:
        return "N/A"
    return f"{value:.1f} Hz"


def fmt_age(value: Optional[float]) -> str:
    if value is None:
        return "N/A"
    return f"{value:.2f} s"


def fmt_xy(linear: Optional[float], angular: Optional[float]) -> str:
    if linear is None or angular is None:
        return "N/A"
    return f"v={linear:+.3f}  w={angular:+.3f}"


def track_state_name(state: int) -> str:
    if state == TrackedPerson.TENTATIVE:
        return "TENTATIVE"
    if state == TrackedPerson.CONFIRMED:
        return "CONFIRMED"
    if state == TrackedPerson.LOST:
        return "LOST"
    return str(state)


def lock_state_name(state: int) -> str:
    if state == PersonPoseArray.IDLE:
        return "IDLE"
    if state == PersonPoseArray.LOCKED:
        return "LOCKED"
    if state == PersonPoseArray.LOST:
        return "LOST"
    return str(state)


def arbiter_mode_name(mode: Optional[int]) -> str:
    mapping = {
        0: "停止",
        1: "跟随",
        2: "降级跟随",
        3: "搜索",
        4: "急停",
    }
    if mode is None:
        return "N/A"
    return mapping.get(mode, str(mode))


def follow_command_name(command: int) -> str:
    mapping = {
        FollowCommand.LOCK: "锁定",
        FollowCommand.UNLOCK: "解锁",
        FollowCommand.RESET: "重置",
        FollowCommand.ESTOP: "急停",
    }
    return mapping.get(command, str(command))


class RateTracker:
    def __init__(self, window_sec: float = 5.0) -> None:
        self.window_sec = window_sec
        self.samples: Deque[float] = deque()

    def mark(self, now: Optional[float] = None) -> None:
        t = time.monotonic() if now is None else now
        self.samples.append(t)
        self._trim(t)

    def hz(self, now: Optional[float] = None) -> Optional[float]:
        t = time.monotonic() if now is None else now
        self._trim(t)
        if len(self.samples) < 2:
            return None
        duration = self.samples[-1] - self.samples[0]
        if duration <= 1e-6:
            return None
        return (len(self.samples) - 1) / duration

    def _trim(self, now: float) -> None:
        while self.samples and now - self.samples[0] > self.window_sec:
            self.samples.popleft()


@dataclass
class TopicSample:
    last_seen: Optional[float] = None
    hz_tracker: RateTracker = field(default_factory=RateTracker)
    count: int = 0

    def mark(self, now: Optional[float] = None) -> None:
        t = time.monotonic() if now is None else now
        self.last_seen = t
        self.count += 1
        self.hz_tracker.mark(t)

    def age(self, now: float) -> Optional[float]:
        if self.last_seen is None:
            return None
        return max(0.0, now - self.last_seen)

    def hz(self, now: float) -> Optional[float]:
        return self.hz_tracker.hz(now)


@dataclass
class TwistSample:
    topic: TopicSample = field(default_factory=TopicSample)
    linear_x: Optional[float] = None
    angular_z: Optional[float] = None


@dataclass
class RangeSample:
    topic: TopicSample = field(default_factory=TopicSample)
    range_m: Optional[float] = None


@dataclass
class DiagnosticBucket:
    name: str = ""
    level: int = DiagnosticStatus.STALE
    message: str = ""
    values: Dict[str, str] = field(default_factory=dict)
    last_seen: Optional[float] = None


class RosDashboardBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("live_dashboard_textual_bridge")
        self.args = args
        self.robot_ns = normalize_ns(args.robot_ns)
        self.start_time = time.monotonic()
        self.lock = threading.Lock()

        self.graph_nodes: List[Tuple[str, str]] = []
        self.graph_services: List[Tuple[str, Sequence[str]]] = []

        self.color = TopicSample()
        self.depth = TopicSample()
        self.person_pose = TopicSample()
        self.diagnostics_topic = TopicSample()
        self.follow_cmd = TwistSample()
        self.avoid_cmd = TwistSample()
        self.final_cmd = TwistSample()
        self.left_range = RangeSample()
        self.right_range = RangeSample()

        self.latest_person_pose: Optional[PersonPoseArray] = None
        self.latest_locked_person: Optional[TrackedPerson] = None
        self.diag_buckets: Dict[str, DiagnosticBucket] = {}
        self.follower_params: Dict[str, str] = {}
        self.perception_params: Dict[str, str] = {}
        self.param_requests_inflight = {"follower": False, "perception": False}
        self.parameter_client_available = AsyncParameterClient is not None

        self.last_command_name = "N/A"
        self.last_command_target_id = -1
        self.last_command_time = "N/A"
        self.last_command_status = "idle"

        self.default_qos = QoSProfile(depth=10)
        self.command_pub = self.create_publisher(FollowCommand, self.args.follow_command_topic, 10)
        self.perception_param_client = (
            AsyncParameterClient(self, join_topic(self.robot_ns, "perception_node"))
            if self.parameter_client_available else None
        )
        self.follower_param_client = (
            AsyncParameterClient(self, join_topic(self.robot_ns, "follower_controller_node"))
            if self.parameter_client_available else None
        )

        self._create_subscriptions()
        self.graph_timer = self.create_timer(1.0, self.refresh_graph_state)
        self.param_timer = self.create_timer(2.0, self.refresh_parameters)

    def _create_subscriptions(self) -> None:
        self.create_subscription(Image, self.args.color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.args.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(PersonPoseArray, self.args.person_pose_topic, self._on_person_pose, self.default_qos)
        self.create_subscription(DiagnosticArray, self.args.diagnostics_topic, self._on_diagnostics, self.default_qos)
        self.create_subscription(Twist, self.args.follow_cmd_topic, self._on_follow_cmd, self.default_qos)
        self.create_subscription(Twist, self.args.avoid_cmd_topic, self._on_avoid_cmd, self.default_qos)
        self.create_subscription(Twist, self.args.final_cmd_topic, self._on_final_cmd, self.default_qos)
        self.create_subscription(Range, self.args.left_range_topic, self._on_left_range, qos_profile_sensor_data)
        self.create_subscription(Range, self.args.right_range_topic, self._on_right_range, qos_profile_sensor_data)
    def _on_color(self, _: Image) -> None:
        with self.lock:
            self.color.mark()

    def _on_depth(self, _: Image) -> None:
        with self.lock:
            self.depth.mark()

    def _on_person_pose(self, msg: PersonPoseArray) -> None:
        with self.lock:
            self.person_pose.mark()
            self.latest_person_pose = msg
            self.latest_locked_person = None
            if msg.lock_id >= 0:
                for person in msg.persons:
                    if person.track_id == msg.lock_id:
                        self.latest_locked_person = person
                        break

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        now = time.monotonic()
        with self.lock:
            self.diagnostics_topic.mark(now)
            for status in msg.status:
                bucket_name = self._guess_diag_bucket(status)
                if not bucket_name:
                    continue
                self.diag_buckets[bucket_name] = DiagnosticBucket(
                    name=status.name,
                    level=status.level,
                    message=status.message,
                    values={item.key: item.value for item in status.values},
                    last_seen=now,
                )

    def _guess_diag_bucket(self, status: DiagnosticStatus) -> Optional[str]:
        keys = {item.key for item in status.values}
        name = status.name.lower()
        if "profile_last_total_ms" in keys or "yolo_ready" in keys or "lock_id" in keys:
            return "perception"
        if "prediction_age_s" in keys or "target_speed_mps" in keys or "follow_controller" in name:
            return "follower"
        if "left_dist" in keys or "current_speed" in keys or "avoidance_status" in name:
            return "avoidance"
        if "stop_latched" in keys or "avoid_latched" in keys or "arbiter_status" in name:
            return "arbiter"
        if "gpio_backend" in keys or "last_left_range" in keys or "ultrasonic_status" in name:
            return "ultrasonic"
        return None

    def _on_follow_cmd(self, msg: Twist) -> None:
        with self.lock:
            self.follow_cmd.topic.mark()
            self.follow_cmd.linear_x = float(msg.linear.x)
            self.follow_cmd.angular_z = float(msg.angular.z)

    def _on_avoid_cmd(self, msg: Twist) -> None:
        with self.lock:
            self.avoid_cmd.topic.mark()
            self.avoid_cmd.linear_x = float(msg.linear.x)
            self.avoid_cmd.angular_z = float(msg.angular.z)

    def _on_final_cmd(self, msg: Twist) -> None:
        with self.lock:
            self.final_cmd.topic.mark()
            self.final_cmd.linear_x = float(msg.linear.x)
            self.final_cmd.angular_z = float(msg.angular.z)

    def _on_left_range(self, msg: Range) -> None:
        with self.lock:
            self.left_range.topic.mark()
            self.left_range.range_m = float(msg.range)

    def _on_right_range(self, msg: Range) -> None:
        with self.lock:
            self.right_range.topic.mark()
            self.right_range.range_m = float(msg.range)

    def refresh_graph_state(self) -> None:
        try:
            nodes = self.get_node_names_and_namespaces()
            services = self.get_service_names_and_types()
        except Exception:
            return
        with self.lock:
            self.graph_nodes = nodes
            self.graph_services = services

    def refresh_parameters(self) -> None:
        if not self.parameter_client_available:
            return
        self._request_follower_params()
        self._request_perception_params()

    def _request_follower_params(self) -> None:
        if self.param_requests_inflight["follower"] or self.follower_param_client is None:
            return
        if not self.follower_param_client.service_is_ready():
            return
        names = [
            "target_distance",
            "theta_deadzone",
            "target_timeout",
            "prediction_horizon_s",
            "limits.v_max",
            "limits.w_max",
        ]
        future = self.follower_param_client.get_parameters(names)
        self.param_requests_inflight["follower"] = True

        def _done(done_future) -> None:
            result_map: Dict[str, str] = {}
            try:
                result = done_future.result()
                result_map = {param.name: self._parameter_value_to_string(param) for param in result.values}
            except Exception:
                result_map = {}
            with self.lock:
                self.param_requests_inflight["follower"] = False
                self.follower_params = result_map

        future.add_done_callback(_done)

    def _request_perception_params(self) -> None:
        if self.param_requests_inflight["perception"] or self.perception_param_client is None:
            return
        if not self.perception_param_client.service_is_ready():
            return
        names = [
            "process_every_n_frames",
            "yolo.ort.intra_op_num_threads",
            "yolo.input_w",
            "yolo.input_h",
            "depth_compare.sample_window_px",
            "depth_compare.min_valid_samples",
        ]
        future = self.perception_param_client.get_parameters(names)
        self.param_requests_inflight["perception"] = True

        def _done(done_future) -> None:
            result_map: Dict[str, str] = {}
            try:
                result = done_future.result()
                result_map = {param.name: self._parameter_value_to_string(param) for param in result.values}
            except Exception:
                result_map = {}
            with self.lock:
                self.param_requests_inflight["perception"] = False
                self.perception_params = result_map

        future.add_done_callback(_done)

    @staticmethod
    def _parameter_value_to_string(param) -> str:
        value = param.value
        type_name = value.type
        if type_name == value.TYPE_BOOL:
            return "true" if value.bool_value else "false"
        if type_name == value.TYPE_INTEGER:
            return str(value.integer_value)
        if type_name == value.TYPE_DOUBLE:
            return f"{value.double_value:.3f}"
        if type_name == value.TYPE_STRING:
            return value.string_value
        return "N/A"

    def send_follow_command(self, command: int, target_id: int = -1) -> None:
        msg = FollowCommand()
        msg.command = command
        msg.target_id = target_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(msg)
        with self.lock:
            self.last_command_name = follow_command_name(command)
            self.last_command_target_id = target_id
            self.last_command_time = time.strftime("%H:%M:%S")
            self.last_command_status = "published"

    @staticmethod
    def _diag_value(diags: Dict[str, DiagnosticBucket], bucket: str, key: str) -> Optional[str]:
        status = diags.get(bucket)
        if status is None:
            return None
        return status.values.get(key)

    @staticmethod
    def _diag_float(diags: Dict[str, DiagnosticBucket], bucket: str, key: str) -> Optional[float]:
        return safe_float(RosDashboardBridge._diag_value(diags, bucket, key))

    @staticmethod
    def _diag_int(diags: Dict[str, DiagnosticBucket], bucket: str, key: str) -> Optional[int]:
        return safe_int(RosDashboardBridge._diag_value(diags, bucket, key))

    @staticmethod
    def _diag_bool(diags: Dict[str, DiagnosticBucket], bucket: str, key: str) -> Optional[bool]:
        value = RosDashboardBridge._diag_value(diags, bucket, key)
        if value is None:
            return None
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes"}:
            return True
        if lowered in {"false", "0", "no"}:
            return False
        return None

    def _node_present(self, nodes: Sequence[Tuple[str, str]], full_name: str) -> bool:
        target_ns, _, target_name = full_name.rpartition("/")
        if not target_ns:
            target_ns = "/"
        for name, ns in nodes:
            graph_ns = ns or "/"
            if name == target_name and graph_ns == target_ns:
                return True
        return False

    @staticmethod
    def _service_present(services: Sequence[Tuple[str, Sequence[str]]], service_name: str) -> bool:
        return any(name == service_name for name, _ in services)
    def snapshot(self) -> Dict[str, object]:
        now = time.monotonic()
        with self.lock:
            nodes = list(self.graph_nodes)
            services = list(self.graph_services)
            diags = {
                key: DiagnosticBucket(
                    name=value.name,
                    level=value.level,
                    message=value.message,
                    values=dict(value.values),
                    last_seen=value.last_seen,
                )
                for key, value in self.diag_buckets.items()
            }
            follower_params = dict(self.follower_params)
            perception_params = dict(self.perception_params)
            pose_msg = self.latest_person_pose
            locked_person = self.latest_locked_person

            data: Dict[str, object] = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "uptime_s": now - self.start_time,
                "robot_ns": self.robot_ns or "/",
                "graph_node_count": len(nodes),
                "graph_service_count": len(services),
                "node_perception": self._node_present(nodes, join_topic(self.robot_ns, "perception_node")),
                "node_follower": self._node_present(nodes, join_topic(self.robot_ns, "follower_controller_node")),
                "node_ultrasonic": self._node_present(nodes, join_topic(self.robot_ns, "ultrasonic_range_node")),
                "node_avoidance": self._node_present(nodes, join_topic(self.robot_ns, "obstacle_avoidance_node")),
                "node_arbiter": self._node_present(nodes, join_topic(self.robot_ns, "arbiter_node")),
                "node_keyboard": self._node_present(nodes, join_topic(self.robot_ns, "keyboard_command_node")),
                "camera_info_service": self._service_present(services, self.args.camera_info_service),
                "parameter_client_available": self.parameter_client_available,
                "color_hz": self.color.hz(now),
                "color_age": self.color.age(now),
                "depth_hz": self.depth.hz(now),
                "depth_age": self.depth.age(now),
                "diagnostics_hz": self.diagnostics_topic.hz(now),
                "person_pose_hz": self.person_pose.hz(now),
                "person_pose_age": self.person_pose.age(now),
                "follow_cmd_hz": self.follow_cmd.topic.hz(now),
                "follow_cmd_age": self.follow_cmd.topic.age(now),
                "follow_cmd_v": self.follow_cmd.linear_x,
                "follow_cmd_w": self.follow_cmd.angular_z,
                "avoid_cmd_hz": self.avoid_cmd.topic.hz(now),
                "avoid_cmd_v": self.avoid_cmd.linear_x,
                "avoid_cmd_w": self.avoid_cmd.angular_z,
                "final_cmd_hz": self.final_cmd.topic.hz(now),
                "final_cmd_v": self.final_cmd.linear_x,
                "final_cmd_w": self.final_cmd.angular_z,
                "left_range": self.left_range.range_m,
                "left_range_hz": self.left_range.topic.hz(now),
                "right_range": self.right_range.range_m,
                "right_range_hz": self.right_range.topic.hz(now),
                "last_command_name": self.last_command_name,
                "last_command_target_id": self.last_command_target_id,
                "last_command_time": self.last_command_time,
                "last_command_status": self.last_command_status,
                "diag_perception_level": diags.get("perception", DiagnosticBucket()).level,
                "diag_perception_message": diags.get("perception", DiagnosticBucket()).message,
                "diag_follower_level": diags.get("follower", DiagnosticBucket()).level,
                "diag_follower_message": diags.get("follower", DiagnosticBucket()).message,
                "diag_arbiter_level": diags.get("arbiter", DiagnosticBucket()).level,
                "diag_arbiter_message": diags.get("arbiter", DiagnosticBucket()).message,
                "intrinsics_ready": self._diag_bool(diags, "perception", "intrinsics_ready"),
                "intrinsics_source": self._diag_value(diags, "perception", "intrinsics_source") or "N/A",
                "camera_fx": self._diag_float(diags, "perception", "camera_fx"),
                "camera_fy": self._diag_float(diags, "perception", "camera_fy"),
                "camera_cx": self._diag_float(diags, "perception", "camera_cx"),
                "camera_cy": self._diag_float(diags, "perception", "camera_cy"),
                "persons_count": len(pose_msg.persons) if pose_msg is not None else None,
                "lock_id": int(pose_msg.lock_id) if pose_msg is not None else None,
                "lock_state_name": lock_state_name(pose_msg.lock_state) if pose_msg is not None else "N/A",
                "detections": self._diag_int(diags, "perception", "last_detection_count"),
                "tracks": self._diag_int(diags, "perception", "active_tracks"),
                "yolo_ready": self._diag_bool(diags, "perception", "yolo_ready"),
                "reid_ready": self._diag_bool(diags, "perception", "reid_ready"),
                "last_total_ms": self._diag_float(diags, "perception", "profile_last_total_ms"),
                "last_yolo_ms": self._diag_float(diags, "perception", "profile_last_yolo_ms"),
                "last_reid_ms": self._diag_float(diags, "perception", "profile_last_reid_ms"),
                "avg_total_ms": self._diag_float(diags, "perception", "profile_avg_total_ms"),
                "avg_yolo_ms": self._diag_float(diags, "perception", "profile_avg_yolo_ms"),
                "avg_reid_ms": self._diag_float(diags, "perception", "profile_avg_reid_ms"),
                "depth_position_ms": self._diag_float(diags, "perception", "depth_position_ms"),
                "last_valid_depth_m": self._diag_float(diags, "perception", "last_valid_depth_m"),
                "perception_process_every_n_frames": perception_params.get("process_every_n_frames", "N/A"),
                "perception_yolo_threads": perception_params.get("yolo.ort.intra_op_num_threads", "N/A"),
                "perception_yolo_input": f"{perception_params.get('yolo.input_w', 'N/A')}x{perception_params.get('yolo.input_h', 'N/A')}",
                "follower_target_valid": self._diag_bool(diags, "follower", "target_valid"),
                "follower_target_seen": self._diag_bool(diags, "follower", "target_seen"),
                "follower_target_age_s": self._diag_float(diags, "follower", "target_age_s"),
                "follower_prediction_age_s": self._diag_float(diags, "follower", "prediction_age_s"),
                "follower_target_vx": self._diag_float(diags, "follower", "target_vx"),
                "follower_target_vy": self._diag_float(diags, "follower", "target_vy"),
                "follower_target_speed": self._diag_float(diags, "follower", "target_speed_mps"),
                "follower_predicted_valid": self._diag_bool(diags, "follower", "predicted_target_valid"),
                "follower_target_distance": follower_params.get("target_distance", "N/A"),
                "follower_theta_deadzone": follower_params.get("theta_deadzone", "N/A"),
                "follower_target_timeout": follower_params.get("target_timeout", "N/A"),
                "follower_prediction_horizon": follower_params.get("prediction_horizon_s", "N/A"),
                "follower_limit_v": follower_params.get("limits.v_max", "N/A"),
                "follower_limit_w": follower_params.get("limits.w_max", "N/A"),
                "arbiter_mode": self._diag_int(diags, "arbiter", "mode"),
                "arbiter_stop_latched": self._diag_bool(diags, "arbiter", "stop_latched"),
                "arbiter_avoid_latched": self._diag_bool(diags, "arbiter", "avoid_latched"),
                "arbiter_target_age_s": self._diag_float(diags, "arbiter", "last_target_age_s"),
                "avoid_left_dist": self._diag_float(diags, "avoidance", "left_dist"),
                "avoid_right_dist": self._diag_float(diags, "avoidance", "right_dist"),
                "avoid_left_age_s": self._diag_float(diags, "avoidance", "left_age_s"),
                "avoid_right_age_s": self._diag_float(diags, "avoidance", "right_age_s"),
                "ultrasonic_gpio_ok": self._diag_bool(diags, "ultrasonic", "gpio_ok"),
                "ultrasonic_gpio_backend": self._diag_value(diags, "ultrasonic", "gpio_backend") or "N/A",
            }

            if locked_person is None:
                data["locked_target"] = None
            else:
                cx = float(locked_person.bbox.x_offset) + 0.5 * float(locked_person.bbox.width)
                cy = float(locked_person.bbox.y_offset) + 0.5 * float(locked_person.bbox.height)
                data["locked_target"] = {
                    "id": int(locked_person.track_id),
                    "state": track_state_name(locked_person.track_state),
                    "confidence": float(locked_person.confidence),
                    "pos_x": float(locked_person.position.x),
                    "pos_y": float(locked_person.position.y),
                    "bbox_cx": cx,
                    "bbox_cy": cy,
                    "bbox_w": int(locked_person.bbox.width),
                    "bbox_h": int(locked_person.bbox.height),
                }

        rows = []
        if pose_msg is not None:
            for person in sorted(pose_msg.persons, key=lambda item: int(item.track_id)):
                rows.append({
                    "id": int(person.track_id),
                    "state": track_state_name(person.track_state),
                    "confidence": float(person.confidence),
                    "pos_x": float(person.position.x),
                    "pos_y": float(person.position.y),
                    "locked": pose_msg.lock_id == person.track_id and pose_msg.lock_state == PersonPoseArray.LOCKED,
                })
        data["target_rows"] = rows
        data["warnings"] = self._build_warnings(data)
        return data

    def _build_warnings(self, data: Dict[str, object]) -> List[str]:
        warnings: List[str] = []
        color_age = safe_float(data.get("color_age"))
        depth_age = safe_float(data.get("depth_age"))
        pose_age = safe_float(data.get("person_pose_age"))

        if color_age is None:
            warnings.append("WARN  还没收到 color 图像")
        elif color_age > 0.5:
            warnings.append(f"WARN  color 图像延迟 {color_age:.2f}s")

        if depth_age is None:
            warnings.append("WARN  还没收到 depth 图像")
        elif depth_age > 0.5:
            warnings.append(f"WARN  depth 图像延迟 {depth_age:.2f}s")

        if pose_age is None:
            warnings.append("WARN  还没收到 /person_pose")
        elif pose_age > 0.5:
            warnings.append(f"WARN  /person_pose 延迟 {pose_age:.2f}s")

        for prefix in ("perception", "follower", "arbiter"):
            level = safe_int(data.get(f"diag_{prefix}_level"))
            message = str(data.get(f"diag_{prefix}_message") or "")
            if level is not None and level >= DiagnosticStatus.WARN and message:
                warnings.append(f"{self._level_name(level):<5} {prefix}: {message}")

        if data.get("intrinsics_ready") is False:
            warnings.append("WARN  intrinsics 尚未就绪")
        if data.get("yolo_ready") is False:
            warnings.append("ERROR YOLO runtime 未就绪")
        if data.get("reid_ready") is False:
            warnings.append("WARN  ReID runtime 未就绪")
        if data.get("lock_state_name") == "LOCKED" and data.get("follower_target_valid") is False:
            warnings.append("WARN  已锁定但 follower 认为目标无效")

        follow_nonzero = self._twist_nonzero(safe_float(data.get("follow_cmd_v")), safe_float(data.get("follow_cmd_w")))
        final_zero = not self._twist_nonzero(safe_float(data.get("final_cmd_v")), safe_float(data.get("final_cmd_w")))
        if follow_nonzero and final_zero:
            warnings.append("WARN  跟随指令非零，但最终 /cmd_vel 为 0")

        avoid_nonzero = self._twist_nonzero(safe_float(data.get("avoid_cmd_v")), safe_float(data.get("avoid_cmd_w")))
        if avoid_nonzero or data.get("arbiter_avoid_latched"):
            warnings.append("WARN  当前避障正在介入")

        if not warnings:
            warnings.append("OK    暂无明显报警")
        return warnings

    @staticmethod
    def _twist_nonzero(linear: Optional[float], angular: Optional[float]) -> bool:
        return abs(linear or 0.0) > 1e-3 or abs(angular or 0.0) > 1e-3

    @staticmethod
    def _level_name(level: int) -> str:
        mapping = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }
        return mapping.get(level, str(level))

class DashboardTextualApp(App):
    CSS = """
    Screen {
        layout: vertical;
    }

    #root {
        height: 1fr;
        layout: vertical;
        padding: 0 1;
    }

    .row {
        layout: horizontal;
        height: 1fr;
    }

    .panel {
        width: 1fr;
        height: 1fr;
        margin: 0 1 1 0;
    }

    .panel-last {
        margin-right: 0;
    }

    .half-panel {
        width: 1fr;
        height: 1fr;
        margin: 0 1 1 0;
    }

    #warnings {
        height: 12;
    }

    #target-table {
        height: 12;
    }

    #control {
        height: 6;
        margin-bottom: 1;
    }
    """

    BINDINGS = [
        Binding("l", "lock", "自动锁定"),
        Binding("enter", "lock_selected", "锁定选中目标"),
        Binding("u", "unlock", "解锁"),
        Binding("r", "reset_follow", "重置"),
        Binding("q", "estop", "急停"),
        Binding("up", "select_prev", "上一目标"),
        Binding("down", "select_next", "下一目标"),
        Binding("x", "quit", "退出"),
        Binding("minus", "scale_down", "缩小显示"),
        Binding("equals", "scale_up", "放大显示"),
    ]

    def __init__(self, bridge: RosDashboardBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.selected_target_id: Optional[int] = None
        self.last_feedback_key = ""
        self.last_feedback_time = 0.0
        self.ui_scale = getattr(bridge.args, "ui_scale", "normal")

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Vertical(id="root"):
            with Horizontal(classes="row"):
                yield Static(id="system", classes="panel")
                yield Static(id="camera", classes="panel")
                yield Static(id="perception", classes="panel panel-last")
            with Horizontal(classes="row"):
                yield Static(id="target", classes="panel")
                yield Static(id="follower", classes="panel")
                yield Static(id="obstacle", classes="panel panel-last")
            with Horizontal(classes="row"):
                yield Static(id="warnings", classes="half-panel")
                yield Static(id="target-table", classes="half-panel panel-last")
            yield Static(id="control")
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.2, self.refresh_panels)
        self.title = "ROS2 Smart Follower 集成仪表盘"
        self.sub_title = "Textual 仪表盘 + FollowCommand 快捷控制"
        self._apply_ui_scale()
        self.refresh_panels()

    def _apply_ui_scale(self) -> None:
        warning_height = {"compact": 9, "normal": 12, "large": 15}[self.ui_scale]
        control_height = {"compact": 5, "normal": 6, "large": 8}[self.ui_scale]
        self.query_one("#warnings", Static).styles.height = warning_height
        self.query_one("#target-table", Static).styles.height = warning_height
        self.query_one("#control", Static).styles.height = control_height

    def _join_lines(self, lines: List[str]) -> str:
        if self.ui_scale == "large":
            return "\n\n".join(lines)
        return "\n".join(lines)

    def action_scale_down(self) -> None:
        order = ["compact", "normal", "large"]
        idx = max(0, order.index(self.ui_scale) - 1)
        self.ui_scale = order[idx]
        self._mark_feedback("-")
        self._apply_ui_scale()
        self.refresh_panels()

    def action_scale_up(self) -> None:
        order = ["compact", "normal", "large"]
        idx = min(len(order) - 1, order.index(self.ui_scale) + 1)
        self.ui_scale = order[idx]
        self._mark_feedback("=")
        self._apply_ui_scale()
        self.refresh_panels()

    def _mark_feedback(self, key_name: str) -> None:
        self.last_feedback_key = key_name
        self.last_feedback_time = time.monotonic()

    def action_lock(self) -> None:
        self.bridge.send_follow_command(FollowCommand.LOCK, -1)
        self._mark_feedback("L")
        self.refresh_panels()

    def action_lock_selected(self) -> None:
        if self.selected_target_id is None:
            self._mark_feedback("ENTER(no target)")
            self.refresh_panels()
            return
        self.bridge.send_follow_command(FollowCommand.LOCK, int(self.selected_target_id))
        self._mark_feedback("ENTER")
        self.refresh_panels()

    def action_unlock(self) -> None:
        self.bridge.send_follow_command(FollowCommand.UNLOCK, -1)
        self._mark_feedback("U")
        self.refresh_panels()

    def action_reset_follow(self) -> None:
        self.bridge.send_follow_command(FollowCommand.RESET, -1)
        self._mark_feedback("R")
        self.refresh_panels()

    def action_estop(self) -> None:
        self.bridge.send_follow_command(FollowCommand.ESTOP, -1)
        self._mark_feedback("Q")
        self.refresh_panels()

    def action_select_prev(self) -> None:
        self._move_selection(-1)
        self._mark_feedback("UP")
        self.refresh_panels()

    def action_select_next(self) -> None:
        self._move_selection(1)
        self._mark_feedback("DOWN")
        self.refresh_panels()

    def _move_selection(self, delta: int) -> None:
        rows = self.bridge.snapshot().get("target_rows", [])
        if not isinstance(rows, list) or not rows:
            self.selected_target_id = None
            return
        ids = [int(row["id"]) for row in rows if isinstance(row, dict) and "id" in row]
        if not ids:
            self.selected_target_id = None
            return
        if self.selected_target_id not in ids:
            self.selected_target_id = ids[0]
            return
        index = ids.index(int(self.selected_target_id))
        self.selected_target_id = ids[(index + delta) % len(ids)]

    def _sync_selection(self, data: Dict[str, object]) -> None:
        rows = data.get("target_rows", [])
        if not isinstance(rows, list) or not rows:
            self.selected_target_id = None
            return
        ids = [int(row["id"]) for row in rows if isinstance(row, dict) and "id" in row]
        if not ids:
            self.selected_target_id = None
            return
        if self.selected_target_id in ids:
            return
        lock_id = safe_int(data.get("lock_id"))
        if lock_id in ids:
            self.selected_target_id = lock_id
        else:
            self.selected_target_id = ids[0]

    def refresh_panels(self) -> None:
        data = self.bridge.snapshot()
        self._sync_selection(data)
        self.query_one("#system", Static).update(Panel(self._system_text(data), title="系统总览"))
        self.query_one("#camera", Static).update(Panel(self._camera_text(data), title="相机 / 输入"))
        self.query_one("#perception", Static).update(Panel(self._perception_text(data), title="感知链路"))
        self.query_one("#target", Static).update(Panel(self._target_text(data), title="当前锁定目标"))
        self.query_one("#follower", Static).update(Panel(self._follower_text(data), title="跟随控制"))
        self.query_one("#obstacle", Static).update(Panel(self._obstacle_text(data), title="避障 / 仲裁"))
        self.query_one("#warnings", Static).update(Panel(self._warnings_text(data), title="报警 / 提示"))
        self.query_one("#target-table", Static).update(Panel(self._target_table(data), title="目标 ID 列表"))
        self.query_one("#control", Static).update(Panel(self._control_text(data), title="控制 / 快捷键"))

    def _system_text(self, data: Dict[str, object]) -> str:
        lines = [
            f"时间={data['timestamp']}",
            f"运行时长={safe_float(data.get('uptime_s')) or 0.0:.1f}s  命名空间={data['robot_ns']}",
            f"节点数={data['graph_node_count']}  服务数={data['graph_service_count']}",
            f"感知节点={bool_mark(data.get('node_perception'))}",
            f"跟随节点={bool_mark(data.get('node_follower'))}",
            f"超声节点={bool_mark(data.get('node_ultrasonic'))}",
            f"避障节点={bool_mark(data.get('node_avoidance'))}",
            f"仲裁节点={bool_mark(data.get('node_arbiter'))}",
            f"键控节点={bool_mark(data.get('node_keyboard'))}",
            f"相机内参服务={bool_mark(data.get('camera_info_service'))}",
            f"参数客户端={bool_mark(data.get('parameter_client_available'))}",
        ]
        return self._join_lines(lines)

    def _camera_text(self, data: Dict[str, object]) -> str:
        lines = [
            f"彩色图: 频率={fmt_hz(safe_float(data.get('color_hz')))}  延迟={fmt_age(safe_float(data.get('color_age')))}",
            f"深度图: 频率={fmt_hz(safe_float(data.get('depth_hz')))}  延迟={fmt_age(safe_float(data.get('depth_age')))}",
            f"诊断流: 频率={fmt_hz(safe_float(data.get('diagnostics_hz')))}",
            f"内参就绪={bool_mark(data.get('intrinsics_ready'))}",
            f"内参来源={data.get('intrinsics_source')}",
            f"fx={fmt_float(safe_float(data.get('camera_fx')), 1)}",
            f"fy={fmt_float(safe_float(data.get('camera_fy')), 1)}",
            f"cx={fmt_float(safe_float(data.get('camera_cx')), 1)}",
            f"cy={fmt_float(safe_float(data.get('camera_cy')), 1)}",
        ]
        return self._join_lines(lines)

    def _perception_text(self, data: Dict[str, object]) -> str:
        lines = [
            f"person_pose: 频率={fmt_hz(safe_float(data.get('person_pose_hz')))}  延迟={fmt_age(safe_float(data.get('person_pose_age')))}",
            f"目标数={data.get('persons_count', 'N/A')}  锁定={data.get('lock_id', 'N/A')} / {data.get('lock_state_name', 'N/A')}",
            f"检测框={data.get('detections', 'N/A')}  跟踪轨迹={data.get('tracks', 'N/A')}",
            f"YOLO就绪={bool_mark(data.get('yolo_ready'))}  ReID就绪={bool_mark(data.get('reid_ready'))}",
            f"最近耗时 总/YOLO/ReID={fmt_float(safe_float(data.get('last_total_ms')), 1, 'ms')} / {fmt_float(safe_float(data.get('last_yolo_ms')), 1, 'ms')} / {fmt_float(safe_float(data.get('last_reid_ms')), 1, 'ms')}",
            f"平均耗时 总/YOLO/ReID={fmt_float(safe_float(data.get('avg_total_ms')), 1, 'ms')} / {fmt_float(safe_float(data.get('avg_yolo_ms')), 1, 'ms')} / {fmt_float(safe_float(data.get('avg_reid_ms')), 1, 'ms')}",
            f"定位耗时={fmt_float(safe_float(data.get('depth_position_ms')), 1, 'ms')}  最近有效深度={fmt_float(safe_float(data.get('last_valid_depth_m')), 3, 'm')}",
            f"抽帧倍率={data.get('perception_process_every_n_frames', 'N/A')}  YOLO线程={data.get('perception_yolo_threads', 'N/A')}",
            f"YOLO输入尺寸={data.get('perception_yolo_input', 'N/A')}",
        ]
        return self._join_lines(lines)

    def _target_text(self, data: Dict[str, object]) -> str:
        target = data.get('locked_target')
        if not isinstance(target, dict):
            lines = [
                f"lock_id={data.get('lock_id', 'N/A')}  锁定状态={data.get('lock_state_name', 'N/A')}",
                f"目标数={data.get('persons_count', 'N/A')}",
                "当前锁定目标不在 persons[] 中",
            ]
        else:
            lines = [
                f"lock_id={data.get('lock_id', 'N/A')}  锁定状态={data.get('lock_state_name', 'N/A')}",
                f"目标数={data.get('persons_count', 'N/A')}",
                f"ID={target['id']}  状态={target['state']}  置信度={target['confidence']:.2f}",
                f"前向距离 x={target['pos_x']:.3f} m",
                f"横向偏移 y={target['pos_y']:.3f} m",
                f"框中心 cx={target['bbox_cx']:.1f}  cy={target['bbox_cy']:.1f}",
                f"框尺寸 w={target['bbox_w']}  h={target['bbox_h']}",
            ]
        return self._join_lines(lines)
    def _follower_text(self, data: Dict[str, object]) -> str:
        lines = [
            f"跟随指令: 频率={fmt_hz(safe_float(data.get('follow_cmd_hz')))}  延迟={fmt_age(safe_float(data.get('follow_cmd_age')))}",
            f"跟随输出: {fmt_xy(safe_float(data.get('follow_cmd_v')), safe_float(data.get('follow_cmd_w')))}",
            f"目标有效={bool_mark(data.get('follower_target_valid'))}  目标可见={bool_mark(data.get('follower_target_seen'))}",
            f"目标年龄={fmt_age(safe_float(data.get('follower_target_age_s')))}  预测年龄={fmt_age(safe_float(data.get('follower_prediction_age_s')))}",
            f"目标速度 vx={fmt_float(safe_float(data.get('follower_target_vx')), 3)}  vy={fmt_float(safe_float(data.get('follower_target_vy')), 3)}",
            f"目标速度标量={fmt_float(safe_float(data.get('follower_target_speed')), 3, 'm/s')}",
            f"预测目标有效={bool_mark(data.get('follower_predicted_valid'))}",
            f"目标距离={data.get('follower_target_distance', 'N/A')}  角度死区={data.get('follower_theta_deadzone', 'N/A')}",
            f"目标超时={data.get('follower_target_timeout', 'N/A')}  预测窗口={data.get('follower_prediction_horizon', 'N/A')}",
            f"速度上限 v/w={data.get('follower_limit_v', 'N/A')} / {data.get('follower_limit_w', 'N/A')}",
        ]
        return self._join_lines(lines)

    def _obstacle_text(self, data: Dict[str, object]) -> str:
        lines = [
            f"左超声距离={fmt_float(safe_float(data.get('left_range')), 3, 'm')}  频率={fmt_hz(safe_float(data.get('left_range_hz')))}",
            f"右超声距离={fmt_float(safe_float(data.get('right_range')), 3, 'm')}  频率={fmt_hz(safe_float(data.get('right_range_hz')))}",
            f"避障指令: 频率={fmt_hz(safe_float(data.get('avoid_cmd_hz')))}  {fmt_xy(safe_float(data.get('avoid_cmd_v')), safe_float(data.get('avoid_cmd_w')))}",
            f"最终 /cmd_vel: 频率={fmt_hz(safe_float(data.get('final_cmd_hz')))}  {fmt_xy(safe_float(data.get('final_cmd_v')), safe_float(data.get('final_cmd_w')))}",
            f"仲裁模式={arbiter_mode_name(safe_int(data.get('arbiter_mode')))}",
            f"停止锁存={bool_mark(data.get('arbiter_stop_latched'))}  避障锁存={bool_mark(data.get('arbiter_avoid_latched'))}",
            f"仲裁目标年龄={fmt_age(safe_float(data.get('arbiter_target_age_s')))}",
            f"避障左右距离={fmt_float(safe_float(data.get('avoid_left_dist')), 3, 'm')} / {fmt_float(safe_float(data.get('avoid_right_dist')), 3, 'm')}",
            f"避障数据年龄={fmt_age(safe_float(data.get('avoid_left_age_s')))} / {fmt_age(safe_float(data.get('avoid_right_age_s')))}",
            f"GPIO正常={bool_mark(data.get('ultrasonic_gpio_ok'))}  后端={data.get('ultrasonic_gpio_backend', 'N/A')}",
        ]
        return self._join_lines(lines)

    def _warnings_text(self, data: Dict[str, object]) -> Text:
        warnings = data.get('warnings', [])
        if not isinstance(warnings, list):
            warnings = [str(warnings)]
        text = Text()
        for index, item in enumerate(warnings):
            line = str(item)
            style = "green" if line.startswith("OK") else "yellow"
            if line.startswith("ERROR"):
                style = "bold red"
            elif line.startswith("WARN"):
                style = "bold yellow"
            text.append(line, style=style)
            if index < len(warnings) - 1:
                text.append("\n")
        return text

    def _target_table(self, data: Dict[str, object]) -> Table:
        table = Table(expand=True)
        table.add_column("选", width=3)
        table.add_column("ID", justify="right")
        table.add_column("状态")
        table.add_column("置信度", justify="right")
        table.add_column("前向X", justify="right")
        table.add_column("横向Y", justify="right")
        rows = data.get("target_rows", [])
        if not isinstance(rows, list) or not rows:
            table.add_row("-", "-", "无目标", "-", "-", "-")
            return table
        for row in rows:
            if not isinstance(row, dict):
                continue
            track_id = int(row.get("id", -1))
            selected = track_id == self.selected_target_id
            locked = bool(row.get("locked", False))
            marker = ">" if selected else " "
            id_text = f"*{track_id}" if locked else str(track_id)
            style = "bold cyan" if selected else ("green" if locked else "")
            table.add_row(
                marker,
                id_text,
                str(row.get("state", "N/A")),
                f"{safe_float(row.get('confidence')) or 0.0:.2f}",
                f"{safe_float(row.get('pos_x')) or 0.0:.2f}",
                f"{safe_float(row.get('pos_y')) or 0.0:.2f}",
                style=style,
            )
        return table

    def _control_text(self, data: Dict[str, object]) -> Text:
        active = time.monotonic() - self.last_feedback_time < 0.9

        def key_style(name: str, fallback: str = "bold white") -> str:
            return "black on bright_green bold" if active and self.last_feedback_key == name else fallback

        up_down_style = "black on bright_green bold" if active and self.last_feedback_key in {"UP", "DOWN"} else "bold cyan"

        text = Text()
        text.append("快捷键: ")
        text.append("L", style=key_style("L"))
        text.append("=自动锁定  ")
        text.append("ENTER", style=key_style("ENTER"))
        text.append("=锁定选中目标  ")
        text.append("U", style=key_style("U"))
        text.append("=解锁  ")
        text.append("R", style=key_style("R"))
        text.append("=重置  ")
        text.append("Q", style=key_style("Q", "bold red"))
        text.append("=急停  ")
        text.append("↑/↓", style=up_down_style)
        text.append("=切换目标  ")
        zoom_style = "black on bright_green bold" if active and self.last_feedback_key in {"-", "="} else "bold magenta"
        text.append("-/=", style=zoom_style)
        text.append("=缩放  ")
        text.append("X", style=key_style("X"))
        text.append("=退出")
        text.append("\n")
        text.append(f"FollowCommand 话题: {self.bridge.args.follow_command_topic}\n")
        text.append(
            f"当前选中ID: {self.selected_target_id if self.selected_target_id is not None else 'N/A'}  "
            f"最近命令: {data.get('last_command_name', 'N/A')}  "
            f"target_id={data.get('last_command_target_id', 'N/A')}\n"
        )
        text.append(
            f"最近发送: {data.get('last_command_time', 'N/A')}  状态={data.get('last_command_status', 'N/A')}  "
            f"按键反馈={self.last_feedback_key or 'N/A'}  显示缩放={self.ui_scale}"
        )
        return text


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2 Smart Follower Textual dashboard")
    parser.add_argument("--robot-ns", default="robot1", help="robot namespace, default: robot1")
    parser.add_argument("--diagnostics-topic", default="/diagnostics")
    parser.add_argument("--camera-info-service", default="/camera/get_camera_info")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--person-pose-topic", default=None)
    parser.add_argument("--follow-cmd-topic", default=None)
    parser.add_argument("--avoid-cmd-topic", default=None)
    parser.add_argument("--final-cmd-topic", default="/cmd_vel")
    parser.add_argument("--left-range-topic", default=None)
    parser.add_argument("--right-range-topic", default=None)
    parser.add_argument("--follow-command-topic", default=None)
    parser.add_argument("--ui-scale", choices=["compact", "normal", "large"], default="normal")
    return parser


def finalize_topics(args: argparse.Namespace) -> argparse.Namespace:
    robot_ns = normalize_ns(args.robot_ns)
    args.person_pose_topic = args.person_pose_topic or join_topic(robot_ns, "person_pose")
    args.follow_cmd_topic = args.follow_cmd_topic or join_topic(robot_ns, "cmd_vel_follow")
    args.avoid_cmd_topic = args.avoid_cmd_topic or join_topic(robot_ns, "cmd_vel_avoid")
    args.left_range_topic = args.left_range_topic or join_topic(robot_ns, "left_ultrasonic/range")
    args.right_range_topic = args.right_range_topic or join_topic(robot_ns, "right_ultrasonic/range")
    args.follow_command_topic = args.follow_command_topic or join_topic(robot_ns, "follow_command")
    return args


def start_bridge(bridge: RosDashboardBridge) -> Tuple[SingleThreadedExecutor, threading.Thread]:
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, name="ros-dashboard-bridge", daemon=True)
    thread.start()
    return executor, thread


def stop_bridge(bridge: RosDashboardBridge, executor: SingleThreadedExecutor, thread: threading.Thread) -> None:
    try:
        executor.remove_node(bridge)
    except Exception:
        pass
    executor.shutdown(timeout_sec=1.0)
    thread.join(timeout=1.0)
    bridge.destroy_node()


def main() -> int:
    if TEXTUAL_IMPORT_ERROR is not None:
        print("[live_dashboard_textual] 缺少 Textual 依赖。")
        print("请先安装：python3 -m pip install textual")
        print("或者先使用轻量版：python3 tools/live_dashboard.py --robot-ns robot1")
        return 2

    args = finalize_topics(build_arg_parser().parse_args())
    rclpy.init()
    bridge = RosDashboardBridge(args)
    executor, thread = start_bridge(bridge)
    app = DashboardTextualApp(bridge)
    try:
        app.run()
    finally:
        stop_bridge(bridge, executor, thread)
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
