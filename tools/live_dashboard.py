#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import shutil
import sys
import textwrap
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

try:
    from rclpy.parameter_client import AsyncParameterClient
except ModuleNotFoundError:  # Some Humble installs do not ship this helper module
    AsyncParameterClient = None
from sensor_msgs.msg import Image, Range
from smart_follower_msgs.msg import PersonPoseArray, TrackedPerson


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
    return "YES" if value else "NO"


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
        0: "STOP",
        1: "FOLLOW",
        2: "FOLLOW_DEGRADED",
        3: "SEARCH",
        4: "ESTOP",
    }
    if mode is None:
        return "N/A"
    return mapping.get(mode, str(mode))


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


class LiveDashboard(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("live_dashboard")
        self.args = args
        self.robot_ns = normalize_ns(args.robot_ns)
        self.start_time = time.monotonic()
        self.graph_nodes: List[Tuple[str, str]] = []
        self.graph_services: List[Tuple[str, Sequence[str]]] = []
        self.hidden_cursor = False

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

        self.default_qos = QoSProfile(depth=10)
        self.parameter_client_available = AsyncParameterClient is not None
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
        self.render_timer = self.create_timer(max(0.1, 1.0 / max(1.0, args.refresh_hz)), self.render)

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
        self.color.mark()

    def _on_depth(self, _: Image) -> None:
        self.depth.mark()

    def _on_person_pose(self, msg: PersonPoseArray) -> None:
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
        self.follow_cmd.topic.mark()
        self.follow_cmd.linear_x = float(msg.linear.x)
        self.follow_cmd.angular_z = float(msg.angular.z)

    def _on_avoid_cmd(self, msg: Twist) -> None:
        self.avoid_cmd.topic.mark()
        self.avoid_cmd.linear_x = float(msg.linear.x)
        self.avoid_cmd.angular_z = float(msg.angular.z)

    def _on_final_cmd(self, msg: Twist) -> None:
        self.final_cmd.topic.mark()
        self.final_cmd.linear_x = float(msg.linear.x)
        self.final_cmd.angular_z = float(msg.angular.z)

    def _on_left_range(self, msg: Range) -> None:
        self.left_range.topic.mark()
        self.left_range.range_m = float(msg.range)

    def _on_right_range(self, msg: Range) -> None:
        self.right_range.topic.mark()
        self.right_range.range_m = float(msg.range)

    def refresh_graph_state(self) -> None:
        try:
            self.graph_nodes = self.get_node_names_and_namespaces()
            self.graph_services = self.get_service_names_and_types()
        except Exception:
            pass

    def refresh_parameters(self) -> None:
        if not self.parameter_client_available:
            return
        self._request_follower_params()
        self._request_perception_params()

    def _request_follower_params(self) -> None:
        if self.param_requests_inflight["follower"] or not self.follower_param_client.service_is_ready():
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
            self.param_requests_inflight["follower"] = False
            try:
                result = done_future.result()
                self.follower_params = {param.name: self._parameter_value_to_string(param) for param in result.values}
            except Exception:
                self.follower_params = {}

        future.add_done_callback(_done)

    def _request_perception_params(self) -> None:
        if self.param_requests_inflight["perception"] or not self.perception_param_client.service_is_ready():
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
            self.param_requests_inflight["perception"] = False
            try:
                result = done_future.result()
                self.perception_params = {param.name: self._parameter_value_to_string(param) for param in result.values}
            except Exception:
                self.perception_params = {}

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

    def _diag(self, bucket: str) -> DiagnosticBucket:
        return self.diag_buckets.get(bucket, DiagnosticBucket())

    def _diag_value(self, bucket: str, key: str) -> Optional[str]:
        return self._diag(bucket).values.get(key)

    def _diag_float(self, bucket: str, key: str) -> Optional[float]:
        return safe_float(self._diag_value(bucket, key))

    def _diag_int(self, bucket: str, key: str) -> Optional[int]:
        return safe_int(self._diag_value(bucket, key))

    def _diag_bool(self, bucket: str, key: str) -> Optional[bool]:
        value = self._diag_value(bucket, key)
        if value is None:
            return None
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes"}:
            return True
        if lowered in {"false", "0", "no"}:
            return False
        return None

    def _node_present(self, full_name: str) -> bool:
        target_ns, _, target_name = full_name.rpartition("/")
        if not target_ns:
            target_ns = "/"
        for name, ns in self.graph_nodes:
            graph_ns = ns or "/"
            if name == target_name and graph_ns == target_ns:
                return True
        return False

    def _service_present(self, service_name: str) -> bool:
        return any(name == service_name for name, _ in self.graph_services)

    def _locked_target_lines(self) -> List[str]:
        msg = self.latest_person_pose
        if msg is None:
            return ["person_pose: N/A", "locked target: N/A"]
        lines = [
            f"lock_id={msg.lock_id}  lock_state={lock_state_name(msg.lock_state)}",
            f"persons={len(msg.persons)}",
        ]
        person = self.latest_locked_person
        if person is None:
            lines.append("locked target: not present in current persons[]")
            return lines
        cx = float(person.bbox.x_offset) + 0.5 * float(person.bbox.width)
        cy = float(person.bbox.y_offset) + 0.5 * float(person.bbox.height)
        lines.extend(
            [
                f"id={person.track_id}  state={track_state_name(person.track_state)}  conf={person.confidence:.2f}",
                f"pos.x={person.position.x:.3f} m",
                f"pos.y={person.position.y:.3f} m",
                f"bbox cx={cx:.1f} cy={cy:.1f}",
                f"bbox w={person.bbox.width} h={person.bbox.height}",
            ]
        )
        return lines

    def _warning_lines(self, now: float) -> List[str]:
        warnings: List[str] = []

        color_age = self.color.age(now)
        depth_age = self.depth.age(now)
        pose_age = self.person_pose.age(now)

        if self.color.last_seen is None:
            warnings.append("WARN  还没收到 color 图像")
        elif color_age is not None and color_age > 0.5:
            warnings.append(f"WARN  color 图像延迟 {color_age:.2f}s")

        if self.depth.last_seen is None:
            warnings.append("WARN  还没收到 depth 图像")
        elif depth_age is not None and depth_age > 0.5:
            warnings.append(f"WARN  depth 图像延迟 {depth_age:.2f}s")

        if self.person_pose.last_seen is None:
            warnings.append("WARN  还没收到 /person_pose")
        elif pose_age is not None and pose_age > 0.5:
            warnings.append(f"WARN  /person_pose 延迟 {pose_age:.2f}s")

        perception = self._diag("perception")
        if perception.last_seen is None:
            warnings.append("WARN  perception diagnostics 缺失")
        elif perception.level >= DiagnosticStatus.WARN and perception.message:
            warnings.append(f"{self._level_name(perception.level):<5} perception: {perception.message}")

        follower = self._diag("follower")
        if follower.last_seen is not None and follower.level >= DiagnosticStatus.WARN and follower.message:
            warnings.append(f"{self._level_name(follower.level):<5} follower: {follower.message}")

        arbiter = self._diag("arbiter")
        if arbiter.last_seen is not None and arbiter.level >= DiagnosticStatus.WARN and arbiter.message:
            warnings.append(f"{self._level_name(arbiter.level):<5} arbiter: {arbiter.message}")

        if self._diag_bool("perception", "intrinsics_ready") is False:
            warnings.append("WARN  intrinsics 尚未就绪")

        if self._diag_bool("perception", "yolo_ready") is False:
            warnings.append("ERROR YOLO runtime 未就绪")

        if self._diag_bool("perception", "reid_ready") is False:
            warnings.append("WARN  ReID runtime 未就绪")

        target_valid = self._diag_bool("follower", "target_valid")
        pose_msg = self.latest_person_pose
        if pose_msg is not None and pose_msg.lock_state == PersonPoseArray.LOCKED and target_valid is False:
            warnings.append("WARN  已锁定但 follower 认为目标无效")

        follow_nonzero = self._twist_nonzero(self.follow_cmd)
        final_zero = not self._twist_nonzero(self.final_cmd)
        if follow_nonzero and final_zero:
            warnings.append("WARN  跟随指令非零，但最终 /cmd_vel 为 0")

        avoid_nonzero = self._twist_nonzero(self.avoid_cmd)
        if avoid_nonzero or self._diag_bool("arbiter", "avoid_latched"):
            warnings.append("WARN  当前避障正在介入")

        if not warnings:
            warnings.append("OK    暂无明显报警")

        return warnings

    @staticmethod
    def _twist_nonzero(sample: TwistSample) -> bool:
        lx = sample.linear_x if sample.linear_x is not None else 0.0
        az = sample.angular_z if sample.angular_z is not None else 0.0
        return abs(lx) > 1e-3 or abs(az) > 1e-3

    @staticmethod
    def _level_name(level: int) -> str:
        mapping = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }
        return mapping.get(level, str(level))

    def render(self) -> None:
        now = time.monotonic()
        system_lines = self._build_system_lines(now)
        camera_lines = self._build_camera_lines(now)
        perception_lines = self._build_perception_lines(now)
        target_lines = self._locked_target_lines()
        follower_lines = self._build_follower_lines(now)
        obstacle_lines = self._build_obstacle_lines(now)
        warning_lines = self._warning_lines(now)

        width = max(120, shutil.get_terminal_size((160, 40)).columns)
        gap = 1
        col_width = max(38, (width - gap * 2) // 3)
        full_width = col_width * 3 + gap * 2

        row1 = self._render_row(
            [
                ("SYSTEM OVERVIEW", system_lines),
                ("CAMERA / INPUT", camera_lines),
                ("PERCEPTION", perception_lines),
            ],
            col_width,
            gap,
        )
        row2 = self._render_row(
            [
                ("LOCKED TARGET", target_lines),
                ("FOLLOWER", follower_lines),
                ("OBSTACLE / ARBITER", obstacle_lines),
            ],
            col_width,
            gap,
        )
        warning_box = self._render_box("WARNINGS", warning_lines, full_width, min_inner_height=max(6, len(warning_lines)))

        output = "\n".join(
            ["\x1b[?25l", "\x1b[2J\x1b[H", *row1, "", *row2, "", *warning_box]
        )
        sys.stdout.write(output)
        sys.stdout.flush()
        self.hidden_cursor = True

    def _build_system_lines(self, now: float) -> List[str]:
        uptime = now - self.start_time
        return [
            f"time={time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"uptime={uptime:.1f}s  ns={self.robot_ns or '/'}",
            f"nodes graph={len(self.graph_nodes)}  services={len(self.graph_services)}",
            f"perception={bool_mark(self._node_present(join_topic(self.robot_ns, 'perception_node')))}",
            f"follower={bool_mark(self._node_present(join_topic(self.robot_ns, 'follower_controller_node')))}",
            f"ultrasonic={bool_mark(self._node_present(join_topic(self.robot_ns, 'ultrasonic_range_node')))}",
            f"avoidance={bool_mark(self._node_present(join_topic(self.robot_ns, 'obstacle_avoidance_node')))}",
            f"arbiter={bool_mark(self._node_present(join_topic(self.robot_ns, 'arbiter_node')))}",
            f"keyboard={bool_mark(self._node_present(join_topic(self.robot_ns, 'keyboard_command_node')))}",
            f"camera_info_srv={bool_mark(self._service_present(self.args.camera_info_service))}",
            f"param_client={bool_mark(self.parameter_client_available)}",
        ]

    def _build_camera_lines(self, now: float) -> List[str]:
        intrinsics_source = self._diag_value("perception", "intrinsics_source") or "N/A"
        return [
            f"color: hz={fmt_hz(self.color.hz(now))}  age={fmt_age(self.color.age(now))}",
            f"depth: hz={fmt_hz(self.depth.hz(now))}  age={fmt_age(self.depth.age(now))}",
            f"diag: hz={fmt_hz(self.diagnostics_topic.hz(now))}",
            f"intrinsics_ready={bool_mark(self._diag_bool('perception', 'intrinsics_ready'))}",
            f"intrinsics_source={intrinsics_source}",
            f"fx={fmt_float(self._diag_float('perception', 'camera_fx'), 1)}",
            f"fy={fmt_float(self._diag_float('perception', 'camera_fy'), 1)}",
            f"cx={fmt_float(self._diag_float('perception', 'camera_cx'), 1)}",
            f"cy={fmt_float(self._diag_float('perception', 'camera_cy'), 1)}",
        ]

    def _build_perception_lines(self, now: float) -> List[str]:
        pose_msg = self.latest_person_pose
        persons_count = len(pose_msg.persons) if pose_msg is not None else "N/A"
        lock_id = pose_msg.lock_id if pose_msg is not None else "N/A"
        lock_state = lock_state_name(pose_msg.lock_state) if pose_msg is not None else "N/A"
        detections = self._diag_int("perception", "last_detection_count")
        tracks = self._diag_int("perception", "active_tracks")
        return [
            f"person_pose: hz={fmt_hz(self.person_pose.hz(now))}  age={fmt_age(self.person_pose.age(now))}",
            f"persons={persons_count}  lock={lock_id} / {lock_state}",
            f"detections={detections if detections is not None else 'N/A'}  tracks={tracks if tracks is not None else 'N/A'}",
            f"yolo_ready={bool_mark(self._diag_bool('perception', 'yolo_ready'))}  reid_ready={bool_mark(self._diag_bool('perception', 'reid_ready'))}",
            f"last total/yolo/reid={fmt_float(self._diag_float('perception', 'profile_last_total_ms'), 1, 'ms')} / {fmt_float(self._diag_float('perception', 'profile_last_yolo_ms'), 1, 'ms')} / {fmt_float(self._diag_float('perception', 'profile_last_reid_ms'), 1, 'ms')}",
            f"avg total/yolo/reid={fmt_float(self._diag_float('perception', 'profile_avg_total_ms'), 1, 'ms')} / {fmt_float(self._diag_float('perception', 'profile_avg_yolo_ms'), 1, 'ms')} / {fmt_float(self._diag_float('perception', 'profile_avg_reid_ms'), 1, 'ms')}",
            f"depth pos={fmt_float(self._diag_float('perception', 'depth_position_ms'), 1, 'ms')}  last_depth={fmt_float(self._diag_float('perception', 'last_valid_depth_m'), 3, 'm')}",
            f"skip_n={self.perception_params.get('process_every_n_frames', 'N/A')}  yolo_thr={self.perception_params.get('yolo.ort.intra_op_num_threads', 'N/A')}",
            f"yolo_in={self.perception_params.get('yolo.input_w', 'N/A')}x{self.perception_params.get('yolo.input_h', 'N/A')}",
        ]

    def _build_follower_lines(self, now: float) -> List[str]:
        return [
            f"cmd_vel_follow: hz={fmt_hz(self.follow_cmd.topic.hz(now))}  age={fmt_age(self.follow_cmd.topic.age(now))}",
            f"follow cmd: {fmt_xy(self.follow_cmd.linear_x, self.follow_cmd.angular_z)}",
            f"target_valid={bool_mark(self._diag_bool('follower', 'target_valid'))}  target_seen={bool_mark(self._diag_bool('follower', 'target_seen'))}",
            f"target_age={fmt_age(self._diag_float('follower', 'target_age_s'))}  pred_age={fmt_age(self._diag_float('follower', 'prediction_age_s'))}",
            f"target_vx={fmt_float(self._diag_float('follower', 'target_vx'), 3)}  target_vy={fmt_float(self._diag_float('follower', 'target_vy'), 3)}",
            f"target_speed={fmt_float(self._diag_float('follower', 'target_speed_mps'), 3, 'm/s')}",
            f"predicted_target_valid={bool_mark(self._diag_bool('follower', 'predicted_target_valid'))}",
            f"target_distance={self.follower_params.get('target_distance', 'N/A')}  theta_deadzone={self.follower_params.get('theta_deadzone', 'N/A')}",
            f"timeout={self.follower_params.get('target_timeout', 'N/A')}  horizon={self.follower_params.get('prediction_horizon_s', 'N/A')}",
            f"limits v/w={self.follower_params.get('limits.v_max', 'N/A')} / {self.follower_params.get('limits.w_max', 'N/A')}",
        ]

    def _build_obstacle_lines(self, now: float) -> List[str]:
        return [
            f"left range: {fmt_float(self.left_range.range_m, 3, 'm')}  hz={fmt_hz(self.left_range.topic.hz(now))}",
            f"right range:{fmt_float(self.right_range.range_m, 3, 'm')}  hz={fmt_hz(self.right_range.topic.hz(now))}",
            f"cmd_vel_avoid: hz={fmt_hz(self.avoid_cmd.topic.hz(now))}  {fmt_xy(self.avoid_cmd.linear_x, self.avoid_cmd.angular_z)}",
            f"final /cmd_vel: hz={fmt_hz(self.final_cmd.topic.hz(now))}  {fmt_xy(self.final_cmd.linear_x, self.final_cmd.angular_z)}",
            f"arbiter mode={arbiter_mode_name(self._diag_int('arbiter', 'mode'))}",
            f"stop_latched={bool_mark(self._diag_bool('arbiter', 'stop_latched'))}  avoid_latched={bool_mark(self._diag_bool('arbiter', 'avoid_latched'))}",
            f"arbiter target_age={fmt_age(self._diag_float('arbiter', 'last_target_age_s'))}",
            f"avoid left/right={fmt_float(self._diag_float('avoidance', 'left_dist'), 3, 'm')} / {fmt_float(self._diag_float('avoidance', 'right_dist'), 3, 'm')}",
            f"avoid ages={fmt_age(self._diag_float('avoidance', 'left_age_s'))} / {fmt_age(self._diag_float('avoidance', 'right_age_s'))}",
            f"gpio_ok={bool_mark(self._diag_bool('ultrasonic', 'gpio_ok'))}  backend={self._diag_value('ultrasonic', 'gpio_backend') or 'N/A'}",
        ]

    def _render_row(
        self,
        panels: Sequence[Tuple[str, Sequence[str]]],
        width: int,
        gap: int,
    ) -> List[str]:
        wrapped = [self._wrap_lines(lines, width - 4) for _, lines in panels]
        inner_height = max(len(lines) for lines in wrapped)
        boxes = [
            self._render_box(title, lines, width, min_inner_height=inner_height)
            for (title, _), lines in zip(panels, wrapped)
        ]
        merged: List[str] = []
        gap_str = " " * gap
        for row in range(len(boxes[0])):
            merged.append(gap_str.join(box[row] for box in boxes))
        return merged

    def _render_box(
        self,
        title: str,
        lines: Sequence[str],
        width: int,
        min_inner_height: int = 0,
    ) -> List[str]:
        inner_width = max(10, width - 4)
        content = self._wrap_lines(lines, inner_width)
        while len(content) < min_inner_height:
            content.append("")
        title_text = f" {title} "
        title_bar = "+" + title_text + "-" * max(0, width - len(title_text) - 2) + "+"
        rows = [title_bar]
        for line in content:
            rows.append(f"| {line[:inner_width]:<{inner_width}} |")
        rows.append("+" + "-" * (width - 2) + "+")
        return rows

    @staticmethod
    def _wrap_lines(lines: Sequence[str], width: int) -> List[str]:
        wrapped: List[str] = []
        for line in lines:
            segments = textwrap.wrap(
                line or "",
                width=width,
                break_long_words=False,
                break_on_hyphens=False,
            )
            wrapped.extend(segments or [""])
        return wrapped

    def restore_terminal(self) -> None:
        if self.hidden_cursor:
            sys.stdout.write("\x1b[?25h\n")
            sys.stdout.flush()
            self.hidden_cursor = False


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2 Smart Follower live dashboard")
    parser.add_argument("--robot-ns", default="robot1", help="robot namespace, default: robot1")
    parser.add_argument("--refresh-hz", type=float, default=5.0, help="screen refresh rate")
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
    return parser


def finalize_topics(args: argparse.Namespace) -> argparse.Namespace:
    robot_ns = normalize_ns(args.robot_ns)
    args.person_pose_topic = args.person_pose_topic or join_topic(robot_ns, "person_pose")
    args.follow_cmd_topic = args.follow_cmd_topic or join_topic(robot_ns, "cmd_vel_follow")
    args.avoid_cmd_topic = args.avoid_cmd_topic or join_topic(robot_ns, "cmd_vel_avoid")
    args.left_range_topic = args.left_range_topic or join_topic(robot_ns, "left_ultrasonic/range")
    args.right_range_topic = args.right_range_topic or join_topic(robot_ns, "right_ultrasonic/range")
    return args


def main() -> int:
    parser = build_arg_parser()
    args = finalize_topics(parser.parse_args())
    rclpy.init()
    node = LiveDashboard(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
