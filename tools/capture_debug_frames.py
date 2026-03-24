#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from smart_follower_msgs.msg import PersonPoseArray, TrackedPerson


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


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


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8"):
        raise RuntimeError(f"unsupported image encoding: {msg.encoding}")
    flat = np.frombuffer(msg.data, dtype=np.uint8)
    row_stride = int(msg.step)
    expected_cols = int(msg.width) * 3
    image_2d = flat.reshape((int(msg.height), row_stride))
    bgr = image_2d[:, :expected_cols].reshape((int(msg.height), int(msg.width), 3)).copy()
    if msg.encoding == "rgb8":
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
    return bgr


def clip_roi(person: TrackedPerson, width: int, height: int) -> Optional[Tuple[int, int, int, int]]:
    x0 = max(0, int(person.bbox.x_offset))
    y0 = max(0, int(person.bbox.y_offset))
    x1 = min(width, x0 + int(person.bbox.width))
    y1 = min(height, y0 + int(person.bbox.height))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def build_yolo_overlay(image: np.ndarray, msg: PersonPoseArray) -> np.ndarray:
    canvas = image.copy()
    for person in msg.persons:
        roi = clip_roi(person, canvas.shape[1], canvas.shape[0])
        if roi is None:
            continue
        x0, y0, x1, y1 = roi
        is_locked = person.track_id == msg.lock_id and msg.lock_state == PersonPoseArray.LOCKED
        color = (0, 255, 0) if is_locked else (0, 200, 255)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2)
        label = f"id={person.track_id} conf={person.confidence:.2f} {track_state_name(person.track_state)}"
        cv2.putText(canvas, label, (x0, max(20, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
    header = f"YOLO/tracking boxes persons={len(msg.persons)} lock={msg.lock_id} {lock_state_name(msg.lock_state)}"
    cv2.putText(canvas, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def build_reid_montage(image: np.ndarray, msg: PersonPoseArray) -> np.ndarray:
    tile_w = 128
    tile_h = 256
    header_h = 64
    persons = list(msg.persons)
    if not persons:
        canvas = np.zeros((header_h + tile_h, tile_w, 3), dtype=np.uint8)
        cv2.putText(canvas, "No person for ReID", (8, header_h + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        return canvas

    tiles = []
    for person in persons:
        roi = clip_roi(person, image.shape[1], image.shape[0])
        if roi is None:
            crop = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
        else:
            x0, y0, x1, y1 = roi
            crop = image[y0:y1, x0:x1]
            crop = cv2.resize(crop, (tile_w, tile_h), interpolation=cv2.INTER_LINEAR)
        is_locked = person.track_id == msg.lock_id and msg.lock_state == PersonPoseArray.LOCKED
        color = (0, 255, 0) if is_locked else (0, 200, 255)
        cv2.rectangle(crop, (0, 0), (tile_w - 1, tile_h - 1), color, 2)
        feature = np.asarray(person.appearance_feature, dtype=np.float32)
        norm = float(np.linalg.norm(feature))
        lines = [
            f"id={person.track_id}",
            f"conf={person.confidence:.2f}",
            f"state={track_state_name(person.track_state)}",
            f"feat_dim={feature.size}",
            f"feat_norm={norm:.2f}",
        ]
        y = 22
        for line in lines:
            cv2.putText(crop, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
            y += 24
        tiles.append(crop)

    canvas = np.zeros((header_h + tile_h, tile_w * len(tiles), 3), dtype=np.uint8)
    canvas[header_h:, :, :] = np.hstack(tiles)
    title = f"ReID crops persons={len(persons)} lock={msg.lock_id} {lock_state_name(msg.lock_state)}"
    cv2.putText(canvas, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "Each crop is the bbox patch sent into the ReID/tracking chain.", (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


class CaptureNode(Node):
    def __init__(self, output_dir: Path, max_delta_ms: float, timeout_s: float) -> None:
        super().__init__("capture_debug_frames")
        self.output_dir = output_dir
        self.max_delta_ns = int(max_delta_ms * 1_000_000.0)
        self.timeout_s = timeout_s
        self.start_time = self.get_clock().now()
        self.images: Deque[Tuple[int, np.ndarray]] = deque(maxlen=90)
        self.done = False

        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, 10)
        self.create_subscription(PersonPoseArray, "/robot1/person_pose", self.on_person_pose, 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info("Waiting for matching /camera/color/image_raw and /robot1/person_pose ...")

    def on_image(self, msg: Image) -> None:
        if self.done:
            return
        try:
            image = image_msg_to_bgr(msg)
        except Exception as ex:
            self.get_logger().warning(f"Failed to decode image: {ex}")
            return
        self.images.append((stamp_to_ns(msg.header.stamp), image))

    def find_best_image(self, target_ns: int) -> Optional[np.ndarray]:
        best: Optional[np.ndarray] = None
        best_delta: Optional[int] = None
        for stamp_ns, image in self.images:
            delta = abs(stamp_ns - target_ns)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best = image
        if best is None or best_delta is None or best_delta > self.max_delta_ns:
            return None
        return best.copy()

    def on_person_pose(self, msg: PersonPoseArray) -> None:
        if self.done or not msg.persons:
            return
        target_ns = stamp_to_ns(msg.header.stamp)
        image = self.find_best_image(target_ns)
        if image is None:
            return

        raw_path = self.output_dir / "robot_camera_input.png"
        yolo_path = self.output_dir / "robot_yolo_result.png"
        reid_path = self.output_dir / "robot_reid_result.png"

        cv2.imwrite(str(raw_path), image)
        cv2.imwrite(str(yolo_path), build_yolo_overlay(image, msg))
        cv2.imwrite(str(reid_path), build_reid_montage(image, msg))
        self.get_logger().info(f"Saved camera input to {raw_path}")
        self.get_logger().info(f"Saved YOLO result to {yolo_path}")
        self.get_logger().info(f"Saved ReID result to {reid_path}")
        self.done = True

    def on_timer(self) -> None:
        if self.done:
            rclpy.shutdown()
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout_s:
            self.get_logger().error("Timed out waiting for a matched non-empty person_pose frame.")
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=".", help="Directory to save output images.")
    parser.add_argument("--max-delta-ms", type=float, default=200.0, help="Maximum allowed stamp mismatch.")
    parser.add_argument("--timeout-s", type=float, default=20.0, help="Timeout while waiting for data.")
    args = parser.parse_args()

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CaptureNode(output_dir=output_dir, max_delta_ms=args.max_delta_ms, timeout_s=args.timeout_s)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
