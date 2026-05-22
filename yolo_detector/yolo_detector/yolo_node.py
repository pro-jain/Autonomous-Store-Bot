#!/usr/bin/env python3
"""
Semantic SLAM Detection Node with Object Tracking and Temporal Filtering

Features:
- YOLO v8 object detection
- Frame-based object tracking
- Confidence and distance-based filtering
- CSV logging of detected objects
- Saves screenshots for each logged detection
- ROS 2 integration with Camera + Odometry
"""

import math
import csv
import os
from datetime import datetime
from collections import defaultdict
from typing import Dict, List, Tuple, Set

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


# ============================================================
# TRACKED OBJECT CLASS
# ============================================================

class TrackedObject:

    def __init__(self, class_name: str, timestamp: float):
        self.class_name = class_name
        self.first_detection = timestamp
        self.last_detection = timestamp

        self.detection_count = 1

        self.positions: List[Tuple[float, float]] = []
        self.thetas: List[float] = []
        self.confidences: List[float] = []

        self.logged_to_csv = False

    def update(
        self,
        x: float,
        y: float,
        theta: float,
        confidence: float,
        timestamp: float
    ):
        self.last_detection = timestamp
        self.detection_count += 1

        self.positions.append((x, y))
        self.thetas.append(theta)
        self.confidences.append(confidence)

    def get_mean_position(self):

        if not self.positions:
            return (0.0, 0.0)

        mean_x = sum(p[0] for p in self.positions) / len(self.positions)
        mean_y = sum(p[1] for p in self.positions) / len(self.positions)

        return mean_x, mean_y

    def get_mean_theta(self):

        if not self.thetas:
            return 0.0

        sin_sum = sum(math.sin(t) for t in self.thetas)
        cos_sum = sum(math.cos(t) for t in self.thetas)

        return math.atan2(sin_sum, cos_sum)

    def get_mean_confidence(self):

        if not self.confidences:
            return 0.0

        return sum(self.confidences) / len(self.confidences)


# ============================================================
# MAIN NODE
# ============================================================

class SemanticSLAMDetector(Node):

    def __init__(self):

        super().__init__('semantic_slam_detector')

        self.bridge = CvBridge()

        # ====================================================
        # YOLO MODEL
        # ====================================================

        self.model = YOLO('yolov8m.pt')

        self.target_objects = [
            'tv',
            'laptop',
            'cell phone',
            'microwave'
        ]

        self.conf_threshold = 0.30

        # ====================================================
        # TRACKING PARAMETERS
        # ====================================================

        self.same_object_radius = 0.5
        self.temporal_window = 2.0
        self.min_detections = 1

        # ====================================================
        # TRACKING STATE
        # ====================================================

        self.frame_count = 0

        self.tracked_objects: Dict[str, Dict[int, TrackedObject]] = defaultdict(dict)

        self.next_object_id: Dict[str, int] = defaultdict(int)

        self.current_frame_detections: Dict[str, Set[int]] = defaultdict(set)

        # ====================================================
        # ROBOT POSE
        # ====================================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # ====================================================
        # CSV STORAGE
        # ====================================================

        self.csv_filename = self._create_csv_file()

        self.csv_writer = None
        self.csv_file = None

        self._init_csv_file()

        # ====================================================
        # SCREENSHOT STORAGE
        # ====================================================

        self.screenshot_dir = os.path.expanduser(
            "~/semantic_slam_logs/screenshots"
        )

        os.makedirs(self.screenshot_dir, exist_ok=True)

        # ====================================================
        # ROS SUBSCRIPTIONS
        # ====================================================

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_timer(10.0, self.log_statistics)

        self.get_logger().info(
            f"Semantic SLAM Detector initialized\n"
            f"CSV: {self.csv_filename}\n"
            f"Screenshots: {self.screenshot_dir}"
        )

    # ============================================================
    # CSV FUNCTIONS
    # ============================================================

    def _create_csv_file(self):

        output_dir = os.path.expanduser("~/semantic_slam_logs")

        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        return os.path.join(
            output_dir,
            f"detections_{timestamp}.csv"
        )

    def _init_csv_file(self):

        self.csv_file = open(self.csv_filename, 'w', newline='')

        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'timestamp',
                'object_id',
                'class_name',
                'robot_x',
                'robot_y',
                'robot_theta',
                'mean_confidence',
                'detection_count'
            ]
        )

        self.csv_writer.writeheader()

    def _log_detection_to_csv(
        self,
        object_id: int,
        tracked_obj: TrackedObject
    ):

        x, y = tracked_obj.get_mean_position()

        theta = tracked_obj.get_mean_theta()

        confidence = tracked_obj.get_mean_confidence()

        self.csv_writer.writerow({
            'timestamp': datetime.now().isoformat(),
            'object_id': object_id,
            'class_name': tracked_obj.class_name,
            'robot_x': f"{x:.3f}",
            'robot_y': f"{y:.3f}",
            'robot_theta': f"{theta:.3f}",
            'mean_confidence': f"{confidence:.3f}",
            'detection_count': tracked_obj.detection_count
        })

        self.csv_file.flush()

        self.get_logger().info(
            f"CSV logged: {tracked_obj.class_name}#{object_id}"
        )

    # ============================================================
    # SCREENSHOT FUNCTION
    # ============================================================

    def _save_detection_screenshot(
        self,
        frame,
        class_name: str,
        object_id: int,
        confidence: float
    ):

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        filename = (
            f"{class_name}_{object_id}_{timestamp}.jpg"
        )

        save_path = os.path.join(
            self.screenshot_dir,
            filename
        )

        display_frame = frame.copy()

        text = (
            f"{class_name}#{object_id} | "
            f"Conf={confidence:.2f} | "
            f"Robot=({self.robot_x:.2f}, {self.robot_y:.2f})"
        )

        cv2.putText(
            display_frame,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imwrite(save_path, display_frame)

        self.get_logger().info(
            f"Screenshot saved: {save_path}"
        )

    # ============================================================
    # ODOM CALLBACK
    # ============================================================

    def odom_callback(self, msg: Odometry):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            quat.w * quat.z +
            quat.x * quat.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            quat.y * quat.y +
            quat.z * quat.z
        )

        self.robot_theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ============================================================
    # OBJECT MATCHING
    # ============================================================

    def find_matching_object(
        self,
        class_name: str,
        x: float,
        y: float,
        timestamp: float
    ):

        matched_id = None

        min_distance = float('inf')

        for obj_id, tracked_obj in self.tracked_objects[class_name].items():

            time_diff = timestamp - tracked_obj.last_detection

            if time_diff > self.temporal_window:
                continue

            tx, ty = tracked_obj.get_mean_position()

            distance = math.sqrt(
                (x - tx) ** 2 +
                (y - ty) ** 2
            )

            if distance < self.same_object_radius:

                if distance < min_distance:
                    min_distance = distance
                    matched_id = obj_id

        if matched_id is not None:
            return False, matched_id

        return True, self.next_object_id[class_name]

    def is_duplicate_in_frame(
        self,
        class_name: str,
        obj_id: int
    ):

        return obj_id in self.current_frame_detections[class_name]

    # ============================================================
    # IMAGE CALLBACK
    # ============================================================

    def image_callback(self, msg: Image):

        self.frame_count += 1

        current_timestamp = float(self.frame_count)

        self.current_frame_detections.clear()

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        try:
            results = self.model(frame, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO error: {e}")
            return

        annotated_frame = results[0].plot()

        detected_objects_info = []

        for detection in results[0].boxes:

            confidence = float(detection.conf[0])

            if confidence < self.conf_threshold:
                continue

            class_id = int(detection.cls[0])

            class_name = self.model.names[class_id]

            if class_name not in self.target_objects:
                continue

            obj_x = self.robot_x
            obj_y = self.robot_y
            obj_theta = self.robot_theta

            is_new, object_id = self.find_matching_object(
                class_name,
                obj_x,
                obj_y,
                current_timestamp
            )

            if self.is_duplicate_in_frame(
                class_name,
                object_id
            ):
                continue

            self.current_frame_detections[class_name].add(
                object_id
            )

            if is_new:

                tracked_obj = TrackedObject(
                    class_name,
                    current_timestamp
                )

                tracked_obj.update(
                    obj_x,
                    obj_y,
                    obj_theta,
                    confidence,
                    current_timestamp
                )

                self.tracked_objects[class_name][object_id] = tracked_obj

                self.next_object_id[class_name] += 1

                status = "NEW"

            else:

                tracked_obj = self.tracked_objects[class_name][object_id]

                tracked_obj.update(
                    obj_x,
                    obj_y,
                    obj_theta,
                    confidence,
                    current_timestamp
                )

                if (
                    tracked_obj.detection_count >= self.min_detections
                    and not tracked_obj.logged_to_csv
                ):

                    self._log_detection_to_csv(
                        object_id,
                        tracked_obj
                    )

                    self._save_detection_screenshot(
                        annotated_frame,
                        class_name,
                        object_id,
                        confidence
                    )

                    tracked_obj.logged_to_csv = True

                status = "UPDATED"

            detected_objects_info.append({
                'class_name': class_name,
                'object_id': object_id,
                'confidence': confidence,
                'status': status
            })

            self.get_logger().info(
                f"[Frame {self.frame_count}] "
                f"{status} {class_name}#{object_id} "
                f"| Conf={confidence:.2f}"
            )

        self._draw_detections(
            annotated_frame,
            detected_objects_info
        )

        cv2.imshow(
            "Semantic SLAM Detection",
            annotated_frame
        )

        cv2.waitKey(1)

    # ============================================================
    # DRAW UI
    # ============================================================

    def _draw_detections(
        self,
        frame,
        detections_info
    ):

        y_offset = 30

        for info in detections_info:

            text = (
                f"{info['class_name']}#{info['object_id']} "
                f"({info['confidence']:.2f})"
            )

            cv2.putText(
                frame,
                text,
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            y_offset += 25

    # ============================================================
    # STATS
    # ============================================================

    def log_statistics(self):

        total = sum(
            len(v)
            for v in self.tracked_objects.values()
        )

        self.get_logger().info(
            f"Tracked objects: {total}"
        )

    # ============================================================
    # CLEANUP
    # ============================================================

    def destroy_node(self):

        if self.csv_file:
            self.csv_file.close()

        cv2.destroyAllWindows()

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SemanticSLAMDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
