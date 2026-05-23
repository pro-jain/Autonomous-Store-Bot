#!/usr/bin/env python3
"""
Semantic Navigator — goto_object.py
====================================
Reads the nearest detected object from the CSV database (produced by yolo_node.py),
then navigates to it using Nav2's NavigateToPose action (AMCL localisation +
NavFn/A* global planner + DWB/RPP local planner with obstacle avoidance).

How it works
------------
1. Loads the YOLO CSV detections log.
2. Subscribes to /odom to know the current robot position.
3. Waits for Nav2 (NavigateToPose action server) to become available.
4. Finds the closest instance of `target_object` in the map frame.
5. Sends a NavigateToPose goal — Nav2 handles:
     • Global path  → NavFn planner (Dijkstra or A*) on the static map
     • Local path   → Regulated Pure Pursuit controller
     • Obstacle avoidance → local + global costmaps with inflation layers
     • Recovery behaviours → spin / back-up / clear costmaps
6. Monitors feedback (distance remaining) and logs the final result.

Usage
-----
# In one terminal — bring up Gazebo + Nav2 + AMCL:
ros2 launch mobile_dd_robot navigation.launch.py

# In another terminal — run this node:
ros2 run mobile_dd_robot goto_object

# Override target object at runtime:
ros2 run mobile_dd_robot goto_object --ros-args -p target_object:=microwave
"""

import math
import time

import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Nav2 action — gives us full planning + obstacle avoidance
from nav2_msgs.action import NavigateToPose

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT SETTINGS  (override via ROS 2 parameters)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_CSV_PATH    = "/home/bodhini/semantic_slam_logs/detections_20260520_131730.csv"
DEFAULT_TARGET      = "laptop"
GOAL_DELAY_SEC      = 3.0   # seconds to wait before sending the first goal
NAV_TIMEOUT_SEC     = 120.0 # abort if navigation takes longer than this


class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')

        # ── ROS 2 parameters ─────────────────────────────────────────────────
        self.declare_parameter('csv_path',     DEFAULT_CSV_PATH)
        self.declare_parameter('target_object', DEFAULT_TARGET)

        self.csv_path      = self.get_parameter('csv_path').value
        self.target_object = self.get_parameter('target_object').value

        self.get_logger().info(
            f"SemanticNavigator started | target='{self.target_object}' | "
            f"csv='{self.csv_path}'"
        )

        # ── Robot pose (from odometry) ────────────────────────────────────────
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False

        # ── Load detections CSV ───────────────────────────────────────────────
        self.database = self._load_database(self.csv_path)

        # ── Odometry subscriber ───────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )

        # ── Nav2 NavigateToPose action client ─────────────────────────────────
        # This single action replaces the simple /goal_pose publisher and gives
        # us:  planning + obstacle avoidance + recovery + feedback + result.
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Start navigation after a short delay ──────────────────────────────
        self._nav_timer = self.create_timer(GOAL_DELAY_SEC, self._start_navigation)

        self.get_logger().info(
            f"Waiting {GOAL_DELAY_SEC}s then navigating to nearest '{self.target_object}' ..."
        )

    # ═════════════════════════════════════════════════════════════════════════
    # DATABASE HELPERS
    # ═════════════════════════════════════════════════════════════════════════

    def _load_database(self, path: str) -> pd.DataFrame:
        """Load the YOLO detection CSV. Exits if the file is missing."""
        try:
            df = pd.read_csv(path)
            required = {'class_name', 'x_position', 'y_position'}
            missing  = required - set(df.columns)
            if missing:
                self.get_logger().error(
                    f"CSV is missing columns: {missing}. "
                    f"Available: {list(df.columns)}"
                )
                raise RuntimeError("Bad CSV schema")
            self.get_logger().info(
                f"Loaded {len(df)} detections from '{path}'"
            )
            return df
        except FileNotFoundError:
            self.get_logger().error(f"CSV not found: {path}")
            raise

    def _find_nearest_object(self):
        """Return (x, y) of the closest instance of target_object, or (None, None)."""
        candidates = self.database[
            self.database['class_name'] == self.target_object
        ]

        if candidates.empty:
            return None, None

        best_dist = float('inf')
        best_x = best_y = None

        for _, row in candidates.iterrows():
            ox, oy = float(row['x_position']), float(row['y_position'])
            d = math.hypot(ox - self.robot_x, oy - self.robot_y)
            if d < best_dist:
                best_dist, best_x, best_y = d, ox, oy

        self.get_logger().info(
            f"Nearest '{self.target_object}' is at ({best_x:.2f}, {best_y:.2f}) "
            f"— {best_dist:.2f} m away"
        )
        return best_x, best_y

    # ═════════════════════════════════════════════════════════════════════════
    # ODOMETRY CALLBACK
    # ═════════════════════════════════════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True

    # ═════════════════════════════════════════════════════════════════════════
    # NAVIGATION
    # ═════════════════════════════════════════════════════════════════════════

    def _start_navigation(self):
        """Called once after the startup delay."""
        # Cancel the one-shot timer immediately
        self._nav_timer.cancel()

        # ── Wait until Nav2 action server is up ──────────────────────────────
        self.get_logger().info("Waiting for Nav2 'navigate_to_pose' action server ...")
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                "navigate_to_pose action server not available after 30 s. "
                "Is navigation.launch.py running?"
            )
            return

        # ── Find goal ─────────────────────────────────────────────────────────
        gx, gy = self._find_nearest_object()
        if gx is None:
            self.get_logger().warn(
                f"No '{self.target_object}' found in the database. Aborting."
            )
            return

        # ── Build NavigateToPose goal ─────────────────────────────────────────
        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id    = 'map'
        goal_pose.header.stamp       = self.get_clock().now().to_msg()
        goal_pose.pose.position.x    = gx
        goal_pose.pose.position.y    = gy
        goal_pose.pose.position.z    = 0.0
        # Face forward (no rotation) — AMCL will handle localisation
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        goal_msg.pose = goal_pose
        # behaviour_tree = '' means use the default BT (replanning + recovery)

        self.get_logger().info(
            f"Sending NavigateToPose goal → ({gx:.2f}, {gy:.2f}) in 'map' frame"
        )

        # ── Send goal asynchronously ──────────────────────────────────────────
        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Called when Nav2 accepts (or rejects) the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was REJECTED by Nav2.")
            return

        self.get_logger().info("Goal ACCEPTED by Nav2 — robot is navigating ...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Periodic feedback from Nav2 — logs distance remaining."""
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"[Nav2 feedback] Distance remaining: {dist:.2f} m", throttle_duration_sec=2.0
        )

    def _result_callback(self, future):
        """Called when navigation finishes (success, failure, or cancelled)."""
        result = future.result()

        # nav2_msgs/action/NavigateToPose result has no payload — status tells all
        status = result.status   # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED

        STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        status_str   = STATUS_NAMES.get(status, f"UNKNOWN({status})")

        if status == 4:
            self.get_logger().info(
                f"Navigation {status_str}! "
                f"Reached '{self.target_object}' at "
                f"({self.robot_x:.2f}, {self.robot_y:.2f})"
            )
        else:
            self.get_logger().error(
                f"Navigation {status_str}. "
                "Check Nav2 logs for costmap / planner errors."
            )

        # Optionally shut down after reaching the goal
        # rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
