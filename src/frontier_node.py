#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import math
import time
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Bool


class RobustFrontierDetector(object):
    """
    Robust Frontier Detector for large-scale outdoor exploration.
    This module fuses map-based frontier detection with semantic direction cues
    (target_theta + target_visible) to generate navigation goals.

    Key features:
    - Angle-constrained frontier selection
    - Virtual goal generation when the target is visible
    - Travel-distance lock to avoid frequent replanning
    - Frontier filtering with obstacle inflation and unknown-region rules
    """

    def __init__(self):
        rospy.init_node('robust_frontier_detector')

        # === Behavior toggles ===
        self.use_virtual_frontier = rospy.get_param('~use_virtual_frontier', True)
        # If True: only select frontiers when target becomes invisible
        self.use_visibility_gate = rospy.get_param('~use_visibility_gate', False)
        # If True: prevent goal switching until travel-distance threshold is reached
        self.use_travel_distance_lock = rospy.get_param('~use_travel_distance_lock', True)

        # === Frontier extraction parameters ===
        self.min_area_thresh = rospy.get_param('~min_area_thresh', 30)
        self.inflate_radius_cells = rospy.get_param('~inflate_radius_cells', 16)
        self.dilate_explored_cells = rospy.get_param('~dilate_explored_cells', 2)
        self.min_neighbors_unknown = rospy.get_param('~min_neighbors_unknown', 2)

        # Virtual goal distance when target is visible
        self.virtual_goal_dist = rospy.get_param('~virtual_goal_dist', 0.5)

        # Angular constraint for selecting "aligned" frontiers
        self.frontier_angle_threshold = rospy.get_param(
            '~frontier_angle_threshold_deg', 15.0
        ) * math.pi / 180.0

        # === Target visibility signals ===
        self.target_theta_in_map = None  # Target direction in map frame (radians)
        self.target_visible = False  # Whether the target is currently visible

        # === Robot pose tracking ===
        self.robot_x = 0.0
        self.robot_y = 0.0

        # === Travel-distance lock state ===
        self.start_x = None
        self.start_y = None
        self.accumulated_travel = 0.0
        self.expected_dist = None
        self.active_goal = None  # (x, y)

        # === Map metadata ===
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        # === ROS topics ===
        map_topic = rospy.get_param('~map_topic', '/map')
        odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        theta_topic = rospy.get_param('~target_theta_topic', '/target_theta')
        visible_topic = rospy.get_param('~target_visible_topic', '/target_visible')
        goal_topic = rospy.get_param('~goal_topic', '/best_frontier_pose')
        marker_topic = rospy.get_param('~marker_topic', '/frontiers_marker')
        self.frame_id = rospy.get_param('~frame_id', 'camera_init')

        # === Subscribers ===
        rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(theta_topic, Float32, self.target_theta_callback, queue_size=10)
        rospy.Subscriber(visible_topic, Bool, self.visible_callback, queue_size=10)

        # === Publishers ===
        self.pub_marker = rospy.Publisher(marker_topic, Marker, queue_size=10)
        self.pub_goal = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)

        rospy.loginfo("RobustFrontierDetector started.")

    # ----------------------------------------------------------
    # Callback functions
    # ----------------------------------------------------------

    def visible_callback(self, msg):
        """Receive target visibility (True/False)."""
        self.target_visible = bool(msg.data)

    def target_theta_callback(self, msg):
        """Receive target direction (theta) in map coordinate frame."""
        self.target_theta_in_map = msg.data

    def odom_callback(self, msg):
        """
        Track robot position and update travel-distance lock.
        The lock prevents switching goals too frequently.
        """
        pos = msg.pose.pose.position
        new_x, new_y = pos.x, pos.y

        # Update travel-distance lock
        if self.active_goal and self.use_travel_distance_lock and self.start_x is not None:
            delta = math.hypot(new_x - self.start_x, new_y - self.start_y)
            self.accumulated_travel += delta
            self.start_x = new_x
            self.start_y = new_y

            if self.expected_dist is not None and self.accumulated_travel >= self.expected_dist:
                rospy.loginfo("Travel-distance lock released.")
                self.active_goal = None
                self.start_x = None
                self.expected_dist = None

        self.robot_x, self.robot_y = new_x, new_y

        # Clear goal once robot reaches the goal region
        if self.active_goal:
            dx = self.active_goal[0] - self.robot_x
            dy = self.active_goal[1] - self.robot_y
            if math.hypot(dx, dy) < 0.8:
                rospy.loginfo("Reached active goal. Clearing goal.")
                self.active_goal = None

    # ----------------------------------------------------------
    # Map processing and frontier extraction
    # ----------------------------------------------------------

    def map_callback(self, msg):
        """
        Main frontier-detection pipeline.
        Runs obstacle inflation, unknown-neighbor filtering,
        connected-component extraction, and final goal selection.
        """
        start_time = time.perf_counter()
        map_stamp = msg.header.stamp

        # Update map metadata
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Convert map to numpy array
        width = msg.info.width
        height = msg.info.height
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))

        # === Generate obstacle/free/unknown masks ===
        obstacle = (data == 100).astype(np.uint8)
        free = (data == 0).astype(np.uint8)
        unknown = (data == -1).astype(np.uint8)

        # === Inflate obstacles to avoid narrow passages ===
        k = self.inflate_radius_cells * 2 + 1
        inflated = cv2.dilate(obstacle, np.ones((k, k), np.uint8), iterations=2)
        navigable = cv2.bitwise_and(free, (1 - inflated))

        # === Dilate explored region (helps reduce noise) ===
        explored = (1 - unknown).astype(np.uint8)
        k2 = self.dilate_explored_cells * 2 + 1
        cv2.dilate(explored, np.ones((k2, k2), np.uint8))

        # === Detect free cells touching unknown cells (frontier rule) ===
        kernel = np.ones((3, 3), np.uint8)
        unknown_neighbors = cv2.filter2D(unknown, -1, kernel)

        frontier_binary = np.where(
            (navigable == 1) & (unknown_neighbors >= self.min_neighbors_unknown),
            255,
            0
        ).astype(np.uint8)

        # Log timing
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        rospy.loginfo("Frontier detection time: %.2f ms" % elapsed_ms)

        # === Connected component filtering ===
        num_labels, labels_im = cv2.connectedComponents(frontier_binary)
        frontier_points = []

        for label in range(1, num_labels):
            ys, xs = np.where(labels_im == label)
            if xs.size < self.min_area_thresh:
                continue

            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            wx = cx * self.resolution + self.origin_x
            wy = cy * self.resolution + self.origin_y
            frontier_points.append((wx, wy))

        # Publish all frontier points as markers
        self.publish_all_frontiers_marker(frontier_points, map_stamp)

        # If a goal is locked, do not replan
        if self.active_goal:
            self.publish_marker(self.active_goal, map_stamp)
            return

        # === Visibility-gated mode ===
        if self.use_visibility_gate and self.target_theta_in_map is not None:
            rospy.loginfo("Visibility gate enabled. Selecting frontier.")
            self.select_and_publish_frontier(frontier_points, map_stamp)
            return

        # === Free mode (virtual goal + fallback frontier) ===
        if not self.use_visibility_gate:
            if self.target_visible and self.target_theta_in_map is not None and self.use_virtual_frontier:
                # Generate a virtual goal along viewing direction
                wx = self.robot_x + math.cos(self.target_theta_in_map) * self.virtual_goal_dist
                wy = self.robot_y + math.sin(self.target_theta_in_map) * self.virtual_goal_dist
                self.active_goal = (wx, wy, map_stamp)
                self.publish_goal(wx, wy, map_stamp)
                self.publish_marker((wx, wy), map_stamp)
                rospy.loginfo("Target visible. Publishing virtual goal.")
                return

            if (not self.target_visible) and self.target_theta_in_map is not None:
                rospy.loginfo("Target not visible. Selecting frontier.")
                self.select_and_publish_frontier(frontier_points, map_stamp)
                return

    # ----------------------------------------------------------
    # Frontier selection logic
    # ----------------------------------------------------------

    def select_and_publish_frontier(self, frontier_points, stamp):
        """
        Select a frontier that aligns with the target direction
        within a specified angular threshold.
        """
        if self.target_theta_in_map is None or len(frontier_points) == 0:
            rospy.loginfo("No valid frontier or missing target angle.")
            return

        best = None
        min_dist = float('inf')

        for wx, wy in frontier_points:
            theta_f = math.atan2(wy - self.robot_y, wx - self.robot_x)
            # Normalize angle difference
            d_theta = (theta_f - self.target_theta_in_map + math.pi) % (2 * math.pi) - math.pi
            if abs(d_theta) < self.frontier_angle_threshold:
                dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if dist < min_dist:
                    min_dist = dist
                    best = (wx, wy)

        if best:
            self.active_goal = best
            self.publish_goal(best[0], best[1], stamp)
            self.publish_marker(best, stamp)
            rospy.loginfo("Publishing frontier goal.")
        else:
            rospy.loginfo("No frontier satisfies angle constraint.")

    # ----------------------------------------------------------
    # Publishing helpers
    # ----------------------------------------------------------

    def publish_goal(self, x, y, stamp):
        """Publish a navigation goal as PoseStamped."""
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)

        # Initialize travel-distance lock
        if self.use_travel_distance_lock:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.accumulated_travel = 0.0
            self.expected_dist = math.hypot(x - self.robot_x, y - self.robot_y)

    def publish_marker(self, goal, stamp):
        """Publish a green sphere marker indicating the active goal."""
        x, y = goal
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        self.pub_marker.publish(marker)

    def publish_all_frontiers_marker(self, points, stamp):
        """Publish all detected frontiers as blue points."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "all_frontiers"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in points]
        self.pub_marker.publish(marker)


def main():
    node = RobustFrontierDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
