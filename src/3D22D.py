#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import tf.transformations as tft
from math import hypot

class EgoGridMapper(object):
    def __init__(self):
        rospy.init_node("pc2_to_ego_map")

        # Parameters
        self.pc_topic = rospy.get_param("~pc_topic", "/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.map_frame = rospy.get_param("~map_frame", "camera_init")
        self.res = rospy.get_param("~resolution", 0.1)
        self.width = rospy.get_param("~width", 500)
        self.height = rospy.get_param("~height", 500)
        self.z_min = rospy.get_param("~z_min", -1.0)
        self.z_max = rospy.get_param("~z_max", 2.0)
        self.r_max = rospy.get_param("~range_max", 100.0)
        self.downsample = rospy.get_param("~downsample", 1)
        self.use_tf = rospy.get_param("~use_tf", False)
        self.odom_frame = rospy.get_param("~odom_frame", "camera_init")
        self.robot_radius_cells = rospy.get_param("~robot_radius_cells", 1)
        
        # Ring filter parameters
        self.r_min = rospy.get_param("~range_min", 0.8)
        self.theta0_deg = rospy.get_param("~ring_center_deg", 0.0)
        self.delta_deg = rospy.get_param("~ring_half_width_deg", 1.5)

        # Publishers/Subscribers
        self.pub_map = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1)
        self.sub_pc = rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)

        # TF
        self.listener = tf.TransformListener() if self.use_tf else None

        # State
        self.has_odom = False
        self.odom_xyyaw = (0.0, 0.0, 0.0)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.last_sensor_stamp = rospy.Time(0)

        rospy.loginfo("EgoGridMapper started: %dx%d @ %.2fm", self.width, self.height, self.res)

    def odom_cb(self, msg):
        rospy.loginfo_throttle(1.0, "Received odometry")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_xyyaw = (x, y, yaw)
        self.has_odom = True

    def pc_cb(self, msg):
        self.last_sensor_stamp = msg.header.stamp
        
        if not self.has_odom:
            return

        x, y, yaw = self.odom_xyyaw
        half_w = (self.width * self.res) * 0.5
        half_h = (self.height * self.res) * 0.5
        origin_x = x - half_w
        origin_y = y - half_h

        # Reset grid
        self.grid.fill(-1)

        # Read point cloud
        points_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Get transform if using TF
        T = None
        if self.use_tf and self.listener is not None:
            try:
                self.listener.waitForTransform(self.odom_frame, msg.header.frame_id,
                                              msg.header.stamp, rospy.Duration(0.05))
                (trans, rot) = self.listener.lookupTransform(self.odom_frame,
                                                             msg.header.frame_id,
                                                             msg.header.stamp)
                T = tft.concatenate_matrices(tft.translation_matrix(trans),
                                             tft.quaternion_matrix(rot))
            except tf.Exception:
                T = None

        # Robot center in grid coordinates
        robot_cx = int((x - origin_x) / self.res)
        robot_cy = int((y - origin_y) / self.res)

        if not (0 <= robot_cx < self.width and 0 <= robot_cy < self.height):
            rospy.logwarn_throttle(1.0, "Robot outside grid bounds")
            return

        # Process point cloud
        i = 0
        for px, py, pz in points_iter:
            i += 1
            if self.downsample > 1 and (i % self.downsample != 0):
                continue

            # Ring filter
            rho = (px * px + py * py) ** 0.5
            if rho < 1e-6:
                continue
                
            theta = np.arctan2(pz, rho)
            theta0 = np.deg2rad(self.theta0_deg)
            if abs(theta - theta0) > np.deg2rad(self.delta_deg):
                continue

            # Range filter
            r = hypot(px - x, py - y)
            if r < self.r_min or r > self.r_max:
                continue

            # Transform to odom frame if using TF
            if T is not None:
                P = np.array([px, py, pz, 1.0])
                P_odom = T.dot(P)
                px, py, pz = P_odom[0], P_odom[1], P_odom[2]

            # Convert to grid coordinates
            gx = int((px - origin_x) / self.res)
            gy = int((py - origin_y) / self.res)
            
            if not (0 <= gx < self.width and 0 <= gy < self.height):
                continue

            # Mark free space along ray
            self._mark_ray_free(robot_cx, robot_cy, gx, gy)
            
            # Mark endpoint as occupied
            self.grid[gy, gx] = 100

        # Publish map
        self._publish_grid(origin_x, origin_y)

    def _publish_grid(self, origin_x, origin_y):
        msg = OccupancyGrid()
        msg.header.stamp = self.last_sensor_stamp
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.pub_map.publish(msg)

    def _mark_ray_free(self, x0, y0, x1, y1):
        """Mark free space along ray from (x0,y0) to (x1,y1) using Bresenham's algorithm."""
        # Adjust start point to avoid robot body
        if self.robot_radius_cells > 0:
            dx = x1 - x0
            dy = y1 - y0
            L = max(abs(dx), abs(dy))
            if L == 0:
                return
            stepx = dx / float(L)
            stepy = dy / float(L)
            x0 = int(round(x0 + stepx * self.robot_radius_cells))
            y0 = int(round(y0 + stepy * self.robot_radius_cells))

        # Bresenham's line algorithm
        x, y = x0, y0
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if not (0 <= x < self.width and 0 <= y < self.height):
                break
            if x == x1 and y == y1:
                break
                
            if self.grid[y, x] == -1:
                self.grid[y, x] = 0

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

def main():
    EgoGridMapper()
    rospy.spin()

if __name__ == "__main__":
    main()
