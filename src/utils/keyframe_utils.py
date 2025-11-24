import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))
import math
from marker_utils import publish_keyframe_marker
from marker_utils import publish_keyframe_marker
class KeyframeManager:
    def __init__(self, node):
        self.node = node  # ROS Node

    def add_keyframe(self, embedding, theta, x, y):
        self.node.keyframes.append((embedding, theta, x, y))

        publish_keyframe_marker(self.node.pub_marker, x, y, theta, len(self.node.keyframes))

#  这个功能允许机器人在被遮挡的时候随着移动补偿目标的方向（测试版本）
#  This function is still under development, and is not relate to the paper

    def add_keyframe_dynamic(self, embedding, old_theta, old_x, old_y, current_yaw, assumed_distance):
        dx = math.cos(old_theta - current_yaw)
        dy = math.sin(old_theta - current_yaw)
        R0 = [[math.cos(old_theta), -math.sin(old_theta)],
              [math.sin(old_theta), math.cos(old_theta)]]
        R1 = [[math.cos(current_yaw), -math.sin(current_yaw)],
              [math.sin(current_yaw), math.cos(current_yaw)]]

        d_world = [R0[0][0]*dx + R0[0][1]*dy, R0[1][0]*dx + R0[1][1]*dy]
        d1_x = R1[0][0]*d_world[0] + R1[0][1]*d_world[1]
        d1_y = R1[1][0]*d_world[0] + R1[1][1]*d_world[1]

        norm = math.sqrt(d1_x**2 + d1_y**2) + 1e-6
        d1_x /= norm
        d1_y /= norm

        cx = self.node.current_x
        cy = self.node.current_y
        gx = cx + d1_x * assumed_distance
        gy = cy + d1_y * assumed_distance
        theta = math.atan2(gy - cy, gx - cx)

        self.node.keyframes.append((embedding, theta, gx, gy))

        publish_keyframe_marker(self.node.pub_marker, cx, cy, theta, len(self.node.keyframes))
