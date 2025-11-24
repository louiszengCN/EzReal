#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, rospkg
# Dynamically add package path to import local utils
pkg_path = rospkg.RosPack().get_path('zson_nav_tools')
sys.path.insert(0, os.path.join(pkg_path, 'scripts'))

import rospy
import numpy as np
import torch
import math
import cv2

from collections import deque
from PIL import Image as PILImage
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker

# Local utility imports (tile splitting, CLIP embeddings, angle mapping, markers, keyframes)
from image_utils import split_image_into_tiles, split_image_into_tiles_grid, draw_scores
from embedding_utils import load_clip_model, compute_tile_embeddings, compute_prompt_scores
from angle_utils import map_tile_to_cam_angles, euler_from_quaternion, average_angles
from marker_utils import publish_marker
from keyframe_utils import KeyframeManager


class TileGlobalDirectionNode(object):
    """
    Tile-based Global Direction Estimator (with keyframe fallback).

    This ROS node:
    - Splits RGB images into tiles (multi-scale optional)
    - Computes CLIP embeddings for each tile
    - Determines which tile is most likely to contain the target
    - Converts tile location → camera angle → global map angle
    - Determines visibility using a saliency-ratio heuristic
    - Publishes target direction (theta) in map frame
    - Falls back to keyframe memory when target becomes invisible
    - Optionally saves visualization video
    """

    def __init__(self):
        rospy.init_node('tile_global_direction_node_with_keyframe_fallback')

        # Visualization & logging options
        self.show_window = rospy.get_param('~show_window', True)
        self.save_visualization = rospy.get_param('~save_visualization', False)
        self.video_output_path = rospy.get_param('~video_output_path', 'tile_vis.mp4')
        self.video_fps = float(rospy.get_param('~video_fps', 30.0))
        self.video_codec = rospy.get_param('~video_codec', 'mp4v')
        self.video_writer = None

        rospy.on_shutdown(self._cleanup_video_writer)

        self.bridge = CvBridge()

        # Tile grid (e.g., [2,3])
        self.grid = tuple(rospy.get_param('~grid', [2, 3]))

        # Target description & image processing
        self.target_text = rospy.get_param('~target_text', ["a red building"])
        self.target_size = tuple(rospy.get_param('~target_size', [640, 480]))
        self.use_multiscale = rospy.get_param('~use_multiscale', True)
        self.use_threshold = rospy.get_param('~use_threshold', False)

        # Keyframe-based fallback settings
        self.dynamic_add_keyframe = rospy.get_param('~dynamic_add_keyframe', False)
        self.assumed_target_distance = rospy.get_param('~assumed_target_distance', 5.0)
        self.max_keyframe_compare = rospy.get_param('~max_keyframe_compare', 5)

        # Multi-scale saliency fusion parameters
        self.saliency_base_4x6_to_2x3 = rospy.get_param('~saliency_base_4x6_to_2x3', 1.5)
        self.saliency_base_8x12_to_4x6 = rospy.get_param('~saliency_base_8x12_to_4x6', 1.5)

        # ROS topics
        self.image_topic = rospy.get_param('~image_topic', '/zed2/zed_node/rgb/image_rect_color')
        self.caminfo_topic = rospy.get_param('~camera_info_topic', '/zed2/zed_node/rgb/camera_info')
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.frontier_pose_topic = rospy.get_param('~frontier_pose_topic', '/best_frontier_pose')

        self.theta_topic = rospy.get_param('~theta_topic', '/target_theta')
        self.visible_topic = rospy.get_param('~visible_topic', '/target_visible')
        self.tile_marker_topic = rospy.get_param('~tile_marker_topic', '/tile_target_marker')

        # Load CLIP model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = load_clip_model(self.device)

        # Optional: load target image embedding for additional supervision
        self.target_image_path = rospy.get_param('~target_image_path', None)
        self.target_image_embedding = self._load_target_image_embedding()

        # Camera information
        self.fx = None
        self.fy = None
        self.width = None
        self.height = None
        self.h_fov = None  # horizontal FOV

        # Robot pose
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # Current best embedding & angle
        self.latest_best_embedding = None
        self.latest_theta_in_map = None

        # Keyframe memory (FIFO queue)
        self.keyframes = deque(maxlen=50)

        # Publishers
        self.pub_theta = rospy.Publisher(self.theta_topic, Float32, queue_size=10)
        self.pub_marker = rospy.Publisher(self.tile_marker_topic, Marker, queue_size=10)
        self.pub_visible = rospy.Publisher(self.visible_topic, Bool, queue_size=10)

        # Subscribers
        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.sub_caminfo = rospy.Subscriber(self.caminfo_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        self.sub_frontier = rospy.Subscriber(self.frontier_pose_topic, PoseStamped, self.frontier_callback, queue_size=10)

        # Keyframe manager helper
        self.keyframe_manager = KeyframeManager(self)

    # ---------------------------------------------------------
    # Camera Info Callback
    # ---------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        """
        Receive camera intrinsics and compute horizontal FOV.
        """
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.width = msg.width
        self.height = msg.height
        self.h_fov = 2.0 * math.atan2(self.width, 2.0 * self.fx)

    # ---------------------------------------------------------
    # Odometry Callback
    # ---------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """
        Update robot global pose (x, y, yaw).
        """
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        self.current_x = pos.x
        self.current_y = pos.y
        self.current_yaw = yaw

    # ---------------------------------------------------------
    # Frontier Callback
    # ---------------------------------------------------------
    def frontier_callback(self, msg: PoseStamped):
        """
        Every time a new frontier goal is generated:
        - Save recent best embedding + angle as a keyframe.
        - Used for fallback when target becomes invisible.
        """
        if self.latest_best_embedding is not None and self.latest_theta_in_map is not None:
            if self.dynamic_add_keyframe:
                # Store more detailed information (pose + assumed distance)
                self.keyframe_manager.add_keyframe_dynamic(
                    self.latest_best_embedding,
                    self.latest_theta_in_map,
                    self.current_x, self.current_y,
                    self.current_yaw,
                    self.assumed_target_distance
                )
            else:
                # Simpler keyframe (embedding + theta + position)
                self.keyframe_manager.add_keyframe(
                    self.latest_best_embedding,
                    self.latest_theta_in_map,
                    self.current_x, self.current_y
                )

    # ---------------------------------------------------------
    # Image Callback (Main pipeline)
    # ---------------------------------------------------------
    def image_callback(self, msg: Image):
        """
        Main perception callback.
        Steps:
        1. Convert image → RGB + resize
        2. Compute tile embeddings (multi-scale optional)
        3. Compute CLIP scores
        4. Determine target tile and visibility
        5. Convert tile → camera angle → global map angle
        6. Publish target theta and marker
        7. If invisible: use keyframe fallback
        8. Optionally draw & save visualization
        """
        if self.current_yaw is None or self.h_fov is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except:
            return

        image_resized = cv2.resize(image, self.target_size)

        # Choose scoring mode
        if self.use_multiscale:
            scores, embeddings_2x3 = self._compute_multiscale_scores(image_resized)
            embeddings = embeddings_2x3
        else:
            tiles = split_image_into_tiles(image_resized, self.grid)
            embeddings = compute_tile_embeddings(tiles, self.model, self.preprocess, self.device)
            scores = compute_prompt_scores(
                embeddings, self.model, self.device,
                self.target_text, self.target_image_embedding
            )

        # Compute tile angles in camera frame
        tile_angles_cam = map_tile_to_cam_angles(self.grid, self.h_fov)

        # Visibility heuristic using saliency contrast
        max_score = float(np.max(scores))
        mean_score = float(np.mean(scores))
        std_score = float(np.std(scores))
        r_score = max_score / (mean_score + 1e-6)

        # Two visibility modes
        if self.use_threshold:
            visible = (max_score > 0.25)
        else:
            visible = (r_score > 0.85 and std_score > 0.050)

        self.pub_visible.publish(Bool(data=bool(visible)))

        # ---------------------------------------------------------
        # Convert tile → global angle (map-frame theta)
        # ---------------------------------------------------------
        if self.current_x is not None and self.current_y is not None and self.current_yaw is not None:

            if visible:
                # Pick highest-scoring tile
                best_idx = int(np.argmax(scores))
                best_angle_cam = float(tile_angles_cam[best_idx])

                # Convert to map frame
                theta_in_map = self.current_yaw - best_angle_cam
                theta_in_map = (theta_in_map + math.pi) % (2 * math.pi) - math.pi

                self.latest_theta_in_map = theta_in_map
                if embeddings is not None:
                    self.latest_best_embedding = embeddings[best_idx]

                self.pub_theta.publish(Float32(data=theta_in_map))
                publish_marker(self.pub_marker, self.current_x, self.current_y, theta_in_map, color='green')

            else:
                # ---------------------------------------------------------
                # Fallback: use recent keyframes when target is invisible
                # ---------------------------------------------------------
                recent_keyframes = list(self.keyframes)[-int(self.max_keyframe_compare):]

                if len(recent_keyframes) > 0:
                    thetas = [theta for _, theta, _, _ in recent_keyframes]
                    avg_theta = average_angles(thetas)
                    self.pub_theta.publish(Float32(data=avg_theta))
                    publish_marker(self.pub_marker, self.current_x, self.current_y, avg_theta, color='blue')

                else:
                    # Hard fallback: use robot's current facing direction
                    self.pub_theta.publish(Float32(data=self.current_yaw))
                    publish_marker(self.pub_marker, self.current_x, self.current_y, self.current_yaw, color='blue')

        # ---------------------------------------------------------
        # Visualization overlay (draw scores)
        # ---------------------------------------------------------
        try:
            image_bgr = cv2.cvtColor(image_resized, cv2.COLOR_RGB2BGR)
            hi = int(np.argmax(scores)) if visible else None
            draw_scores(image_bgr, scores, grid=self.grid, highlight_idx=hi, show=self.show_window)

            self._ensure_video_writer()
            self._write_vis_frame(image_bgr)
        except:
            pass

    # ---------------------------------------------------------
    # Load target image embedding (optional)
    # ---------------------------------------------------------
    def _load_target_image_embedding(self):
        """Load a target reference image and compute its CLIP embedding."""
        if self.target_image_path is None or self.target_image_path == "":
            return None

        img = PILImage.open(self.target_image_path).convert("RGB")
        processed = self.preprocess(img).unsqueeze(0).to(self.device)
        with torch.no_grad():
            feat = self.model.encode_image(processed)
            feat = feat / feat.norm(dim=-1, keepdim=True)
        return feat.detach().cpu().numpy()

    # ---------------------------------------------------------
    # Multiscale scoring pipeline
    # ---------------------------------------------------------
    def _compute_multiscale_scores(self, image):
        """
        Compute CLIP scores using 3 scales:
        - 2x3 (coarse)
        - 4x6 (mid)
        - 8x12 (fine)

        Then fuse saliency bottom-up using std-based exponential weighting.
        """
        # Coarse 2x3
        tiles_2x3 = split_image_into_tiles_grid(image, (2, 3))
        embeddings_2x3 = compute_tile_embeddings(tiles_2x3, self.model, self.preprocess, self.device)
        s2 = compute_prompt_scores(embeddings_2x3, self.model, self.device,
                                   self.target_text, self.target_image_embedding).reshape((2, 3))

        # Mid 4x6
        tiles_4x6 = split_image_into_tiles_grid(image, (4, 6))
        emb_4x6 = compute_tile_embeddings(tiles_4x6, self.model, self.preprocess, self.device)
        s4 = compute_prompt_scores(emb_4x6, self.model, self.device,
                                   self.target_text, self.target_image_embedding).reshape((4, 6))

        # Fine 8x12
        tiles_8x12 = split_image_into_tiles_grid(image, (8, 12))
        emb_8x12 = compute_tile_embeddings(tiles_8x12, self.model, self.preprocess, self.device)
        s8 = compute_prompt_scores(emb_8x12, self.model, self.device,
                                   self.target_text, self.target_image_embedding).reshape((8, 12))

        # Fuse fine → mid
        s4_fused = np.copy(s4)
        for i in range(4):
            for j in range(6):
                patch = s8[i*2:(i+1)*2, j*2:(j+1)*2].flatten()
                top2 = np.sort(patch)[-2:]
                std = np.clip(np.std(patch), 0, 1.0)
                bonus = (self.saliency_base_8x12_to_4x6 ** std)
                s4_fused[i, j] += bonus * np.sum(top2)

        # Fuse mid → coarse
        s2_fused = np.copy(s2)
        for i in range(2):
            for j in range(3):
                patch = s4_fused[i*2:(i+1)*2, j*2:(j+1)*2].flatten()
                top1 = np.max(patch)
                std = np.clip(np.std(patch), 0, 1.0)
                bonus = (self.saliency_base_4x6_to_2x3 ** std)
                s2_fused[i, j] += bonus * top1

        return s2_fused.flatten(), embeddings_2x3

    # ---------------------------------------------------------
    # Video writer setup
    # ---------------------------------------------------------
    def _ensure_video_writer(self):
        """Initialize video writer if needed."""
        if not self.save_visualization:
            return
        if self.video_writer is not None:
            return
        out_w, out_h = self.target_size
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
        self.video_writer = cv2.VideoWriter(
            self.video_output_path, fourcc, self.video_fps, (out_w, out_h)
        )
        if not self.video_writer.isOpened():
            self.video_writer = None

    def _write_vis_frame(self, frame_bgr):
        """Write one visualization frame to output video."""
        if not self.save_visualization or self.video_writer is None:
            return
        out_w, out_h = self.target_size
        if (frame_bgr.shape[1], frame_bgr.shape[0]) != (out_w, out_h):
            frame_bgr = cv2.resize(frame_bgr, (out_w, out_h))
        self.video_writer.write(frame_bgr)

    def _cleanup_video_writer(self):
        """Release video writer on node shutdown."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None


def main():
    node = TileGlobalDirectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
