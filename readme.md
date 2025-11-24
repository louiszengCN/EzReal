# Ezreal Code Overview
---

## 1️⃣ 3D22D.py
**Function:** Converts 3D point clouds → 2D occupancy grid.  
Designed for use with 3D SLAM systems (e.g., **LIO-SAM**, **FAST-LIO**).  
Please replace all topics according to your own hardware/device setup.

### **Subscribe**
| Topic | Type | Description |
|-------|------|-------------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Input point cloud (LiDAR or depth sensor) |
| `/Odometry` | `nav_msgs/Odometry` | Robot pose used to align the ego-centric grid |

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Generated 2D occupancy map |

---

## 2️⃣ tile_node.py
**Function:** Computes target direction (theta) and visibility from RGB images.  
Useful for long-range target guidance in outdoor navigation.

### **Subscribe**
| Topic | Type | Description |
|-------------------------|------|-------------|
| `/image` | `sensor_msgs/Image` | RGB camera stream |
| `/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (used to compute horizontal FOV) |
| `/Odometry` | `nav_msgs/Odometry` | Robot pose and global yaw |
| `/best_frontier_pose` | `geometry_msgs/PoseStamped` | Feedback from the frontier module for keyframe fallback |

> **Note:**  
> This node does *not* include a motion planner.  
> To move the robot, your planner/controller should subscribe to **/best_frontier_pose**.

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/target_theta` | `std_msgs/Float32` | Estimated global direction of the target (radians) |
| `/target_visible` | `std_msgs/Bool` | Whether the target is currently visible |
| `/tile_target_marker` | `visualization_msgs/Marker` | Visualization arrow in RViz |

---

## 3️⃣ frontier_node.py
**Function:** Frontier detection + selection of optimal navigation goal  
(by fusing visual direction and map frontier structure).

### **Subscribe**
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid from EgoGridMapper |
| `/Odometry` | `nav_msgs/Odometry` | Robot position and orientation |
| `/target_theta` | `std_msgs/Float32` | Direction from the vision module |
| `/target_visible` | `std_msgs/Bool` | Visibility output from the vision module |

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/best_frontier_pose` | `geometry_msgs/PoseStamped` | Selected navigation goal (frontier or virtual goal) |
| `/frontiers_marker` | `visualization_msgs/Marker` | RViz visualization of detected frontiers |

> **Note:**  
> `/best_frontier_pose` contains the final (x, y) navigation target.  
> Your planner can directly subscribe to this topic to control the robot.

---

## 🗺 Overall Topic Flow

```commandline
3D22D.py
├── sub: /cloud_registered, /Odometry
└── pub: /map

tile_node.py
├── sub: /image_rect_color, /camera_info, /Odometry, /best_frontier_pose
└── pub: /target_theta, /target_visible, /tile_target_marker

frontier_node.py
├── sub: /map, /Odometry, /target_theta, /target_visible
└── pub: /best_frontier_pose, /frontiers_marker
```

---

### Notes
- The planner (active search + fallback strategy) is currently being improved and will be released soon.
- For now, users can run their own planner by subscribing to **/best_frontier_pose** to test navigation.

**Updating date: 11.24**
