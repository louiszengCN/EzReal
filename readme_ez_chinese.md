# Ezreal 代码说明
---

## 1️⃣ 3D22D.py 
**功能：3D 点云 → 2D 占据栅格地图，适合用3D SLAM 算法的情况（LIO-SAM，FAST-LIO），请把话题替换为自己设备上的**

### **Subscribe**
| Topic | Type | Description |
|-------|------|-------------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 输入点云（激光/深度） |
| `/Odometry` | `nav_msgs/Odometry` | 机器人位姿（用于定位 Ego grid） |

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 输出的 2D 栅格地图 |

---

## 2️⃣ tile_node.py
**功能：RGB 图像 → 目标方向估计（theta）+ 可见性判断，请把话题替换为自己设备上的**

### **Subscribe**
| Topic                   | Type | Description |
|-------------------------|------|-------------|
| `/image`                | `sensor_msgs/Image` | 输入 RGB 图像 |
| `/camera_info` | `sensor_msgs/CameraInfo` | 相机内参（用于计算 FOV） |
| `/Odometry`             | `nav_msgs/Odometry` | 机器人全局朝向 |
| `/best_frontier_pose`   | `geometry_msgs/PoseStamped` | 来自前沿点模块，用于 keyframe fallback |
**本代码不含规划器，如需要驱动机器人底盘，需要用规划器或者controller订阅/best_frontier_pose，即可控制机器人开始探索**

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/target_theta` | `std_msgs/Float32` | 目标全局方向（弧度） |
| `/target_visible` | `std_msgs/Bool` | 目标是否可见 |
| `/tile_target_marker` | `visualization_msgs/Marker` | Rviz 方向箭头可视化 |

---

## 3️⃣ frontier_node.py  
**功能：前沿点检测 + 融合方向信息选择探索目标**

### **Subscribe**
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 输入栅格地图（来自 EgoGridMapper） |
| `/Odometry` | `nav_msgs/Odometry` | 机器人位置与朝向 |
| `/target_theta` | `std_msgs/Float32` | 视觉模块输出的目标方向 |
| `/target_visible` | `std_msgs/Bool` | 目标是否可见 |

### **Publish**
| Topic | Type | Description |
|-------|------|-------------|
| `/best_frontier_pose` | `geometry_msgs/PoseStamped` | 最终导航目标（前沿点或虚拟目标） |
| `/frontiers_marker` | `visualization_msgs/Marker` | 前沿点的 Rviz 可视化 |

---
**/best_frontier_pose即包含了系统选择的最佳前沿点坐标，订阅该坐标即可实现机器人导航**
## 🗺 Overall Topic Flow
```
3D22D.py
├── sub: /cloud_registered, /Odometry
└── pub: /map

Frontier_node.py
├── sub: /image_rect_color, /camera_info, /Odometry, /best_frontier_pose
└── pub: /target_theta, /target_visible, /tile_target_marker

Tile_node.py
├── sub: /map, /Odometry, /target_theta, /target_visible
└── pub: /best_frontier_pose, /frontiers_marker
```

论文中的active search + fallback strategy和我们的规划器代码写在一起，目前我们正在改进该规划器，预计近期上线 
各位读者可以先采用自己的规划器订阅对应话题运行代码，测试对应功能。

Updating date: 11.24