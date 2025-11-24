import math
import numpy as np

#  计算tile中心点的全局方向
#  calculate the tile direction in WORLD coordinate
def map_tile_to_cam_angles(grid, h_fov):
    rows, cols = grid
    tile_angles = []
    for i in range(rows):
        for j in range(cols):
            center_ratio = (j + 0.5) / cols - 0.5
            angle = center_ratio * h_fov
            tile_angles.append(angle)
    return tile_angles
#  计算方向的均值
def average_angles(angles):
    return math.atan2(
        np.mean([math.sin(a) for a in angles]),
        np.mean([math.cos(a) for a in angles])
    )

def euler_from_quaternion(q):
    x, y, z, w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw
