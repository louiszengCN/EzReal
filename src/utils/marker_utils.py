from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
#  用来维护可视化的 绿色箭头代表目标可见时的目标方向 红色代表关键帧方向 蓝色代表遮挡时的维持方向
#  This is visualization code. Green ARROW means the direction when goal is visible, Red ARROW is the KEYFRAME direction
#  Blue ARROW denotes the heading maintain direction during occlusion.
def publish_marker(publisher, x, y, theta, color='green'):
    marker = Marker()
    marker.header.frame_id = "camera_init"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.70
    marker.scale.y = 1.5
    marker.scale.z = 1.0

    if color == 'green':
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'blue':
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    else:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    marker.color.a = 1.0

    start = Point(x=x, y=y, z=0.0)
    end = Point(
        x=start.x + math.cos(theta) * 2.0,
        y=start.y + math.sin(theta) * 2.0,
        z=0.0
    )
    marker.points = [start, end]
    publisher.publish(marker)

def publish_keyframe_marker(publisher, x, y, theta, marker_id):
    marker = Marker()
    marker.header.frame_id = "camera_init"
    marker.ns = "keyframe_directions"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.70
    marker.scale.y = 1.50
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    start = Point(x=x, y=y, z=0.0)
    end = Point(
        x=start.x + math.cos(theta) * 1.5,
        y=start.y + math.sin(theta) * 1.5,
        z=0.0
    )
    marker.points = [start, end]
    publisher.publish(marker)
