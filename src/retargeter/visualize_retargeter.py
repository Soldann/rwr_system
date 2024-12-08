import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
import numpy as np


class KeyvectorVisualizer:
    def __init__(self, marker_publisher):
        self.marker_publisher = marker_publisher
        self.markers = []

    def reset_markers(self):
        self.markers = []

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        for idx, marker in enumerate(marker_array.markers):
            marker.id = idx

        self.marker_publisher.publish(marker_array)

    def generate_keyvectors(self, keyvectors, origin, stamp):
        markers = []
    
        # Add bone lines
        for vector_name, vector in keyvectors.items():
            bone_marker = Marker()
            bone_marker.header.frame_id = "hand_root"
            bone_marker.header.stamp = stamp
            bone_marker.ns = vector_name
            bone_marker.type = Marker.ARROW
            bone_marker.action = Marker.ADD
            bone_marker.scale.x = 0.005  # Line width
            bone_marker.scale.y = 0.010  # Head width
            bone_marker.scale.z = 0.015  # Length width
            bone_marker.color.a = 1.0
            bone_marker.color.r = 1.0  # Red color
            # print("origin", origin)
            # print(vector)
            start_joint = origin.flatten()
            end_joint = vector.flatten()
            start_joint = float(start_joint[0]), float(start_joint[1]), float(start_joint[2])
            end_joint = float(start_joint[0] + end_joint[0]), float(start_joint[1] + end_joint[1]), float(start_joint[2] + end_joint[2])
            p_start = Point(x=start_joint[0], y=start_joint[1], z=start_joint[2])
            p_end = Point(x=end_joint[0], y=end_joint[1], z=end_joint[2])
            bone_marker.points.append(p_start)
            bone_marker.points.append(p_end)

            markers.append(bone_marker)

        self.markers.extend(markers)

    def generate_hand_markers(self, keypoints, stamp):
        markers = []

        # Create marker for joints
        joint_marker = Marker()
        joint_marker.header.frame_id = "hand_root"
        joint_marker.header.stamp = stamp
        joint_marker.ns = "joints"
        joint_marker.type = Marker.POINTS
        joint_marker.action = Marker.ADD
        joint_marker.scale.x = 0.01  # Point width
        joint_marker.scale.y = 0.01  # Point height
        joint_marker.color.a = 1.0
        joint_marker.color.g = 1.0  # red color

        # Add joint points
        for name, point in keypoints.items():
            point = point.flatten()
            point = float(point[0]), float(point[1]), float(point[2])
            p = Point(x=point[0], y=point[1], z=point[2])
            joint_marker.points.append(p)

            label_marker = Marker()
            label_marker.header.frame_id = "root"
            label_marker.header.stamp = stamp
            label_marker.ns = "labels"
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.scale.z = 0.005  # Text Height
            label_marker.color.a = 0.8
            label_marker.color.r = 1.0  # black color
            label_marker.color.g = 1.0  # black color
            label_marker.color.b = 1.0  # black color
            label_marker.text = name
            q = Quaternion()
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            label_marker.pose = Pose() # initialize with same point value and identity quaternion
            label_marker.pose.position = p
            label_marker.pose.orientation = q

            markers.append(label_marker)
        markers.append(joint_marker)
        self.markers.extend(markers)

    def generate_frame_markers(self, origin, x_axis, y_axis, z_axis, stamp):
        markers = []
        axes = {
            "x": (x_axis, (1.0, 0.0, 0.0)),  # Red
            "y": (y_axis, (0.0, 1.0, 0.0)),  # Green
            "z": (z_axis, (0.0, 0.0, 1.0)),  # Blue
        }
        for i, (axis_name, (axis_vector, color)) in enumerate(axes.items()):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "hand_root"
            arrow_marker.header.stamp = stamp
            arrow_marker.ns = "frame"
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.005  # Shaft diameter
            arrow_marker.scale.y = 0.008  # Head diameter
            arrow_marker.scale.z = 0.01  # Head length
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = color[0]
            arrow_marker.color.g = color[1]
            arrow_marker.color.b = color[2]

            # Start and end points of the arrow
            p_start = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
            p_end = Point(
                x=float(origin[0] + axis_vector[0]),
                y=float(origin[1] + axis_vector[1]),
                z=float(origin[2] + axis_vector[2]),
            )
            arrow_marker.points.append(p_start)
            arrow_marker.points.append(p_end)

            markers.append(arrow_marker)

        self.markers.extend(markers)
