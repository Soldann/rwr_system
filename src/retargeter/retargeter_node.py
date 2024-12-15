#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from faive_system.src.retargeter import Retargeter
from faive_system.src.common.utils import numpy_to_float32_multiarray
import os
from faive_system.src.viz.visualize_mano import ManoHandVisualizer
import time
# from tf_transformations import euler_from_quaternion
from visualize_retargeter import KeyvectorVisualizer

class RetargeterNode(Node):
    def __init__(self, debug=False):
        super().__init__("rokoko_node")

        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/urdf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/mano_adjustments", "")
        self.declare_parameter("retarget/retargeter_cfg", "")
        self.declare_parameter("debug", True)

        try:
            mjcf_filepath = self.get_parameter("retarget/mjcf_filepath").value
        except:
            mjcf_filepath = None
        
        try:
            urdf_filepath = self.get_parameter("retarget/urdf_filepath").value
        except:
            urdf_filepath = None
        hand_scheme = self.get_parameter("retarget/hand_scheme").value
        # mano_adjustments's default value is None
        mano_adjustments = self.get_parameter("retarget/mano_adjustments").value
        if mano_adjustments == "":
            mano_adjustments = None

        retargeter_cfg = self.get_parameter("retarget/retargeter_cfg").value
        if retargeter_cfg == "":
            retargeter_cfg = None

        debug = self.get_parameter("debug").value
        
        self.wrist_positions = None
        self.current_wrist_quaternion = Quaternion()
        # self.current_wrist_quaternion.w = 1.0
        # self.current_wrist_quaternion.x = 0.0
        # self.current_wrist_quaternion.y = 0.0
        # self.current_wrist_quaternion.z = 0.0
        self.current_wrist_quaternion.x = 0.09638827335909649
        self.current_wrist_quaternion.y = 0.09438689349698638
        self.current_wrist_quaternion.z = 0.11731742291713203
        self.current_wrist_quaternion.w = 0.9838887322126031
        self.wrist_angle = 0.0

        # subscribe to ingress topics
        self.ingress_mano_sub = self.create_subscription(
            Float32MultiArray, "/ingress/mano", self.ingress_mano_cb, 10
        )

        self.ingress_wrist_sub = self.create_subscription(
            PoseStamped, "/ingress/wrist", self.ingress_wrist_cb, 10
        )
        
        self.retargeter = Retargeter(
            device="cpu",  mjcf_filepath= mjcf_filepath, urdf_filepath=urdf_filepath, 
            hand_scheme=hand_scheme, mano_adjustments=mano_adjustments, retargeter_cfg=retargeter_cfg
        )
        
        self.joints_pub = self.create_publisher(
            Float32MultiArray, "/hand/policy_output", 10
        )
        self.debug = debug
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, 'retarget/normalized_mano_points', 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub)

            self.rviz_pub_keyvectors = self.create_publisher(MarkerArray, 'retarget/keyvectors', 10)
            self.keyvector_visualizer = KeyvectorVisualizer(self.rviz_pub_keyvectors)
        
        self.timer = self.create_timer(0.005, self.timer_publish_cb)
        self.keypoint_positions = None
    
    def ingress_mano_cb(self, msg):
        self.keypoint_positions = np.array(msg.data).reshape(-1, 3)
    
    def ingress_wrist_cb(self,msg):
        self.wrist_positions = msg.pose
        if self.current_wrist_quaternion == None:
            self.current_wrist_quaternion = msg.pose.orientation

    def quat2yaw(self, q):
        return np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    
    def quat2pitch(self, q):
        return np.arcsin(-2.0*(q.x*q.z - q.w*q.y))
    
    def quat2roll(self, q):
        return np.arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    
    def normalize_quaternion(self, q):
        """Normalize a quaternion to ensure it has unit length."""
        norm = np.linalg.norm(q)
        return q / norm

    def quaternion_angle(self, q1, q2):
        """
        Compute the angle between two quaternions in radians.

        Args:
            q1 (array-like): First quaternion (w, x, y, z).
            q2 (array-like): Second quaternion (w, x, y, z).

        Returns:
            float: The angle between the two quaternions in radians.
        """
        # Normalize the quaternions
        q1 = self.normalize_quaternion(np.array(q1))
        q2 = self.normalize_quaternion(np.array(q2))

        # Compute the dot product of the quaternions
        dot_product = np.dot(q1, q2)

        # Clamp the dot product to the valid range for arccos (to handle numerical errors)
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Compute the angle
        angle = 2 * np.arccos(dot_product)
        return angle
    
    def quaternion_multiply(self, q0, q1):
        """
        Multiplies two quaternions.

        Input
        :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
        :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

        Output
        :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

        """
        # Extract the values from q0
        w0 = q0.w
        x0 = q0.x
        y0 = q0.y
        z0 = q0.z

        # Extract the values from q1
        w1 = q1.w
        x1 = q1.x
        y1 = q1.y
        z1 = q1.z

        # Computer the product of the two quaternions, term by term
        q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        # Create a 4 element array containing the final quaternion
        final_quaternion = Quaternion()
        final_quaternion.w = q0q1_w
        final_quaternion.x = q0q1_x
        final_quaternion.y = q0q1_y
        final_quaternion.z = q0q1_z

        # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
        return final_quaternion
        
    def timer_publish_cb(self):
        if self.keypoint_positions is None:
            print("No keypoints received yet")
            time.sleep(1)
            return
        # try:
        if self.debug:
            self.mano_hand_visualizer.reset_markers()
            self.keyvector_visualizer.reset_markers()

        debug_dict = {}
        joint_angles, debug_dict = self.retargeter.retarget(self.keypoint_positions, debug_dict)

        if self.debug:
            self.mano_hand_visualizer.generate_hand_markers(
                debug_dict["normalized_joint_pos"],
                stamp=self.get_clock().now().to_msg(),
            )

            self.keyvector_visualizer.generate_hand_markers(
                {"palm": debug_dict["palm"]},                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_hand_markers(
                debug_dict["fingertips"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_hand_markers(
                debug_dict["kinematics_chain"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_keyvectors(
                [debug_dict["keyvectors_faive"]["palm2thumb"],
                 debug_dict["keyvectors_faive"]["palm2index"],
                 debug_dict["keyvectors_faive"]["palm2middle"],
                 debug_dict["keyvectors_faive"]["palm2ring"],
                 debug_dict["keyvectors_faive"]["palm2pinky"],
                 ],
                debug_dict["palm"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_keyvectors(
                [debug_dict["keyvectors_faive"]["thumb2index"]],
                debug_dict["fingertips"]["thumb"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_keyvectors(
                [debug_dict["keyvectors_faive"]["index2middle"]],
                debug_dict["fingertips"]["index"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_keyvectors(
                [debug_dict["keyvectors_faive"]["middle2ring"]],
                debug_dict["fingertips"]["middle"],
                stamp=self.get_clock().now().to_msg()
            )
            self.keyvector_visualizer.generate_keyvectors(
                [debug_dict["keyvectors_faive"]["ring2pinky"]],
                debug_dict["fingertips"]["ring"],
                stamp=self.get_clock().now().to_msg()
            )

        if self.wrist_positions and self.current_wrist_quaternion:
            # inverse_q = self.current_wrist_quaternion
            # inverse_q.w = - inverse_q.w
            # relative_q = self.quaternion_multiply(self.wrist_positions.orientation, inverse_q)
            # self.wrist_angle = 2* np.arctan2(np.sqrt(relative_q.x**2 + relative_q.y**2 + relative_q.z**2), relative_q.w)

            self.wrist_angle = self.quaternion_angle([self.current_wrist_quaternion.w, self.current_wrist_quaternion.x, self.current_wrist_quaternion.y, self.current_wrist_quaternion.z,],
                                                     [self.wrist_positions.orientation.w, self.wrist_positions.orientation.x, self.wrist_positions.orientation.y, self.wrist_positions.orientation.z])
            # yaw_change = self.quat2yaw(relative_q)
            # print(yaw_change)
            # self.wrist_angle += np.rad2deg(yaw_change)
            # self.current_wrist_quaternion = self.wrist_positions.orientation
            # self.wrist_angle = euler_from_quaternion((self.wrist_positions.orientation.x, self.wrist_positions.orientation.y, self.wrist_positions.orientation.z, self.wrist_positions.orientation.w))
            # self.wrist_angle = self.wrist_angle[0]
            # wrist_angle = self.quat2yaw(self.wrist_positions.orientation)*-0.8
        # self.wrist_angle = np.clip(- np.rad2deg(self.wrist_angle) + 150,-35,35)
        # self.wrist_angle = - np.rad2deg(self.wrist_angle) + 150
        joint_angles[0] = (( np.rad2deg(self.wrist_angle) - 135)+180) % 360-180
        # joint_angles[0] = 0
        # joint_angles = np.concatenate((joint_angles,wrist_angle))
        self.joints_pub.publish(
            numpy_to_float32_multiarray(np.deg2rad(joint_angles))
        )

        if self.debug:
            self.mano_hand_visualizer.publish_markers()
            self.keyvector_visualizer.publish_markers()
        # except Exception as e:
        #     print(f"Error in timer_publish_cb: {e}")
        #     pass


def main(args=None):
    rclpy.init(args=args)
    node = RetargeterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
