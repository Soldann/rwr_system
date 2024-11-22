#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from webcam_mediapipe_ingress import MediaPipeTracker
from faive_system.src.common.utils import numpy_to_float32_multiarray
import cv2


class MediapipeNode(Node):
    def __init__(self):
        super().__init__("rokoko_node")

        self.tracker = MediaPipeTracker(show=True)
        self.tracker.start()

        ingress_period = 0.005  # Timer period in seconds
        self.timer = self.create_timer(ingress_period, self.timer_publish_cb)

        self.ingress_mano_pub = self.create_publisher(
            Float32MultiArray, "/ingress/mano", 10
        )

    def timer_publish_cb(self):
        if self.tracker.frame is not None:
            cv2.imshow('MediaPipe Hands', self.tracker.frame)
            if cv2.waitKey(5) & 0xFF == 27:
                self.tracker.stop()
        keypoint_positions = self.tracker.get_keypoint_positions()
        if keypoint_positions is not None:
            print(keypoint_positions)
        # wait_cnt = 1
        # while (keypoint_positions is None):
        #     if (wait_cnt % 10):
        #         print("waiting for hand tracker")
        #     wait_cnt+=1
        #     keypoint_positions = self.tracker.get_keypoint_positions()


            keypoint_positions_msg = numpy_to_float32_multiarray(keypoint_positions)
            self.ingress_mano_pub.publish(keypoint_positions_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MediapipeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
