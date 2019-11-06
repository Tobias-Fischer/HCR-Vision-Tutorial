#!/usr/bin/env python

"""
@Tobias Fischer (t.fischer@imperial.ac.uk)
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

from __future__ import print_function, division, absolute_import

import os
import rospy
import rospkg
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np


class PixelProjector(object):
    def __init__(self):
        self.cv_bridge = CvBridge()

        # We need this to be able to project pixels to rays, see the callback() method
        print("Wait for camera message")
        cam_info = rospy.wait_for_message("/camera_info", CameraInfo, timeout=None)
        self.img_proc = PinholeCameraModel()
        self.img_proc.fromCameraInfo(cam_info)

        self.tf_broadcaster = TransformBroadcaster()
        self.camera_frame = 'frame'

        # Here, we want to synchronize the color and depth topics
        # This is useful if we want to obtain the 3D coordinates
        # given a 2D point (e.g. a keypoint - centre of the hand or something similar)
        # Make sure to install the ros-melodic-openni-launch package
        color_sub = message_filters.Subscriber('/image', Image)
        depth_sub = message_filters.Subscriber('/depth_registered', Image)

        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1, allow_headerless=False)
        ts.registerCallback(callback)

        print('Init done')

    def callback(self, color_msg, depth_msg):
        # First convert the ROS message to an OpenCV compatible image type
        color_img = self.cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # u, v are 2D pixel coordinates
        u, v = 100.0, 100.0  # this pixel of interest should be obtained from some other vision module

        x, y, _ = self.img_proc(u, v)
        z = depth_img[int(u), int(v)]
        # z = z / 1000.0  # This might be necessary depending on the camera
        print('World coordinates: ', x, y, z)

        self.tf_broadcaster.sendTransform([x, y, z], transformations.quaternion_from_euler(0, 0, 0), color_msg.header.stamp,
                                          '/poi',
                                          self.camera_frame)


if __name__ == "__main__":
    try:
        rospy.init_node("pixel_projector")
        pixel_projector = PixelProjector()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("See ya")
    except rospy.ROSException as e:
        if str(e) == "publish() to a closed topic":
            print("See ya")
        else:
            raise e
    except KeyboardInterrupt:
        print("Shutting down")

