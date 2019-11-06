#!/usr/bin/env python

"""
@Tobias Fischer (t.fischer@imperial.ac.uk)
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

from __future__ import print_function, division, absolute_import

import os
import rospy
import rospkg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np


class EdgeDetector(object):
    def __init__(self):
        self.pub_edge_image = rospy.Publisher("/edgeimage", Image, queue_size=1)
        self.cv_bridge = CvBridge()

        # Note: I recommend creating the subscriber always last, so that all other variables already exist
        # For images, make sure to set a large buff_size to avoid lags
        self.sub = rospy.Subscriber("/image", Image, self.callback, queue_size=1, buff_size=2**24)
        print('Init done')

    def callback(self, msg):
        # First convert the ROS message to an OpenCV compatible image type
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # You can now use any OpenCV operation you find to extract "meaning" from the image
        # Here, let's extract edges from the image using the Canny Edge detector
        edges = cv2.Canny(img, 100, 200)
        
        # Now, let's convert the OpenCV image back to a ROS message
        edge_img = self.cv_bridge.cv2_to_imgmsg(edges)

        # I recommend setting the timestamp consistent with that of the original image
        # This is useful if you want to synchronise images later on
        edge_img.header.stamp = msg.header.stamp

        # Finally, publish the edge image
        self.pub_edge_image.publish(edge_img)


if __name__ == "__main__":
    try:
        rospy.init_node("edge_detector")
        edge_detector = EdgeDetector()
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

