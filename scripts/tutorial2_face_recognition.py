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


class FaceDetector(object):
    def __init__(self):
        self.pub_face_recognition = rospy.Publisher("/face", String, queue_size=1)
        self.cv_bridge = CvBridge()
        cascPath = os.path.join(rospkg.RosPack().get_path("ros_vision_tutorial"), "models/haarcascade_frontalface_default.xml")
        self.faceCascade = cv2.CascadeClassifier(cascPath)

        # Note: I recommend creating the subscriber always last, so that all other variables already exist
        # For images, make sure to set a large buff_size to avoid lags
        self.sub = rospy.Subscriber("/image", Image, self.callback, queue_size=1, buff_size=2**24)
        print('Init done')

    def callback(self, msg):
        # First convert the ROS message to an OpenCV compatible image type
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # You can now use any OpenCV operation you find to extract "meaning" from the image
        # Here, let's extract faces from the image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        img_draw = img.copy()

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(img_draw, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Now, let's convert the OpenCV image back to a ROS message
        face_img = self.cv_bridge.cv2_to_imgmsg(img_draw)

        # I recommend setting the timestamp consistent with that of the original image
        # This is useful if you want to synchronise images later on
        face_img.header.stamp = msg.header.stamp

        # Finally, publish the face image
        self.pub_face_image.publish(face_img)


if __name__ == "__main__":
    try:
        rospy.init_node("face_detector")
        face_detector = FaceDetector()
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

