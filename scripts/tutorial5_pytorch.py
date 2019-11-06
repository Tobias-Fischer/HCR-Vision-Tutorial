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
import torchvision

"""
Helpful resources:
- https://pytorch.org/blog/torchvision03/
- https://github.com/qixuxiang/mask_rcnn_ros
- https://github.com/facebookresearch/detectron2
"""


class ROSPyTorch(object):
    def __init__(self):
        self.model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=True)
        # set it to evaluation mode, as the model behaves differently
        # during training and during evaluation
        self.model.eval()

        '''
        During inference, the model requires only the input tensors, and returns the post-processed
        predictions as a ``List[Dict[Tensor]]``, one for each input image. The fields of the ``Dict`` are as
        follows:
            - boxes (``FloatTensor[N, 4]``): the predicted boxes in ``[x1, y1, x2, y2]`` format, with values between
              ``0`` and ``H`` and ``0`` and ``W``
            - labels (``Int64Tensor[N]``): the predicted labels for each image
            - scores (``Tensor[N]``): the scores or each prediction
            - keypoints (``FloatTensor[N, K, 3]``): the locations of the predicted keypoints, in ``[x, y, v]`` format.
        '''

        self.cv_bridge = CvBridge()

        self.sub = rospy.Subscriber("/image", Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img_tensor = torchvision.transforms.functional.to_tensor(img)
        output = self.model([img_tensor])
        keypoints_np = output[0]["keypoints"].to("cpu")
        boxes_np = output[0]["boxes"].to("cpu")


if __name__ == "__main__":
    try:
        rospy.init_node("ros_pytorch_example")
        ros_tf = ROSPyTorch()
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
    print()  # print new line
