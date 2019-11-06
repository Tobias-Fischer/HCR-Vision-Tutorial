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
import tensorflow as tf


class ROSTensorFlow(object):
    def __init__(self):
        # Setup tensorflow (v1.14 / v2.0 compatible)
        tf.compat.v1.disable_eager_execution()
        self.device_id = "/gpu:0"
        self.input_size = 32
        self.labels = ['airplane', 'car', 'cat', 'dog', 'flower', 'fruit', 'motorbike', 'person']

        with tf.device(self.device_id):
            config = tf.compat.v1.ConfigProto(inter_op_parallelism_threads=1,
                                              intra_op_parallelism_threads=1)
            if "gpu" in self.device_id:
                config.gpu_options.allow_growth = True
                config.gpu_options.per_process_gpu_memory_fraction = 0.3
            config.log_device_placement = False
            self.sess = tf.compat.v1.Session(config=config)
            tf.compat.v1.keras.backend.set_session(self.sess)

        # Load your model. You will need to specify model_path
        # Toy model from https://github.com/shubham0204/Multiclass_Object_Classification7
        # Trained on https://www.kaggle.com/prasunroy/natural-images
        model_path = os.path.join(rospkg.RosPack().get_path("ros_vision_tutorial"), "models/object_classifier_basic.h5")
        self.model = tf.keras.models.load_model(model_path, compile=False)
        self.model._make_predict_function()
        self.graph = tf.compat.v1.get_default_graph()

        self.cv_bridge = CvBridge()

        self.sub = rospy.Subscriber("/image", Image, self.callback, queue_size=1, buff_size=2**24)

        print('Init done')

    def callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # probably you will need to do some pre-processing like image resizing etc.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_resized = cv2.resize(gray, dsize=(self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        img_resized = np.array(np.reshape(img_resized, (self.input_size, self.input_size, 1))) / 255
        sample_X = img_resized.reshape( ( 1 , self.input_size**2 ) ).astype( np.float32 )

        # This is a trick specific to ROS
        # As in ROS, the callback runs in a separate thread
        # Which is not typically the case in Python scripts!
        with self.graph.as_default():
            tf.compat.v1.keras.backend.set_session(self.sess)
            prediction = self.model.predict(sample_X, verbose=0).argmax(1)
            # Now, do whatever you want with the prediction ..
            print('Prediction object class: {:10s}'.format(self.labels[prediction[0]]), end='\r', flush=True)


if __name__ == "__main__":
    try:
        rospy.init_node("ros_tensorflow_example")
        ros_tf = ROSTensorFlow()
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
