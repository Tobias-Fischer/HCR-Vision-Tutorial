# Human-Centred Robotics Vision Tutorial

Hi there, I hope you find this repository useful. Please let me know if you have any feedback - for example, you can use the Issue list on GitHub.


## Code (Python):

This repository contains a ROS package. In your `catkin_ws` (get there using `cd ~/catkin_ws/src`), you can clone this repository (`git clone https://github.com/Tobias-Fischer/HCR-Vision-Tutorial.git`) and then build the package `catkin build` or `catkin_make`.

There are five examples included in the `scripts` directory:
1. [`tutorial1_edge_detection.py`](./scripts/tutorial1_edge_detection.py): This example shows the conversion from a ROS `sensor_msgs/Image` to a `numpy` array and applying a simple [OpenCV](https://opencv.org/) method (`Canny` edge detection) to this image. Finally, the `numpy` array is converted back to a ROS `sensor_msgs/Image` and published on a topic.
1. [`tutorial2_face_recognition.py`](./scripts/tutorial2_face_recognition.py): This example is very similar to the first one, but this time a `CascadeClassifier` is used to detect faces rather than detecting edges.
1. [`tutorial3_3d_from_2d.py`](./scripts/tutorial3_3d_from_2d.py): This example demonstrates how to obtain the depth at a given pixel using a depth camera like the Microsoft Kinect or Asus Xtion. This requires synchronisation of two ROS topics which is also demonstrated.
1. [`tutorial4_tensorflow.py`](./scripts/tutorial4_tensorflow.py): This example shows how to use [https://www.tensorflow.org/](TensorFlow) with ROS. In particular, one has to be careful as ROS callbacks are executed on a separate thread, which may cause issues with TensorFlow if not properly handled.
1. [`tutorial5_pytorch.py`](./scripts/tutorial5_pytorch.py): [https://pytorch.org/](PyTorch) is becoming increasingly popular. This is a simple example demonstrating how to use PyTorch in a ROS node (body keypoint detection).

## Code (C++):
For C++ related tricks, please see the [README_CPP.md](./README_CPP.md).

## Excercises:

1. Try combining the third and last examples to obtain the 3D coordinates of the body keypoints.
1. Replace the dummy model in the fourth tutorial with a more sophisticated model.
1. Enhance the second tutorial to output a [http://docs.ros.org/melodic/api/sensor_msgs/html/msg/RegionOfInterest.html](`sensor_msgs/RegionOfInterest`) to a new topic.

## Lecture slides:
Please see the [slides.pdf](./slides.pdf)

## Further reading:
1. [Programming with ROS Chapter 12: Follow-Bot](https://github.com/osrf/rosbook/) - this includes a nice exercise that allows the TurtleBot to follow a line
1. [OpenCV in Python](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/OpenCV-in-Python.html) - this includes a good overview of OpenCV in Python, with a lot of exercises.
1. [ROS Wiki](http://wiki.ros.org) - the wiki provides an overview of ROS packages with corresponding tutorials.
1. [OpenCV docs](https://docs.opencv.org/) - provides the API for the various OpenCV classes, as well as tutorials in Python and C++
1. [PCL docs](http://docs.pointclouds.org) - while OpenCV is designed to handle 2D images, PCL is designed to work with point clouds. As above, the docs contain examples and the API.
1. [Papers with Code](https://paperswithcode.com) - provides an overview of state-of-the-art methods that provide code in a wide range of computer vision tasks. Note that this often requires substantial installation and is not nearly as straightforward to use as OpenCV or PCL.
