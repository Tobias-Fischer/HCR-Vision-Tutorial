# PCL and ROS in C++

## Headers
```
  // point cloud conversions
  #include <pcl_conversions/pcl_conversions.h>
  // ROS point cloud
  #include <pcl_ros/point_cloud.h>
  // point cloud types
  #include <pcl/point_types.h>
```
## Subscriber
```
  ros::NodeHandle nh;
  // if callback is a function
  ros::Subscriber sub=nh.subscribe("topic_name", buffer_dim, callback);
  // if callback is a class method
  ros::Subscriber sub=nh.subscribe("topic_name", buffer_dim, &class_name::callback, this);
```
## Callback without copying data
```
  void callback(const PointCloud::ConstPtr& msg)
  {
    PointCloud::ConstPtr pointCloudPtr=msg;
    // now pointCloudPtr is a pointer to the point cloud
    // ...
  }
```
## CMake
```
  find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    pcl_conversions
  )
  find_package(PCL REQUIRED)
  catkin_package(
    CATKIN_DEPENDS pcl_ros 
  )
```
# OpenCV and ROS in C++

## Headers
```
  #include <cv_bridge/cv_bridge.h>
  #include <image_transport/image_transport.h>
  // OpenCV
  #include <opencv2/opencv.hpp>
  // image encodings
  #include <sensor_msgs/image_encodings.h>
  // image msg
  #include <sensor_msgs/Image.h>
```
## Subscriber
```
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // if callback is a function
  image_transport::Subscriber sub=it.subscribe("topic_name", buffer_dim, callback);
  // if callback is a class method
  image_transport::Subscriber sub=it.subscribe("topic_name", buffer_dim, &class_name::callback, this);
```
## Callback with no copy
```
  void callback(const sensor_msgs::Image::ConstPtr& msg)
  {
    // cv_bridge pointer that interfaces between the image msg and cv::Mat image type
    cv_bridge::CvImageConstPtr cvPtr;
    try
    {
      // no copy; 'encoding' is the image encoding
      cvPtr=cv_bridge::toCvShare(msg, encoding);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("[callback] cv_bridge exception: %s", e.what());
      return;
    }
    // now cvPtr->image is a cv::Mat image
    // ...
    // copy the image, for example
    cv::Mat img;
    cvPtr->image.copyTo(img);
    // ...
  }
```
## CMake
```
  find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    cv_bridge
    image_transport
  )
  find_package(OpenCV REQUIRED)
  catkin_package(
    CATKIN_DEPENDS sensor_msgs cv_bridge image_transport
    DEPENDS OpenCV
  )
```
