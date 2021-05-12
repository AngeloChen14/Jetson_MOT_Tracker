#pragma once

// ROS
#include <ros/ros.h>

#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

namespace jetson_mot_tracker {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class MotTracker
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  MotTracker(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~MotTracker();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void detectCallback(const vision_msgs::Detection2DArray& message);
  void depthCallback(const sensor_msgs::Image& message);
  void caminfoCallback(const sensor_msgs::CameraInfo& message);

  /*!
   * Calculate 3D poition of detections based on 2D detection bounding box and anligned depth image
   * @param message the received message.
   */
  void positionCalculator(const vision_msgs::Detection2DArray& detects,vision_msgs::Detection2DArray& detects_out,const sensor_msgs::Image& depthimage, image_geometry::PinholeCameraModel& cam_model);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber detect_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber caminfo_sub_;

  ros::Publisher angle_pub_;

  ros::Timer timer1_;

  //! ROS topic name to subscribe to.
  std::string detectSubTopic_;
  std::string depthSubTopic_;
  std::string caminfoSubTopic_;
  
  sensor_msgs::Image depth_image_;
  image_geometry::PinholeCameraModel cam_model_;

  //! ROS service server.
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
};
} /* namespace */
