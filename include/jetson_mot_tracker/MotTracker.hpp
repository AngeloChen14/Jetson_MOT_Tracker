#pragma once

#include "jetson_mot_tracker/tracker.h" 
// ROS
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc.hpp>


#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
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
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);
  void caminfoCallback(const sensor_msgs::CameraInfo& message);

  /*!
   * Calculate 3D poition of detections based on 2D detection bounding box and anligned depth image
   * @param detects the received Detection2DArray message.
   * @return true if valid human detection exists, false otherwise.
   */
  bool positionCalculator(const vision_msgs::Detection2DArray& detects,vision_msgs::Detection2DArray& detects_out,const sensor_msgs::Image& depthimage, image_geometry::PinholeCameraModel& cam_model);
  
    /*!
   * Calculate 3D poition of detections based on 2D detection bounding box and anligned depth image
   * @param detects the received Detection2DArray message.
   * @return true if valid human detection exists, false otherwise.
   */
  std::vector<geometry_msgs::Point> detectPreprocessing(const vision_msgs::Detection2DArray& detectsArray);
  
  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  //! ROS node handle.
  void bodyMarkerPublish(Tracker& trackers);

  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber detect_sub_;
  // ros::Subscriber depth_sub_;
  ros::Subscriber caminfo_sub_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber depth_sub_;

  ros::Publisher body_marker_publisher_;
  ros::Publisher detection_publisher_;

  ros::Timer timer1_;

  //! ROS topic name to subscribe to.
  std::string detectSubTopic_;
  std::string depthSubTopic_;
  std::string caminfoSubTopic_;
  std::string detectGlobalFrame_;
  
  sensor_msgs::Image depth_image_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Time lastUpdateTime_;

  //! ROS tf2 .
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  // message_filters::Subscriber<vision_msgs::Detection2DArray> detect_sub_;
  // tf2_ros::MessageFilter<vision_msgs::Detection2DArray> tf2_filter_;


  
  Tracker trackers_;
};

struct Color
{
  float r, g, b, a;
};

using ColorPalette = std::array<Color, 8>;
// a palette of 8 colors to colorize the different body markers and the body index map
static const ColorPalette BODY_COLOR_PALETTE{ { { 1.0f, 0.0f, 0.0f, 1.0f },
                                                { 0.0f, 1.0f, 0.0f, 1.0f },
                                                { 0.0f, 0.0f, 1.0f, 1.0f },
                                                { 1.0f, 1.0f, 0.0f, 1.0f },
                                                { 1.0f, 0.0f, 1.0f, 1.0f },
                                                { 0.0f, 1.0f, 1.0f, 1.0f },
                                                { 0.0f, 0.0f, 0.0f, 1.0f },
                                                { 1.0f, 1.0f, 1.0f, 1.0f } } };

} /* namespace */
