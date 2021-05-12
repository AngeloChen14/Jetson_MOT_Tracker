#include "jetson_mot_tracker/MotTracker.hpp"
// STD
#include <string>

namespace jetson_mot_tracker {

MotTracker::MotTracker(ros::NodeHandle& nodeHandle)
 : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  detect_sub_ = nodeHandle_.subscribe(detectSubTopic_, 100,
                                      &MotTracker::detectCallback, this);
  depth_sub_  = nodeHandle_.subscribe(depthSubTopic_, 100,
                                      &MotTracker::depthCallback, this);
  caminfo_sub_ = nodeHandle_.subscribe(caminfoSubTopic_, 5,
                                      &MotTracker::caminfoCallback, this);
                                      

  angle_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/camera_angle", 100, true);
  // timer1_ = nodeHandle_.createTimer(ros::Duration(0.1),&MotTracker::timer1Callback,this);  

  ROS_INFO("Successfully launched azure sensor angle node.");
}

MotTracker::~MotTracker()
{

}

bool MotTracker::readParameters()
{
  if (!nodeHandle_.getParam("detect_subscriber_topic", detectSubTopic_)) return false;
  if (!nodeHandle_.getParam("depthimage_subscriber_topic", depthSubTopic_)) return false;
  if (!nodeHandle_.getParam("caminfo_subscriber_topic", caminfoSubTopic_)) return false;
  return true;
}

void MotTracker::detectCallback(const vision_msgs::Detection2DArray& msg_raw)
{
  vision_msgs::Detection2DArray msg_updated;
  positionCalculator(msg_raw, msg_updated,depth_image_,rgb_cam_info_); //Update detections with 3D positions
  
  // body_msgs_ = message;
  // std_msgs::Float64 msg;
  // msg.data = calculateAngle(body_msgs_, laser_msgs_);
  // angle_pub_.publish(msg);
}

void MotTracker::depthCallback(const sensor_msgs::Image& message)
{
  depth_image_ = message;
}

void MotTracker::caminfoCallback(const sensor_msgs::CameraInfo& message)
{
  rgb_cam_info_ = message;
}


void MotTracker::positionCalculator(const vision_msgs::Detection2DArray& detects,vision_msgs::Detection2DArray& detects_out,\
                                                     const sensor_msgs::Image& depthimage, const sensor_msgs::CameraInfo& caminfo)
{
  detects_out = detects;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(depthimage,depthimage.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  for(auto detect:detects.detections)
  {
    int x = detect.bbox.center.x;
    int y = detect.bbox.center.y;
    float depth = cv_ptr->image.at<float>(y,x);
    ROS_INFO_STREAM("Target depth at "<<x<<','<<" is:"<<depth);
  }
  
  // return detections;
}

} /* namespace */