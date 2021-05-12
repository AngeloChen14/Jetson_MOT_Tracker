// #include <ros/ros.h>
#include "jetson_mot_tracker/MotTracker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jetson_mot_tracker");
  ros::NodeHandle nodeHandle("~");
  jetson_mot_tracker::MotTracker MotTracker(nodeHandle);
  
  ros::spin();
  return 0;
}
