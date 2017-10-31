#include "withrobot_camera.hpp"	/* withrobot camera API */

#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

class oCam_ROS
{
private:
  Withrobot::Camera* camera_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  ros::Publisher image_info_pub_;

  int width_;
  int height_;
  int fps_;
  std::string config_file_;
  std::string device_path_;
  std::string fourcc_format_;

public:
  oCam_ROS();
  ~oCam_ROS();
};
