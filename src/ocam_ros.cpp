#include <stdio.h>
#include <errno.h>

#include "ros/ros.h"
#include "ocam_ros.h"

#include "opencv2/opencv.hpp"
#include "withrobot_camera.hpp"	/* withrobot camera API */


oCam_ROS::oCam_ROS() :
  nh_(), nh_private_("~"), it_(nh_)
{
  std::string image_topic;
  bool show_image;
  nh_private_.param<std::string>("device_path", device_path_, "/dev/video2");
  nh_private_.param<int>("width", width_, 640);
  nh_private_.param<int>("height", height_, 480);
  nh_private_.param<int>("fps", fps_, 80);
  nh_private_.param<std::string>("format", fourcc_format_, "GREY");
  nh_private_.param<std::string>("image_topic", image_topic, "camera/image");
  nh_private_.param<bool>("show_image", show_image, true);

  nh_private_.param<std::string>("config_file", config_file_, "~/ocam.yaml");

  image_pub_ = it_.advertise(image_topic, 1);

  /*
   * initialize oCam-1MGN
   *
   * [ supported image formats ]
   *
   * USB 3.0
   * 	[0] "8-bit Greyscale 1280 x 720 60 fps"
 *	[1] "8-bit Greyscale 1280 x 960 45 fps"
 *	[2] "8-bit Greyscale 320 x 240 160 fps"
 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
 *
 * USB 2.0
   * 	[0] "8-bit Greyscale 1280 x 720 30 fps"
 *	[1] "8-bit Greyscale 1280 x 960 22.5 fps"
 *	[2] "8-bit Greyscale 320 x 240 160 fps"
 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
 *
   */
  const char* devPath = "/dev/video2";

  camera_ = new Withrobot::Camera(device_path_.c_str());

  /* USB 3.0 */
  /* 8-bit Greyscale 1280 x 720 60 fps */
  camera_->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, fps_);

  /* 8-bit Greyscale 1280 x 960 45 fps */
  //camera_->set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 45);

  /* 8-bit Greyscale 320 x 240 160 fps */
  //camera_->set_format(320, 240, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 160);

  /* 8-bit Greyscale 640 x 480 80 fps */
  //camera_->set_format(640, 480, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 80);

  /* USB 2.0 */
  /* 8-bit Greyscale 1280 x 720 30 fps */
  //camera_->set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 30);

  /* 8-bit Greyscale 1280 x 960 22.5 fps */
  //camera_->set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 2, 45);

  /* 8-bit Greyscale 320 x 240 160 fps */
  //camera_->set_format(320, 240, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 160);

  /* 8-bit Greyscale 640 x 480 80 fps */
  //camera_->set_format(640, 480, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 80);

  /*
   * get current camera format (image size and frame rate)
   */
  Withrobot::camera_format camFormat;
  camera_->get_current_format(camFormat);

  /*
   * Print infomations
   */
  std::string camName = camera_->get_dev_name();
  std::string camSerialNumber = camera_->get_serial_number();

  ROS_INFO("dev: %s, serial number: %s", camName.c_str(), camSerialNumber.c_str());
  ROS_INFO("----------------- Current format informations -----------------");
  camFormat.print();
  ROS_INFO("---------------------------------------------------------------");

  /*
   * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
   *
   *  [0] "Brightness",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )  // gain
   *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
   *
   */
  int brightness = camera_->get_control("Brightness");
  int exposure = camera_->get_control("Exposure (Absolute)");

  camera_->set_control("Brightness", brightness);
  camera_->set_control("Exposure (Absolute)", exposure);

  /*
   * Start streaming
   */
  if (!camera_->start()) {
    ROS_ERROR("Failed to start.");
    exit(0);
  }

  /*
   * Initialize OpenCV
   */
  std::string windowName = camName + " " + camSerialNumber;
  cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
  if (show_image)
    cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO|CV_WINDOW_AUTOSIZE);

  /*
   * Main loop
   */
  while (ros::ok()) {
    /* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    int size = camera_->get_frame(srcImg.data, camFormat.image_size, 1);

    /* If the error occured, restart the camera_-> */
    if (size == -1) {
      printf("error number: %d\n", errno);
      ROS_ERROR("Cannot get image from camera");
      camera_->stop();
      camera_->start();
      continue;
    }

    /* Publish Image to ROS */
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", srcImg).toImageMsg();
    image_pub_.publish(msg);
    ros::spinOnce();

    if (show_image)
    {
      /* Show image */
      cv::imshow(windowName.c_str(), srcImg);
      char key = cv::waitKey(1);

      /* Keyboard options */
      switch (key)
      {
      /* When press the 'q' key then quit. */
      case 'q':
        cv::destroyAllWindows();
        show_image = false;
        break;

        /* When press the '[' key then decrease the exposure time. */
      case '[':
        exposure = camera_->get_control("Exposure (Absolute)");
        camera_->set_control("Exposure (Absolute)", --exposure);
        break;

        /* When press the ']' key then increase the exposure time. */
      case ']':
        exposure = camera_->get_control("Exposure (Absolute)");
        camera_->set_control("Exposure (Absolute)", ++exposure);
        break;

        /* When press the '-' key then decrease the brightness. */
      case '-':
        exposure = camera_->get_control("Brightness");
        camera_->set_control("Brightness", --brightness);
        break;

        /* When press the '=' key then increase the brightness. */
      case '=':
        exposure = camera_->get_control("Brightness");
        camera_->set_control("Brightness", ++brightness);
        break;

      default:
        break;
      }
    }
  }

  cv::destroyAllWindows();

  /*
   * Stop streaming
   */
  camera_->stop();

  ROS_INFO("Ocam Done.\n");
}

oCam_ROS::~oCam_ROS()
{
  delete camera_;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ocam_ros_node");
  oCam_ROS thing;
  return 0;
}
