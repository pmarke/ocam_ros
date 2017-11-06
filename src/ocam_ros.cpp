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
  std::string mono_image_topic;
  bool show_image;
  nh_private_.param<std::string>("device_path", device_path_, "/dev/video2");
  nh_private_.param<int>("width", width_, 640);
  nh_private_.param<int>("height", height_, 480);
  nh_private_.param<int>("fps", fps_, 80);
  nh_private_.param<std::string>("format", fourcc_format_, "GREY");
  nh_private_.param<std::string>("image_topic", image_topic, "camera/image_raw");
  nh_private_.param<std::string>("mono_image_topic", mono_image_topic, "camera/image_mono");
  nh_private_.param<bool>("show_image", show_image, false);

  nh_private_.param<std::string>("config_file", config_file_, "~/ocam.yaml");

  image_pub_ = it_.advertise(image_topic, 1);
  mono_image_pub_ = it_.advertise(mono_image_topic, 1);

  /*
     * initialize oCam-1CGN
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
	const char* devPath = "/dev/video1";

    Withrobot::Camera camera(devPath);

    /* USB 3.0 */
    /* bayer RBG 1280 x 720 60 fps */
    camera.set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, fps_);

    /* bayer RBG 1280 x 960 45 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','B','G','R')), 1, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','B','G','R')'), 1, 80);

    /* USB 2.0 */
    /* bayer RBG 1280 x 720 30 fps */
    //camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 30);

    /* bayer RBG 1280 x 960 22.5 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 2, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 80);

    /*
     * get current camera format (image size and frame rate)
     */
    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    /*
     * Print infomations
     */
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    /*
     * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
     *
     *  [0] "Gain",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )
     *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
     *
     */
    int brightness = camera.get_control("Gain");
    int exposure = camera.get_control("Exposure (Absolute)");

    camera.set_control("Gain", brightness);
    camera.set_control("Exposure (Absolute)", exposure);

    /*
     * Start streaming
     */
    if (!camera.start()) {
    	perror("Failed to start.");
    	exit(0);
    }

    /*
     * Initialize OpenCV
     */
    std::string windowName = camName + " " + camSerialNumber;
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat colorImg(cv::Size(camFormat.width, camFormat.height), CV_8UC3);
    cv::Mat monoImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

    if (show_image)
    {
      cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO|CV_WINDOW_AUTOSIZE);
    }

    /*
     * Main loop
     */
    bool quit = false;
    ros::Rate loop_rate(80);
    while (ros::ok()) {
    	/* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

    	/* If the error occured, restart the camera. */
    	if (size == -1 && errno) {
    	    printf("error number: %d\n", errno);
    	    perror("Cannot get image from camera");
    	    camera.stop();
    	    camera.start();
    	    continue;
    	}

		  cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2BGR);
      cv::cvtColor(colorImg, monoImg, cv::COLOR_BGR2GRAY);

      /* Publish Image to ROS */
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImg).toImageMsg();
      sensor_msgs::ImagePtr mono_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", monoImg).toImageMsg();
      msg->header.frame_id = "ocam";
      mono_msg->header.frame_id = "ocam";
      msg->header.stamp = ros::Time::now();
      mono_msg->header.stamp = ros::Time::now();
      image_pub_.publish(msg);
      mono_image_pub_.publish(mono_msg);
      // ros::spinOnce();
      // loop_rate.sleep();
      if (show_image)
      {
        /* Show image */
      	cv::imshow(windowName.c_str(), colorImg);
        char key = cv::waitKey(1);

      	/* Keyboard options */
      	switch (key) {
      	/* When press the 'q' key then quit. */
      	case 'q':
      		show_image = false;
      		break;

      	/* When press the '[' key then decrease the exposure time. */
      	case '[':
      		exposure = camera.get_control("Exposure (Absolute)");
      		camera.set_control("Exposure (Absolute)", --exposure);
      		break;

  		/* When press the ']' key then increase the exposure time. */
      	case ']':
      		exposure = camera.get_control("Exposure (Absolute)");
      		camera.set_control("Exposure (Absolute)", ++exposure);
      		break;

  		/* When press the '-' key then decrease the brightness. */
      	case '-':
      		exposure = camera.get_control("Gain");
      		camera.set_control("Gain", --brightness);
      		break;

  		/* When press the '=' key then increase the brightness. */
      	case '=':
      		exposure = camera.get_control("Gain");
      		camera.set_control("Gain", ++brightness);
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
    camera.stop();

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
