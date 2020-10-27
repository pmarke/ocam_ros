#include "ocam_ros.h"


oCam_ROS::oCam_ROS() :
    nh_(),
    nh_private_("~"),
    it_(nh_),
    info_manager_(nh_, "ocam"),
    auto_exposure_count_(0)
{
    initROS();
    initCamera();
    initCV();
    camera_->enumerate_controls();

    run();
}

void oCam_ROS::run()
{
    int size = -1;
    int tries = 20;
    while (size == -1 && tries--)
    {
        size = camera_->get_frame(srcImg->data, camFormat_.image_size, 1);
        camera_->stop();
        camera_->start();
    }

    if (tries == 0)
    {
        ROS_ERROR("Unable to retrieve images from camera after 20 tries, quitting");
        return;
    }

    while (ros::ok())
    {
        int size = camera_->get_frame(srcImg->data, camFormat_.image_size, 1);
        now_ = ros::Time::now();

        if (size == -1 && errno) {
            ROS_ERROR("Missed an image from camera");
            camera_->stop();
            camera_->start();
            continue;
        }
        cv::cvtColor(*srcImg, *dstImg, (color_ ? cv::COLOR_BayerGB2BGR : cv::COLOR_BayerGB2GRAY));

//        if (auto_exposure_)
//            autoExposure();
        if (show_image_)
            showImage();

        sensor_msgs::ImagePtr msg;
        msg = cv_bridge::CvImage(std_msgs::Header(), (color_ ? "bgr8" : "mono8"), *dstImg).toImageMsg();
        msg->header.frame_id = frame_id_;
        msg->header.stamp = now_;
        updateCamInfo();

        image_pub_.publish(*msg, info_);

        ros::spinOnce();
    }
    cv::destroyAllWindows();
    camera_->stop();
}

void oCam_ROS::initROS()
{
    nh_private_.param<std::string>("device_path", device_path_, "/dev/video0");
    nh_private_.param<int>("width", width_, 640);
    nh_private_.param<int>("height", height_, 480);
    nh_private_.param<int>("fps", fps_, 100);
    nh_private_.param<std::string>("image_topic", image_topic_, "image");
    nh_private_.param<std::string>("frame_id", frame_id_, "ocam");
    nh_private_.param<std::string>("camera_info_url", cam_info_url_, "");
    nh_private_.param<bool>("show_image", show_image_, false);
    nh_private_.param<bool>("rescale_camera_info", rescale_camera_info_, false);
    nh_private_.param<bool>("auto_exposure", auto_exposure_, false);
    nh_private_.param<bool>("color", color_, false);
    nh_private_.param<int>("wb_red", wb_red_, 125);
    nh_private_.param<int>("wb_blue", wb_blue_, 125);
    nh_private_.param<int>("exposure", exposure_, 100);
    nh_private_.param<int>("brightness", brightness_, 200);

    ROS_INFO_STREAM("Rescale Camera Info: " << (rescale_camera_info_ ? "on" : "off"));
    ROS_INFO_STREAM("Auto Exposure: " << (auto_exposure_ ? "on" : "off"));
    ROS_INFO_STREAM("Color: " << (color_ ? "on" : "off"));
    ROS_INFO_STREAM("brightness: " << brightness_);
    ROS_INFO_STREAM("exposure: " << exposure_);
    ROS_INFO_STREAM("wb_red: " << wb_red_);
    ROS_INFO_STREAM("wb_blue: " << wb_blue_);

    image_pub_ = it_.advertiseCamera(image_topic_, 1);
}

void oCam_ROS::initCamera()
{
    camera_ = new Withrobot::Camera(device_path_.c_str());
    camera_->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','R', 'E', 'Y'), 1, fps_);
//    camera_->set_format(width_, height_, Withrobot::fourcc_to_pixformat('R', 'G', 'B', ' '), 1, fps_);
    camera_->get_current_format(camFormat_);

    camera_->set_control("Gain", brightness_);
    camera_->set_control("Exposure (Absolute)", exposure_);
    camera_->set_control("Exposure Auto", auto_exposure_);
    camera_->set_control("White Balance Red Component", wb_red_);
    camera_->set_control("White Balance Blue Component", wb_blue_);

    brightness_ = camera_->get_control("Gain");
    exposure_ = camera_->get_control("Exposure (Absolute)");

    std::string camName = camera_->get_dev_name();
    std::string camSerialNumber = camera_->get_serial_number();

    ROS_INFO("port: %s, dev: %s, serial number: %s", device_path_.c_str(), camName.c_str(), camSerialNumber.c_str());
    ROS_INFO("----------------- Current format information ------------------");
    ROS_INFO("Pixel format: %c, %c, %c, %c", (camFormat_.pixformat >> 0) & 0xFF, (camFormat_.pixformat >> 8) & 0xFF, (camFormat_.pixformat >> 16) & 0xff, (camFormat_.pixformat >> 24) & 0xFF);
    ROS_INFO("Width: %d, Height: %d, Image size: %d", camFormat_.width, camFormat_.height, camFormat_.image_size);
    ROS_INFO("Frame Rate : %d / %d (%.2f fps)", camFormat_.rate_numerator, camFormat_.rate_denominator, camFormat_.frame_rate);
    ROS_INFO("---------------------------------------------------------------");


    if (info_manager_.validateURL(cam_info_url_))
    {
        info_manager_.loadCameraInfo(cam_info_url_);
    }
    info_ = info_manager_.getCameraInfo();

    if (!camera_->start())
    {
        perror("Failed to start.");
        exit(0);
    }
}

void oCam_ROS::initCV()
{
    srcImg = new cv::Mat(cv::Size(camFormat_.width, camFormat_.height), CV_8UC1);
    dstImg = new cv::Mat(cv::Size(camFormat_.width, camFormat_.height), (color_ ? CV_8UC3 : CV_8UC1));

    if (show_image_)
    {
#if  CV_MAJOR_VERSION >=4
    cv::namedWindow("oCam ROS", cv::WINDOW_KEEPRATIO|cv::WINDOW_AUTOSIZE);
#else
    cv::namedWindow("oCam ROS", CV_WINDOW_KEEPRATIO|CV_WINDOW_AUTOSIZE);
#endif
        

        ROS_INFO("<--------- oCam Adjustment -------->");
        ROS_INFO("Quit adjustment and image view: q");
        ROS_INFO("Decrease exposure: [");
        ROS_INFO("Increase exposure: ]");
        ROS_INFO("Decrease brightness: -");
        ROS_INFO("Increase brightness: =");
        ROS_INFO("Decrease WB Red: 1");
        ROS_INFO("Increase WB Red: 2");
        ROS_INFO("Decrease WB Blue: 3");
        ROS_INFO("Increase WB Blue: 4");
        ROS_INFO("Toggle Auto Exposure: e");
        ROS_INFO("<---------------------------------->");
    }
}

void oCam_ROS::autoExposure()
{
    if ((++auto_exposure_count_ % (fps_/80)) == 0)
    {
        auto_exposure_count_ = 0;
        // secant method for finding exposure at target intensity
        static float intensity_min = 18;
        static float exposure_min = 1;
        static float target_intensity = 128; // 8 bit image is 0-255
        float intensity = cv::mean(*dstImg).val[0]; // mean intensity
        float exposure_new = ((exposure_ - exposure_min)*(target_intensity - intensity_min))/(intensity - intensity_min) + exposure_min;

        // LPF the exposure to prevent large jumps inducing oscillations
        static float alpha = 0.7;
        exposure_ = int(alpha*exposure_ + (1-alpha)*exposure_new);

        // saturate exposure at its limits
        if (exposure_ > 625)
            exposure_ = 625;
        if (exposure_ < 1)
            exposure_ = 1;

        // set the new exposure
        camera_->set_control("Exposure (Absolute)", exposure_);
    }
}

void oCam_ROS::updateCamInfo()
{

    if (info_.height == 0 && info_.width == 0)
    {
        info_.height = dstImg->rows;
        info_.width = dstImg->cols;
    }
    else if (info_.height != dstImg->rows || info_.width != dstImg->cols)
    {
        if (rescale_camera_info_)
        {
            int old_width = info_.width;
            int old_height = info_.height;
            rescaleCameraInfo(dstImg->cols, dstImg->rows);
            ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                          old_width, old_height, dstImg->cols, dstImg->rows);
        }
        else
        {
            ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                          "Use rescale_camera_info param for rescaling",
                          info_.width, info_.height, dstImg->cols, dstImg->rows);
        }
    }
    info_.header.stamp = now_;
    info_.header.frame_id = frame_id_;
}



// Rescaling taken from cv_camera package. See http://wiki.ros.org/cv_camera for ROS 
// package info and https://github.com/OTL/cv_camera for source.
void oCam_ROS::rescaleCameraInfo(int width, int height)
{
    double width_coeff = width / info_.width;
    double height_coeff = height / info_.height;
    info_.width = width;
    info_.height = height;

    // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
    info_.K[0] *= width_coeff;
    info_.K[2] *= width_coeff;
    info_.K[4] *= height_coeff;
    info_.K[5] *= height_coeff;

    info_.P[0] *= width_coeff;
    info_.P[2] *= width_coeff;
    info_.P[5] *= height_coeff;
    info_.P[6] *= height_coeff;
}

void oCam_ROS::showImage()
{
    if (show_image_)
    {
        cv::imshow("oCam ROS", *dstImg);
        char key = cv::waitKey(1);

        switch (key)
        {
        case 'q':
            show_image_ = false;
            break;

        case '[':
            if (exposure_ > 1)
            {
                camera_->set_control("Exposure (Absolute)", --exposure_);
                std::cout << "oCam Exposure: " << exposure_ << "\n";
            }
            break;

        case ']':
            if (exposure_ < 625)
            {
                camera_->set_control("Exposure (Absolute)", ++exposure_);
                std::cout << "oCam exposure: " << exposure_ << "\n";
            }
            break;

        case '-':
            if (brightness_ > 0)
            {
                camera_->set_control("Gain", --brightness_);
                std::cout << "oCam brightness: " << brightness_ << "\n";
            }
            break;

        case '=':
            if (brightness_ < 250)
            {
                camera_->set_control("Gain", ++brightness_);
                std::cout << "oCam brightness: " << brightness_ << "\n";
            }
            break;
        case '1':
            if (wb_red_ < 250)
            {
                camera_->set_control("White Balance Red Component", ++wb_red_);
                std::cout << "oCam wb red: " << wb_red_ << "\n";
            }
            break;
        case '2':
            if (wb_red_ > 0)
            {
                camera_->set_control("White Balance Red Component", --wb_red_);
                std::cout << "oCam wb red: " << wb_red_ << "\n";
            }
            break;
        case '3':
            if (wb_blue_ < 250)
            {
                camera_->set_control("White Balance Blue Component", ++wb_blue_);
                std::cout << "oCam wb blue: " << wb_blue_ << "\n";
            }
            break;
        case '4':
            if (wb_blue_ > 0)
            {
                camera_->set_control("White Balance Blue Component", --wb_blue_);
                std::cout << "oCam wb blue: " << wb_blue_ << "\n";
            }
            break;

        case 'e':
            auto_exposure_ = !auto_exposure_;
            camera_->set_control("Exposure, Auto", auto_exposure_);
            std::cout << "oCam auto exposure: " << (auto_exposure_ ? "enabled" : "disabled") << std::endl;
            break;

        default:
            break;
        }
    }
}

oCam_ROS::~oCam_ROS()
{
    delete camera_;
    delete srcImg;
    delete dstImg;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ocam_ros_node");
    oCam_ROS thing;
    return 0;
}
