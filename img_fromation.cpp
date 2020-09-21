#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#define CAMERA_READ
//#define VIDEO_READ

#ifdef CAMERA_READ
#include "camera.hpp"
#endif

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MVS_node");
    ros::NodeHandle MVS_node;
    cv::Mat src;
    //bool video_flag;
  
   // infantry_node.param("camera/video_flag", video_flag, false);

#ifdef CAMERA_READ
    camera::Camera infantry_cap(MVS_node);
#endif

#ifdef VIDEO_READ
    std::string video_path;
    infantry_node.param<std::string>("camera/video_path", video_path, "haha");
    cv::VideoCapture *cap = nullptr;
    cap = new cv::VideoCapture(video_path);
#endif

    image_transport::ImageTransport main_cam_image(MVS_node);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("image_raw", 1);

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info;
    sensor_msgs::Image image_msg;
    cam_info.reset(new camera_info_manager::CameraInfoManager(MVS_node, "main_camera", ""));
    if (!cam_info->isCalibrated())
    {
        cam_info->setCameraName("/dev/video0");
        sensor_msgs::CameraInfo cam_info_;
        cam_info_.header.frame_id = image_msg.header.frame_id;
        cam_info_.width = 640;
        cam_info_.height = 480;
        cam_info->setCameraInfo(cam_info_);
    }
    sensor_msgs::CameraInfoPtr camera_info;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

    ros::Rate loop_rate(40);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();


#ifdef CAMERA_READ
    infantry_cap >> src;    
#endif
  
#ifdef VIDEO_READ
    if (!cap->read(src))
    {
        break;
    }
#endif       

    if (src.empty())
    {
        continue;
    }

    cv_ptr->image = src;
    image_msg = *(cv_ptr->toImageMsg());
    image_msg.header.stamp = ros::Time::now();
    image_msg.header.frame_id = "main_camera";
    camera_info = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info->getCameraInfo()));
    camera_info->header.frame_id = image_msg.header.frame_id;
    camera_info->header.stamp = image_msg.header.stamp;
    image_pub.publish(image_msg, *camera_info);
    }

   
#ifdef VIDEO_READ
       if (cap != nullptr)
    {
        delete cap;
    }
#endif     

    return 0;
}
