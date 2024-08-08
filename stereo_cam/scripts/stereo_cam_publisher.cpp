#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class Camera
{
    ros::NodeHandle nh_;

public:
    
    void video_cap_setting()
    {
        cap.open(0);
        cap.set(CAP_PROP_FRAME_WIDTH,img_w);
        cap.set(CAP_PROP_FRAME_HEIGHT, img_h);
        cap.set(CAP_PROP_FPS, fps);
        if(!auto_exposure)
        {
            cap.set(CAP_PROP_AUTO_EXPOSURE,0.25);
            cap.set(CAP_PROP_EXPOSURE,exposure_time);
            cap.set(CAP_PROP_AUTO_WB,auto_white_balance);
        }
    };

    void CameraCallback()
    {
        if(!cap.isOpened())
        {
            ROS_ERROR("Camera is not opened\n");
        }

        if(!cap.read(cv_frame))
        {
            ROS_WARN("Detect no Image\n");
        }
        else
        {   
            std_msgs::Header header; // empty header
            header.seq = img_counter++; // user defined counter
            header.stamp = ros::Time::now(); // time
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_frame);
            img_bridge.toImageMsg(img_msg);
            rawpub.publish(img_msg);
        }
    };

    Camera():it_(nh_)
    {
        nh_.getParam("img_width", img_w);
        nh_.getParam("img_height", img_h);
        nh_.getParam("rame_rate", fps);
        nh_.getParam("auto_exposure", auto_exposure);
        nh_.getParam("exposure_time", exposure_time);
        nh_.getParam("auto_white_balance", auto_white_balance);
        if(nh_.hasParam("topic_name"))
        {
            nh_.getParam("topic_name",pubstr);
            nh_.getParam("queue_size", queue_size);
        }

        video_cap_setting();
        
        rawpub = it_.advertise(pubstr,queue_size,&Camera::CameraCallback);
    };

private:
    int img_w{1280};
    int img_h{480};
    int fps{30};
    bool auto_exposure{true};
    int exposure_time{-6};
    bool auto_white_balance{true};
    image_transport::Publisher rawpub;
    VideoCapture cap;
    string pubstr{"/stereo_camera/raw"};
    int queue_size{1};
    cv::Mat cv_frame;
    cv_bridge::CvImage img_bridge;
    int img_counter{0};
    sensor_msgs::Image img_msg;
    image_transport::ImageTransport it_;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_camera");
    Camera camera_node;
    ros::spin();
    return 0;
}