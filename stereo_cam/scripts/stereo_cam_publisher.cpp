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


int main(int argc, char** argv)
{   
    //node init
    ros::init(argc, argv, "publish_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    int img_w{1280};
    int img_h{480};
    int fps{30};
    int queue_size{10};
    bool auto_exposure{true};
    int exposure_time{-6};
    bool auto_white_balance{true};
    string pubstr{"/stereo_camera/raw"};
    string image_format{"mjpg"};
    cv_bridge::CvImage img_bridge;
    int img_counter{0};
    cv::Mat cv_frame;
    sensor_msgs::Image img_msg;
    string cam_port_name{"/dev/video0"};


    //load param
    nh.getParam("img_width", img_w);
    nh.getParam("img_height", img_h);
    nh.getParam("frame_rate", fps);
    nh.getParam("auto_exposure", auto_exposure);
    nh.getParam("exposure_time", exposure_time);
    nh.getParam("auto_white_balance", auto_white_balance);
    if(nh.hasParam("topic_name"))
    {
        nh.getParam("topic_name",pubstr);
    }
    if(nh.hasParam("image_format"))
    {
        nh.getParam("image_format",image_format);
    }
    if(nh.hasParam("cam_port_name"))
    {
        nh.getParam("cam_port_name",cam_port_name);
    }
    if(nh.hasParam("queue_size"))
    {
        nh.getParam("queue_size", queue_size); 
    }


    //camera init
    //cv::VideoCapture cap;
    //cap.open(0);
    //cap.open(0);
    cv::VideoCapture cap(cam_port_name,cv::CAP_V4L2);
    if (image_format == "mjpg")
        {std::cout << "Image Format :MJPG: " << cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M>
    else
        {std::cout << "Image Format :YUYV: " << cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y>
    //cap.set(CAP_PROP_FPS,fps);
    cap.set(3,img_w);
    cap.set(4, img_h);
    std::cout<<"fps: "<< cap.get(5);
    //std::cout<<"format: "<< cap.get(CAP_PROP_FOURCC);
    std::cout<<"img_w: "<< cap.get(3);
    std::cout<<"img_h: "<< cap.get(4);
    if(!cap.isOpened())
        {
            ROS_ERROR("Camera is not opened\n");
        }
    else
    {
        ROS_INFO("Camera opened\n");
    }
    cap.set(CAP_PROP_AUTO_EXPOSURE,3);
    if(!auto_exposure)
    {
        std::cout<<"EXPOSURE RESET"<<endl;
        //cap.set(CAP_PROP_AUTO_EXPOSURE,3);
        cap.set(CAP_PROP_AUTO_EXPOSURE,1);
        cap.set(CAP_PROP_EXPOSURE,exposure_time);
    }
    cap.set(CAP_PROP_AUTO_WB,3);
    if(!auto_white_balance)
    {
        std::cout<<"WHITEBALANCE RESET"<<endl;
        cap.set(CAP_PROP_AUTO_WB,1);
        cap.set(CAP_PROP_WB_TEMPERATURE,6500);
    }
    std::cout<<"Auto_Exposure"<<cap.get(CAP_PROP_AUTO_EXPOSURE)<<endl;
    std::cout<<"Exposure_Time"<<cap.get(CAP_PROP_EXPOSURE)<<endl;
    std::cout<<"AUTO_WHITEBALANCE"<<cap.get(CAP_PROP_AUTO_WB)<<endl;
    std::cout<<"WB_TEMP"<<cap.get(CAP_PROP_WB_TEMPERATURE)<<endl;
    //publisher init
    image_transport::Publisher rawpub = it.advertise(pubstr,queue_size);


    //start loop
    ros::Rate loop_rate(fps);
    while (ros::ok())
        {   
            if(!cap.read(cv_frame))
            {
                ROS_WARN("Detect no Image\n");
            }
            else
            {   //cap >> cv_frame;
                //std::cout<<"size"<<cv_frame.size()<<endl;
                cv::cvtColor(cv_frame, cv_frame, CV_BGR2RGB);
                std_msgs::Header header; // empty header
                header.seq = img_counter++; // user defined counter
                header.stamp = ros::Time::now(); // time
                //std::cout<<header.stamp<<endl;
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_fram>
                img_bridge.toImageMsg(img_msg);
                rawpub.publish(img_msg);

            }

            ros::spinOnce();
            loop_rate.sleep();
        }

    return 0;
}
