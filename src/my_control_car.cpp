#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"


bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;
int cnt =0,veloc;
int flag = 0,curve=0;

void sign_callback(const std_msgs::Float32::ConstPtr& msg) {
    if(msg->data == 1)
    {
        ROS_INFO("re phai");    // re pha1 la so 1
        flag = 1;
    }
    else if(msg->data == 0)
    {
        ROS_INFO("re trai");      // re trai la so 0
        flag = 0;
    }
    cnt = 150;
    car->driverCar(detect->getLeftLane(), detect->getRightLane(), 0, flag);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // static long cnt=0;
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
        detect->update(cv_ptr->image);
        if(cnt >130)
        {
            --cnt;
            veloc = 20;
            car->driverCar(detect->getLeftLane(), detect->getRightLane(), veloc, flag);
            ROS_INFO("cnt %d",cnt);
        }
        else if(cnt > 90)
        {
            --cnt;
            veloc = 30;
            car->driverCar(detect->getLeftLane(), detect->getRightLane(), veloc, flag);
            ROS_INFO("cnt %d",cnt);
        }
        else if(cnt > 10)
        {
            --cnt;
            veloc = 40;
            car->driverCar(detect->getLeftLane(), detect->getRightLane(), veloc, flag);
            ROS_INFO("cnt %d",cnt);
        }
        else
        {
            cnt = 0;
            curve =0;
            flag =2;
            veloc = 60;
            if (DetectLane::variance>6){
                cnt = 130;
            }
            car->driverCar(detect->getLeftLane(), detect->getRightLane(), veloc, flag);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // ROS_INFO("%d",cnt);
    // cnt++;
    waitKey(10);
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_control_car");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        ros::NodeHandle nh_get_sign;
        ros::Subscriber number_subscriber = nh_get_sign.subscribe("/sign",1,sign_callback);
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("sudo_image", 1, imageCallback);

        ros::spin();
    } else {
        videoProcess();
    }
    cv::destroyAllWindows();
}