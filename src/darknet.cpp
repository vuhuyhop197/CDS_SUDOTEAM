#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "wrapper_cpp.h"
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <iostream>

/************************************************************************/
std_msgs::Float32 sign_pub;
ros::Publisher sent_sign;
/************************************************************************/
float *output;
char *cfgfile = "/home/vhop/CuocDuaSo/src/ROS_Package_example/lane_detect/darknet_wrapper/darknet/yolo-obj.cfg";
char *weight = "/home/vhop/CuocDuaSo/src/ROS_Package_example/lane_detect/darknet_wrapper/darknet/traffic_sign_board_v2.weights";
float thresh = 0.4;
int hits;
box **outboxes = new box *;
float **outprobs = new float *;
int **outclasses = new int *;
int flag;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        IplImage ilp = cv_ptr->image;
        Darknet::getInstance().darknet_detect_img(&ilp, thresh, &hits, outboxes, outprobs, outclasses);
        /********************************** test zone ********************************/
        //outclasses = 1 turn right, outclasses = 0 turn left
        if (**outprobs * 100 > 30)
        {
            if (**outclasses)
            {
                ROS_INFO("turn right");
                flag = 1;
                sign_pub.data = 1;
                sent_sign.publish(sign_pub);
            }
            else
            {
                ROS_INFO("turn left");
                flag = 0;
                sign_pub.data = 0;
                sent_sign.publish(sign_pub);
            }
        }

        /********************************************************************************/
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    cv::waitKey(10);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    Darknet::getInstance(cfgfile, weight);
    cv::startWindowThread();

    ros::NodeHandle nh, nh_pub_sign;

    sent_sign = nh_pub_sign.advertise<std_msgs::Float32>("/sign", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("sudo_image", 1, imageCallback);
    ros::spin();
    cv::destroyAllWindows();
}
