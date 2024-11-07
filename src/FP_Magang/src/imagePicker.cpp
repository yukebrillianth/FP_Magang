#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace std;
using namespace cv;

ros::Publisher image_pub;

void imagePickerCb(const std_msgs::String::ConstPtr &subMsg)
{
    Mat frame = imread(subMsg->data);

    if (frame.empty())
    {
        ROS_ERROR("Could not open or find the image at path: %s", subMsg->data.c_str());
        return;
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    image_pub.publish(img_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_picker");
    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/toImageProcessor", 50);

    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/toImagePicker", 50, imagePickerCb);

    ros::spin();
    return 0;
}