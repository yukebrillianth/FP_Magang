#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace std;
using namespace cv;

ros::Publisher image_pub;

void imagePickerCb(const std_msgs::Bool &subMsg)
{
    // Generate rand
    srand(static_cast<unsigned int>(time(0)));
    int randomIndex = rand() % 3;

    Mat frame = imread("./assets/bola" + to_string(randomIndex + 1) + ".jpg");

    if (frame.empty())
    {
        ROS_ERROR("Could not open or find the image at path: ./assets/bola%d.jpg", randomIndex + 1);
        return;
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    image_pub.publish(img_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_picker");
    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/toImageProcessor", 1);

    ros::Subscriber sub = nh.subscribe("/toImagePicker", 1, imagePickerCb);

    ros::spin();
    return 0;
}