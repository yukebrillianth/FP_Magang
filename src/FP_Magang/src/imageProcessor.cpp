#include <ros/ros.h>
#include <FP_Magang/ballPos.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

ros::Publisher ball_pos_pub;

void imageProcessorCb(const sensor_msgs::Image::ConstPtr &img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat frame_resized, frame_hsv, frame_thres;
    int h[2] = {0, 32}, s[2] = {80, 255}, v[2] = {64, 255};

    resize(cv_ptr->image, frame_resized, Size(900, 600));

    // Convert RGB to HSV
    cvtColor(frame_resized, frame_hsv, COLOR_BGR2HSV);
    inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_thres);

    vector<vector<Point>> contours;
    findContours(frame_thres, contours, RETR_TREE, CHAIN_APPROX_NONE);

    if (!contours.empty())
    {
        // Get Largest Contour Area
        size_t largestContourAreaI = 0;
        double largestArea = contourArea(contours[0]);

        // Get largest contour area
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);

            if (area > largestArea && area < 4000.0)
            {
                largestArea = area;
                largestContourAreaI = i;
            }
        }

        Moments m = moments(contours[largestContourAreaI]);
        double mArea = m.m00;
        if (mArea != 0)
        {
            double x = m.m10 / mArea;
            double y = m.m01 / mArea;

            // Posisi bola
            Point2f ballPosition(x, y);
            circle(frame_resized, ballPosition, 3, Scalar(0, 0, 255), -1);

            Point2f circleLine;
            float BallRadius;

            // Hitung min enclosing circle
            minEnclosingCircle(contours[largestContourAreaI], circleLine, BallRadius);
            circle(frame_resized, circleLine, (int)BallRadius, Scalar(0, 255, 255), 2);

            FP_Magang::ballPos ball_pos_msg;
            ball_pos_msg.x = x;
            ball_pos_msg.y = y;
            ball_pos_pub.publish(ball_pos_msg);

            ROS_INFO("Ball position: x = %f, y = %f", x, y);
        }
        imshow("Resized Image", frame_resized);
        imshow("Thresholded Image", frame_thres);
        waitKey(24);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ball_pos_pub = nh.advertise<FP_Magang::ballPos>("/ballPosition", 1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/toImageProcessor", 1, imageProcessorCb);

    ros::spin();
    return 0;
}