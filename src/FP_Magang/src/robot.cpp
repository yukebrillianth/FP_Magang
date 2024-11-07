#include <ros/ros.h>
#include <std_msgs/String.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <FP_Magang/ballPos.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <cstdlib>
#include <ctime>

using namespace std;

class Robot
{
public:
    int status;
    bool isRobotCarryingBall = false, imageSent = false;
    float robot_x = 0, robot_y = 0, robot_theta = 0, ball_x = 0, ball_y = 0;
    FP_Magang::PC2BS msg;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher toImagePicker;
    ros::Subscriber sub;
    ros::Subscriber ballPos;

    void ballPosCb(const FP_Magang::ballPos::ConstPtr &ballPosMsg)
    {
        ball_x = msg.bola_x = ballPosMsg->x;
        ball_y = msg.bola_y = ballPosMsg->y;
    }

    void bsCallback(const FP_Magang::BS2PC::ConstPtr &subMsg)
    {
        status = subMsg->status;

        robot_theta = subMsg->th;
        robot_x = (subMsg->enc_right * sin(degreeToRad(45))) + (subMsg->enc_left * sin(degreeToRad(45)));
        robot_y = (subMsg->enc_right * cos(degreeToRad(45))) - (subMsg->enc_left * cos(degreeToRad(45)));

        // Jika robot membawa bola, posisi bola mengikuti posisi robot
        if (isRobotCarryingBall)
        {
            ball_x = robot_x;
            ball_y = robot_y;
        }
        else
        {
            if (robot_x == ball_x && ball_y == robot_y)
            {
                isRobotCarryingBall = true;
            }
        }
    }

    Robot()
    {
        pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);

        /**
         * @brief Send image path to image picker node
         *
         */
        toImagePicker = nh.advertise<std_msgs::String>("/toImagePicker", 50);
        ballPos = nh.subscribe<FP_Magang::ballPos>("/ballPos", 50, &Robot::ballPosCb, this);
        sub = nh.subscribe<FP_Magang::BS2PC>("/bs2pc", 50, &Robot::bsCallback, this);
    }

    char getKey()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

    float degreeToRad(float degrees)
    {
        return degrees * M_PI / 180;
    }

    void setMotorSpeeds(float x, float y, float theta)
    {
        // Tentukan batas lapangan (dengan padding 58px)
        float panjangLapangan = 1016;
        float tinggiLapangan = 716;
        float min_x = 58;                   // Batas kiri
        float max_x = panjangLapangan - 58; // Batas kanan
        float min_y = 58;                   // Batas atas
        float max_y = tinggiLapangan - 58;  // Batas bawah

        float newX = (x * cos(degreeToRad(robot_theta))) - (y * sin(degreeToRad(robot_theta)));
        float newY = (x * sin(degreeToRad(robot_theta))) + (y * cos(degreeToRad(robot_theta)));

        ROS_INFO("\nRobot x: %f y: %f newY: %f newX: %f\n", robot_x, robot_y, newX, newY);
        // if (min_y >= robot_y || max_y <= (newY + robot_y))
        // {
        //     newY = 0;
        // }

        msg.motor1 = (newX * 2 / 3) + (newY * 0) + (theta * 1 / 3);
        msg.motor2 = (newX * -1 / 3) + (newY * -1 / sqrt(3)) + (theta * 1 / 3);
        msg.motor3 = (newX * -1 / 3) + (newY * sqrt(3) / 3) + (theta * 1 / 3);

        if (isRobotCarryingBall)
        {
            ball_x = robot_x;
            ball_y = robot_y;
        }
        msg.bola_x = ball_x;
        msg.bola_y = ball_y;
    }

    float calcDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    }

    void kickBall(float distance)
    {
        if (isRobotCarryingBall)
        {
            float kick_angle = degreeToRad(robot_theta);
            ball_x += distance * cos(kick_angle);
            ball_y += distance * sin(kick_angle);
            isRobotCarryingBall = false;
        }
    }

    void status1()
    {
        ROS_INFO("Press 'W', 'A', 'S', 'D' to control, 'Q' to quit, 'E' to rotate CW, 'R' to rotate CCW, 'Z' to pick up ball, 'X' to release ball, 'O' to kick 100px, 'P' to kick 300px.");
        char key = getKey();
        float x = 0, y = 0, theta = 0;
        if (key == 'w')
            y = -1000;
        else if (key == 's')
            y = 1000;
        else if (key == 'a')
            x = 1000;
        else if (key == 'd')
            x = -1000;
        else if (key == 'e')
            theta = 1000;
        else if (key == 'r')
            theta = -1000;
        else if (key == 'z')
        {
            // Ambil bola jika dalam radius 100px
            if (calcDistance(robot_x, robot_y, ball_x, ball_y) <= 10)
            {
                isRobotCarryingBall = true;
                ball_x = robot_x;
                ball_y = robot_y;
            }
        }
        else if (key == 'x')
            isRobotCarryingBall = false;
        else if (key == 'o')
            kickBall(100); // Tendang bola sejauh 100px
        else if (key == 'p')
            kickBall(300); // Tendang bola sejauh 300px
        else if (key == 'q')
        {
            ROS_INFO("Exiting...");
            ros::shutdown();
            return;
        }

        setMotorSpeeds(x, y, theta);
        pub.publish(msg);
    }

    void status2()
    {
        // Pick image with idx 0 to 2
        string images[3] = {
            "/home/yukebrillianth/KULIAH/ROBOTIK/MAGANG/FP/FP_Magang/assets/bola1.jpg",
            "/home/yukebrillianth/KULIAH/ROBOTIK/MAGANG/FP/FP_Magang/assets/bola2.jpg",
            "/home/yukebrillianth/KULIAH/ROBOTIK/MAGANG/FP/FP_Magang/assets/bola3.jpg"};
        int randomIndex = rand() % 3;
        std_msgs::String imgPath;
        imgPath.data = images[randomIndex];
        toImagePicker.publish(imgPath);

        // receive coordinate
        float deltaX = ball_x - robot_x;
        float deltaY = ball_y - robot_y;
        robot_theta = atan2(deltaY, deltaX) * 180 / M_PI; // Menghitung sudut dalam derajat
        float speed = 1000;

        // Kecepatan tetap
        setMotorSpeeds(speed * cos(degreeToRad(robot_theta)), speed * sin(degreeToRad(robot_theta)), robot_theta);
        pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    // srand(static_cast<unsigned int>(time(0)));
    ros::init(argc, argv, "robot_publisher");
    Robot robot;
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        if (robot.status == 1)
        {
            robot.status1();
        }
        else if (robot.status == 2)
        {
            robot.status2();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
