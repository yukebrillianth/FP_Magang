#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <math.h>

using namespace std;

class RobotController
{
private:
    ros::NodeHandle nh;
    ros::Publisher pc2bs_pub;
    ros::Subscriber bs2pc_sub;

    float interval = 0;

    /**
     * @brief Untuk konversi derajat ke radian
     *
     * @param deg input derajat
     * @return float
     */
    float degToRad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float calcDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    }

    float sigmoid(float interval)
    {
        return 1.0 / (1.0 + exp(-2.5 * (2 * interval - 1)));
    }

    void setMotorSpeeds(float x, float y, float theta)
    {
        if (keyControl)
        {
            float ct = -bsMsg.th; // last theta
            float sinus = sin(ct * M_PI / 180);
            float cosinus = cos(ct * M_PI / 180);

            float cx = x;
            float cy = y;

            x = (cx * cosinus) + (cy * -sinus);
            y = (cx * sinus) + (cy * cosinus);
        }

        if (!keyControl && interval < 1)
        {
            interval += 0.02;
        }
        else
        {
            interval = 1;
        }

        if (((robot_x + (x / 50)) + 27.7430 < 0) || ((robot_x + (x / 50)) - 926.866219 > 0))
        {
            x = 0;
        }
        if (((robot_y + (y / 50)) + 31.4311 < 0) || ((robot_y + (y / 50)) - 627.356796 > 0))
        {
            y = 0;
        }

        (round(x) == 0) ? x = 0 : x = x;
        (round(y) == 0) ? y = 0 : y = y;
        (round(theta) == 0) ? theta = 0 : theta = theta;

        if ((round(x) == 0) && (round(y) == 0) && (round(theta / 10) == 0))
        {
            interval = 0;
            // if (bsMsg.status == 4)
            // {
            //     phase++;
            // }
            return;
        }

        msg.motor1 = ((x * 2 / 3) + (y * 0) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor2 = ((x * -1 / 3) + (y * (sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor3 = ((x * -1 / 3) + (y * -(sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);

        // Update posisi bola jika membawa bola
        if (isGrab)
        {
            msg.bola_x = robot_x;
            msg.bola_y = robot_y;
        }
        else
        {
            msg.bola_x = ball_x;
            msg.bola_y = ball_y;
        }
    }

    void updateRobotPos(float enc_left, float enc_right)
    {
        robot_x = (enc_right * sin(45 * M_PI / 180)) + (enc_left * sin(45 * M_PI / 180));
        robot_y = (enc_right * cos(45 * M_PI / 180)) - (enc_left * cos(45 * M_PI / 180));
    }

    void kickBall(float distance)
    {
        if (isGrab)
        {
            float theta = bsMsg.th;
            float x = robot_x + distance * sin(degToRad(theta));
            float y = robot_y + distance * cos(degToRad(theta));

            if (x + 27.7430 < 0)
            {
                x = -27.7430;
            }
            else if (x - 926.866219 > 0)
            {
                x = 926.866219;
            }

            if (y + 31.4311 < 0)
            {
                y = -31.4311;
            }
            else if (y - 627.356796 > 0)
            {
                y = 627.356796;
            }

            ball_x = x;
            ball_y = y;
            isGrab = false;

            msg.bola_x = x;
            msg.bola_y = y;
            pc2bs_pub.publish(msg);
            ros::spinOnce();
        }
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

public:
    FP_Magang::PC2BS msg;
    FP_Magang::BS2PC bsMsg;
    float robot_x = 0, robot_y = 0, robot_theta = 0;
    float ball_x = 0, ball_y = 0;
    bool isGrab = false;
    bool keyControl = false;

    RobotController()
    {
        // Inisialisasi publisher dan subscriber
        pc2bs_pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 10);
        bs2pc_sub = nh.subscribe("/bs2pc", 10, &RobotController::baseStationHandler, this);
    }

    void run()
    {
        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            ros::spinOnce();
            if (bsMsg.status == 1)
            {
                keyControl = true;
                status1();
            }
            loop_rate.sleep();
        }
    }

    void baseStationHandler(const FP_Magang::BS2PC &msg)
    {
        bsMsg = msg;
    }

    void status1()
    {
        ROS_INFO("Control: W/A/S/D (move), Q/E (rotate), Z (grab ball), X (release ball), O (kick 100px), P (kick 300px), K (quit)");

        // Dapatkan input keyboard
        char key = getKey();
        float x = 0, y = 0, theta = 0;

        ROS_INFO("Key: %d", key);

        // Proses input
        if (key == 'w')
        {
            y = 1000;
        }
        else if (key == 's')
        {
            y = -1000;
        }
        else if (key == 'a')
        {
            x = 1000;
        }
        else if (key == 'd')
        {
            x = -1000;
        }
        else if (key == 'e')
        {
            theta = -1000;
        }
        else if (key == 'q')
        {
            theta = 1000;
        }
        else if (key == 'z')
        {
            if (calcDistance(robot_x, robot_y, ball_x, ball_y) <= 50)
            {
                isGrab = true;
                msg.bola_x = robot_x;
                msg.bola_y = robot_y;
            }
        }
        else if (key == 'x')
            isGrab = false;
        else if (key == 'o')
            kickBall(100); // Tendang bola sejauh 100px
        else if (key == 'p')
        {
            kickBall(300); // Tendang bola sejauh 300px
        }
        else if (key == 'k')
        {
            ROS_INFO("Exiting...");
            ros::shutdown();
            return;
        }

        // Atur kecepatan motor
        setMotorSpeeds(x, y, theta);

        // Publish pesan
        pc2bs_pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");
    RobotController controller;
    controller.run();
    return 0;
}