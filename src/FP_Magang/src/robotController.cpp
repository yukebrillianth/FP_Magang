#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <FP_Magang/ballPos.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

using namespace std;

class RobotController
{
private:
    ros::NodeHandle nh;
    ros::Publisher pc2bs_pub;
    ros::Subscriber bs2pc_sub;
    ros::Subscriber ballPosition_sub;
    ros::Publisher toImagePicker_pub;

    float interval = 0;

    /**
     * @brief Konversi derajat ke radian
     *
     * @param deg
     * @return float
     */
    float degToRad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    /**
     * @brief Update local robot pos
     *
     * @param enc_left
     * @param enc_right
     */
    void updateRobotPos(float enc_left, float enc_right)
    {
        robot_x = (enc_right * sin(45 * M_PI / 180)) + (enc_left * sin(45 * M_PI / 180));
        robot_y = (enc_right * cos(45 * M_PI / 180)) - (enc_left * cos(45 * M_PI / 180));
    }

    /**
     * @brief Bikin s curve
     *
     * @param interval
     * @return float
     */
    float sigmoid(float interval)
    {
        return 1.0 / (1.0 + exp(-2.5 * (2 * interval - 1)));
    }

    /**
     * @brief Jarak 2 titik
     *
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return float
     */
    float calcDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    }

    /**
     * @brief Read wasd
     *
     * @return char
     */
    char getKey()
    {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        new_tio.c_cc[VMIN] = 1; // Minimum number of characters to read
        new_tio.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval timeout = {0, 0};

        char ch;
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0)
        {
            read(STDIN_FILENO, &ch, 1);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

        return ch;
    }

    /**
     * @brief Base station sub cb
     *
     * @param msg
     */
    void baseStationHandler(const FP_Magang::BS2PC &msg)
    {

        if (msg.status == 1)
        {
            keyControl = true;
        }

        if (msg.status == 3)
        {
            dest_x = msg.tujuan_x;
            dest_y = msg.tujuan_y;
        }

        if (bsMsg.status != msg.status && (msg.status == 2 || msg.status == 4))
        {
            step = 0;
            std_msgs::Bool m;
            m.data = true;
            toImagePicker_pub.publish(m);
        }

        bsMsg = msg;
        updateRobotPos(msg.enc_left, msg.enc_right);
    }

    /**
     * @brief Ball pos sub cb
     *
     * @param msg
     */
    void ballPosHandler(const FP_Magang::ballPos &msg)
    {
        // Set ball pos to image pos
        ball_x = msg.x;
        ball_y = msg.y;

        if (bsMsg.status == 2)
        {
            dest_x = ball_x;
            dest_y = ball_y;
        }
        else if (bsMsg.status == 4)
        {
            float x = ball_x - robot_x;
            float y = ball_y - robot_y;

            dest_x = ball_x - 100 * cos(atan2(y, x));
            dest_y = ball_y - 100 * sin(atan2(y, x));

            step = 1;
        }
    }

public:
    FP_Magang::BS2PC bsMsg;
    FP_Magang::PC2BS msg;

    float robot_x = 0, robot_y = 0;
    float ball_x = 0, ball_y = 0;
    float dest_x = 0, dest_y = 0;

    int step = 0;
    bool isGrab = false;
    bool keyControl = false;

    RobotController()
    {
        // Init pub sub
        pc2bs_pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);
        bs2pc_sub = nh.subscribe("/bs2pc", 10, &RobotController::baseStationHandler, this);
        toImagePicker_pub = nh.advertise<std_msgs::Bool>("/toImagePicker", 1);
        ballPosition_sub = nh.subscribe("/ballPosition", 10, &RobotController::ballPosHandler, this);
    }

    /**
     * @brief run controller
     *
     */
    void run()
    {
        float svDeg = 0;

        ros::Rate rate(50);
        while (ros::ok())
        {
            ros::spinOnce();
            if (bsMsg.status == 1)
            {
                keyControl = true;
                status1();
                setMotorSpeed(0, 0, 0);
            }
            else if (bsMsg.status == 2)
            {
                keyControl = false;
                msg.bola_x = ball_x;
                msg.bola_y = ball_y;
                pc2bs_pub.publish(msg);
                status2();
            }
            else if (bsMsg.status == 3)
            {
                keyControl = false;
                status3();
            }
            else if (bsMsg.status == 4)
            {
                keyControl = false;
                status4(svDeg);
            }
            rate.sleep();
        }
    }

    void kickBall(float distance)
    {
        float ct = bsMsg.th;
        float x = ball_x + (distance * sin(degToRad(ct)));
        float y = ball_y + (distance * cos(degToRad(ct)));

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

        FP_Magang::PC2BS msg;
        msg.bola_x = x;
        msg.bola_y = y;
        pc2bs_pub.publish(msg);
        ros::spinOnce();
    }

    void setMotorSpeed(float x, float y, float theta)
    {
        if (keyControl)
        {
            float ct = -bsMsg.th;
            float sinus = sin(degToRad(ct));
            float cosinus = cos(degToRad(ct));

            float tempX = x;
            float tempY = y;

            x = (tempX * cosinus) + (tempY * -sinus);
            y = (tempX * sinus) + (tempY * cosinus);
        }

        if (!keyControl && interval < 1)
        {
            interval += 0.02;
        }
        else
        {
            interval = 1;
        }

        // Pembatas lapangan
        int fieldWidth = 1016, fieldHeight = 716;
        float batasX = fieldWidth - 58 * 2;
        float batasY = fieldHeight - 58 * 2;
        if (((robot_x + (x / 50)) < 0) || ((robot_x + (x / 50)) > batasX))
        {
            x = 0;
        }
        if (((robot_y + (y / 50)) < 0) || ((robot_y + (y / 50)) > batasY))
        {
            y = 0;
        }

        (round(x) == 0) ? x = 0 : x = x;
        (round(y) == 0) ? y = 0 : y = y;
        (round(theta) == 0) ? theta = 0 : theta = theta;

        if ((round(x) == 0) && (round(y) == 0) && (round(theta / 10) == 0))
        {
            interval = 0;
            if (bsMsg.status == 4)
            {
                step++;
            }
            return;
        }

        FP_Magang::PC2BS msg;
        msg.motor1 = ((x * 2 / 3) + (y * 0) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor2 = ((x * -1 / 3) + (y * (sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);
        msg.motor3 = ((x * -1 / 3) + (y * -(sqrt(3) / 3)) + (theta * 1 / 3)) * sigmoid(interval);

        msg.bola_x = ball_x;
        msg.bola_y = ball_y;
        pc2bs_pub.publish(msg);
        ros::spinOnce();
    }

    void status1()
    {
        ROS_INFO("Control: W/A/S/D (move), Q/E (rotate), Z (grab ball), X (release ball), O (kick 100px), P (kick 300px), K (quit)");

        char key = getKey();

        switch (key)
        {
        case 'w': // maju
            setMotorSpeed(0, 1000, 0);
            break;
        case 's': // mundur
            setMotorSpeed(0, -1000, 0);
            break;
        case 'a': // kiri
            setMotorSpeed(1000, 0, 0);
            break;
        case 'd': // kanan
            setMotorSpeed(-1000, 0, 0);
            break;
        case 'q': // berlawanan jarum jam
            setMotorSpeed(0, 0, 1000);
            break;
        case 'e': // searah jarum jam
            setMotorSpeed(0, 0, -1000);
            break;
        case 'z': // ambil bola jika jarak <= 50
            if (calcDistance(robot_x, robot_y, ball_x, ball_y) <= 50)
            {
                isGrab = true;
            }
            break;
        case 'x': // lepas bola
            isGrab = false;
            break;
        case 'p': // tendang 300 px
            if (isGrab)
            {
                isGrab = false;
                kickBall(300);
            }
            break;
        case 'o': // tendang 100px
            if (isGrab)
            {
                isGrab = false;
                kickBall(100);
            }
            break;
        case 'k': // keluar
            ROS_INFO("Exiting...");
            ros::shutdown();
            break;
        }

        if (isGrab)
        {
            ball_x = robot_x;
            ball_y = robot_y;
            msg.bola_x = ball_x;
            msg.bola_y = ball_y;
            pc2bs_pub.publish(msg);
        }
        /**
         * @brief Gak jadi ges, gak bisa dilepas
         *
         */
        // else if (calcDistance(robot_x, robot_y, ball_x, ball_y) <= 10)
        // else if (robot_x == ball_x && robot_y == ball_y)
        // {
        //     isGrab = true;
        // }
    }

    void status2()
    {
        float x = dest_x - robot_x;
        float y = dest_y - robot_y;

        if (x == 0 && y == 0)
            return;

        // mendapatkan arah teta sesuai robot
        float th = 90 - degToRad(atan2(y, x));

        setMotorSpeed(x, y, (th - bsMsg.th));
    }

    void status3()
    {
        float x = dest_x - robot_x;
        float y = dest_y - robot_y;

        if (x == 0 && y == 0)
            return;

        // mendapatkan arah theta sesuai robot
        float th = 90 - degToRad(atan2(y, x));

        setMotorSpeed(x, y, (th - bsMsg.th));
    }

    void status4(float &svDeg)
    {
        float x;
        float y;
        float th;

        if (step == 1)
        {
            keyControl = false;
            x = dest_x - robot_x;
            y = dest_y - robot_y;
            th = 90 - (degToRad(atan2(y, x)));
            svDeg = bsMsg.th;
            setMotorSpeed(x, y, (th - (bsMsg.th)));
        }
        else if (step == 2)
        {
            if (abs((bsMsg.th - svDeg) / 720) >= 1)
            {
                step = 3;
            }

            keyControl = true;
            x = M_PI * 100;
            y = 0;
            th = 360;
            setMotorSpeed(x, y, -th);
        }
        else if (step == 3)
        {
            keyControl = false;
            x = ball_x - robot_x;
            y = ball_y - robot_y;
            th = 0;
            svDeg = 0;
            setMotorSpeed(x, y, th);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");

    RobotController robot;
    robot.run();

    return 0;
}