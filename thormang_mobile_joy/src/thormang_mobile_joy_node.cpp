
#include <string>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

using namespace std;
ros::Publisher jointPub;

enum button_map {BTN_A = 0, BTN_B, BTN_X, BTN_Y, BTN_L1, BTN_R1, BTN_BACK, BTN_START, BTN_HOME, BTN_LSTICK, BTN_RSTICK};

void joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    const double y_axis_speed = M_PI / 2; // 90 deg / s
    const double x_axis_speed = M_PI / 4; // 45 deg / s

    double left_x = msg->axes[0]; // -1.0(right) ~ 1.0(left)
    double left_y = msg->axes[1]; // -1.0(down) ~ 1.0(up)
    double right_x = msg->axes[3]; // -1.0(right) ~ 1.0(left)
    double right_y = msg->axes[4]; // -1.0(down) ~ 1.0(up)

    double left_wheel_velocity = left_y * y_axis_speed - right_x * x_axis_speed;
    double right_wheel_velocity = -(left_y * y_axis_speed + right_x * x_axis_speed);

    if(fabs(left_wheel_velocity) < 0.2) left_wheel_velocity = 0;
    if(fabs(right_wheel_velocity) < 0.2) right_wheel_velocity = 0;

    sensor_msgs::JointState jointMsg;
    jointMsg.name.push_back(string("left_wheel_joint"));
    jointMsg.velocity.push_back(left_wheel_velocity);

    jointMsg.name.push_back(string("right_wheel_joint"));
    jointMsg.velocity.push_back(right_wheel_velocity);

    jointMsg.position.resize(2);

    if(msg->buttons[BTN_A])
    {
        jointMsg.name.push_back(string("neck_lidar_joint"));
        jointMsg.velocity.push_back(0.785398);
        jointMsg.position.push_back(1.570796327);
    }
    else if (msg->buttons[BTN_B])
    {
        jointMsg.name.push_back(string("neck_lidar_joint"));
        jointMsg.velocity.push_back(0.785398);
        jointMsg.position.push_back(-1.570796327);
    }
    jointPub.publish(jointMsg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_mobile_joy");
    ros::NodeHandle nh("/thormang_mobile");

    jointPub = nh.advertise<sensor_msgs::JointState>("joint_set", 1);
    ros::Subscriber joySub = nh.subscribe("/joy", 1, joyCallback);

    ros::spin();

}

