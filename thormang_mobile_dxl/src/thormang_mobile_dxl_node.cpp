
// STD
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// USER
#include "thormang_mobile_dxl/dynamixel_pro_device.h"
#include "thormang_mobile_dxl/dynamixel_pro_comm.h"
#include "thormang_mobile_dxl/thormang_mobile_ros.h"



enum MOTOR_NAME {LEFT_WHEEL = 0, RIGHT_WHEEL, LIDAR_MOTOR, MOTOR_NUM};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_mobile_dxl");
    ros::NodeHandle nh("/thormang_mobile");

    ros::Rate loop_rate(100);


    // motor id setting
    motor_device motor[MOTOR_NUM];

    motor[LEFT_WHEEL].id = 16;
    motor[LEFT_WHEEL].mode = OP_VELOCITY;
    motor[LEFT_WHEEL].joint_name = "left_wheel_joint";
    motor[LEFT_WHEEL].offset_position = 0.0;

    motor[RIGHT_WHEEL].id = 17;
    motor[RIGHT_WHEEL].mode = OP_VELOCITY;
    motor[RIGHT_WHEEL].joint_name = "right_wheel_joint";
    motor[RIGHT_WHEEL].offset_position = 0.0;

    motor[LIDAR_MOTOR].id = 18;
    motor[LIDAR_MOTOR].mode = OP_POSITION;
    motor[LIDAR_MOTOR].joint_name = "neck_lidar_joint";
    motor[LIDAR_MOTOR].offset_position = -27.5 * DEG2RAD;

    // ROS connectivity
    ThormangMobileJointROS ROSComm(nh, motor, MOTOR_NUM);
    DynamxelProAdaptor dxlAdaptor("/dev/ttyUSB0", motor, MOTOR_NUM);

    // Initializing port
    if(dxlAdaptor.initPort(3000000) == -1) return -1;

    // Model get
    if(dxlAdaptor.getModel() == -1) return -1;

    // Torque ON!
    dxlAdaptor.torqueOn(1);
    dxlAdaptor.syncReadReady();

    while(ros::ok())
    {

        // Parameter Update
        dxlAdaptor.parameterUpdate();

        // Packet Exchange
        dxlAdaptor.packetExchange();

        // Get Datas
        dxlAdaptor.getDatas();

        // just for checking everything is going well.
        // ROS_INFO("motor1 - pos : %.2lf rad, vel : %.2lf rad/s",motor[0].present_position, motor[0].present_velocity);

        ROSComm.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    dxlAdaptor.torqueOn(0);


    return 0;
}
