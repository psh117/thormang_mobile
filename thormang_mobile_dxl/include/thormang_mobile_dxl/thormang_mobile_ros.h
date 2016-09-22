
// STD
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// USER
#include "thormang_mobile_dxl/dynamixel_pro_device.h"

class ThormangMobileJointROS
{
    ros::Publisher jointPub_;
    ros::Subscriber jointSub_;

    motor_device *motorDevPtr_;
    int motorNum_;

    sensor_msgs::JointState jointMsg_;

    void jointCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        int size = msg->name.size();
        for(int i=0; i<size; i++)
        {
            int motorIndex = -1;
            for(int j=0; j<motorNum_; j++)
            {
                if(msg->name[i] == motorDevPtr_[j].joint_name)
                {
                    motorIndex = j;
                    break;
                }
            }
            if(motorIndex != -1)
            {
                // mutex lock
                motorDevPtr_[motorIndex].goal_position = msg->position[i];
                motorDevPtr_[motorIndex].goal_velocity = msg->velocity[i];
                // mutex unlock
            }
        }
    }

public:
    ThormangMobileJointROS(ros::NodeHandle &nh, motor_device *motorDevPtr, int motorNum) : motorDevPtr_(motorDevPtr), motorNum_(motorNum)
    {
        jointPub_ = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
        jointSub_ = nh.subscribe("joint_set", 1, &ThormangMobileJointROS::jointCallback, this);
        for(int i=0; i<motorNum_; i++)
        {
            jointMsg_.name.push_back(motorDevPtr_[i].joint_name);
        }
        jointMsg_.position.resize(motorNum_);
        jointMsg_.velocity.resize(motorNum_);
        jointMsg_.effort.resize(motorNum_);
    }
    void publish()
    {
        for(int i=0; i<motorNum_; i++)
        {
            jointMsg_.position[i] = motorDevPtr_[i].present_position;
            jointMsg_.velocity[i] = motorDevPtr_[i].present_velocity;
            jointMsg_.effort[i] = motorDevPtr_[i].present_current;
        }

        jointPub_.publish(jointMsg_);
    }
};
