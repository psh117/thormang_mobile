
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

class ThormangMobileJointROS
{
    ros::Publisher jointPub_;
    ros::Subscriber jointSub_;

    motor_device *motorDevPtr_;

    sensor_msgs::JointState jointMsg_;

    void jointCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        int size = msg->name.size();
        for(int i=0; i<size; i++)
        {
            int motorIndex = -1;
            for(int j=0; j<MOTOR_NUM; j++)
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
    ThormangMobileJointROS(ros::NodeHandle &nh, motor_device *motorDevPtr) : motorDevPtr_(motorDevPtr)
    {
        jointPub_ = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
        jointSub_ = nh.subscribe("joint_set", 1, &ThormangMobileJointROS::jointCallback, this);
        for(int i=0; i<MOTOR_NUM; i++)
        {
            jointMsg_.name.push_back(motorDevPtr_[i].joint_name);
        }
        jointMsg_.position.resize(MOTOR_NUM);
        jointMsg_.velocity.resize(MOTOR_NUM);
        jointMsg_.effort.resize(MOTOR_NUM);
    }
    void publish()
    {
        for(int i=0; i<MOTOR_NUM; i++)
        {
            jointMsg_.position[i] = motorDevPtr_[i].present_position;
            jointMsg_.velocity[i] = motorDevPtr_[i].present_velocity;
            jointMsg_.effort[i] = motorDevPtr_[i].present_current;
        }

        jointPub_.publish(jointMsg_);
    }
};

void motor_informations()
{
}

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
    motor[LIDAR_MOTOR].joint_name = "lidar_joint";
    motor[LIDAR_MOTOR].offset_position = -27.5 * DEG2RAD;



    ThormangMobileJointROS comm(nh, motor);


    dynamixel::PortHandler *portHandler =
            dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    dynamixel::PacketHandler *packetHandler2 =
            dynamixel::PacketHandler::getPacketHandler(2.0);



    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler2,
                                             ADDR_PRO_GOAL_POSITION, 8);

    // Initialize Groupsyncread instance for Present Position
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler2,
                                           ADDR_PRO_PRESENT_POSITION, 8);


    // Initializing port
    if(init_port(portHandler) == -1) return -1;

    // Model get
    for(int i=0; i<MOTOR_NUM; i++)
    {
        int _comm_result = COMM_TX_FAIL;
        _comm_result = packetHandler2->read2ByteTxRx(portHandler,
                                      motor[i].id,
                                      ADDR_PRO_MODEL_NUMBER,
                                      &motor[i].model_num,
                                      &motor[i].error);
        if(_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("ID = %d: Model Number Read Failed.", motor[i].id);
            if (_comm_result != COMM_SUCCESS)
                packetHandler2->printTxRxResult(_comm_result);

            return -1;
        }
        else
        {
            ROS_INFO("ID = %d: Model = %d", motor[i].id, motor[i].model_num);
        }
    }

    // Torque ON!
    for(int i=0; i<MOTOR_NUM; i++)
    {
        int _comm_result = packetHandler2->write1ByteTxRx(portHandler,
                                                          motor[i].id,
                                                          ADDR_PRO_TORQUE_ENABLE,
                                                          1,
                                                          &motor[i].error);
        if (_comm_result != COMM_SUCCESS)
            packetHandler2->printTxRxResult(_comm_result);
    }

    for(int i=0; i<MOTOR_NUM; i++)
    {
        groupSyncRead.addParam(motor[i].id);
        // groupSyncWrite.addParam(motor[i].id);
    }

    while(ros::ok())
    {

        // Parameter Update
        groupSyncWrite.clearParam();

        for(int i=0; i<MOTOR_NUM; i++)
        {
            motor[i].make_bytes();
            groupSyncWrite.addParam(motor[i].id, motor[i].tx_bytes);
        }


        // Packet Exchange
        int _comm_result = groupSyncRead.txRxPacket();
        if (_comm_result != COMM_SUCCESS)
            packetHandler2->printTxRxResult(_comm_result);

        _comm_result = groupSyncWrite.txPacket();
        if (_comm_result != COMM_SUCCESS)
            packetHandler2->printTxRxResult(_comm_result);


        // Get Datas
        for(int i=0; i<MOTOR_NUM; i++)
        {
            bool available = groupSyncRead.isAvailable(motor[i].id,
                                      ADDR_PRO_PRESENT_POSITION, 8);
            if(available == true)
            {
                motor[i].update_position(groupSyncRead.getData(motor[i].id,
                                                  ADDR_PRO_PRESENT_POSITION,4));
                motor[i].update_velocity(groupSyncRead.getData(motor[i].id,
                                                  ADDR_PRO_PRESENT_VELOCITY,4));
                motor[i].present_current = 0.0;
            }
            else
            {
                ROS_ERROR("ID = %d: sync read failed", motor[i].id);
            }
        }


        // just for checking everything is going well.
        // ROS_INFO("motor1 - pos : %.2lf rad, vel : %.2lf rad/s",motor[0].present_position, motor[0].present_velocity);

        comm.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    portHandler->closePort();

    return 0;
}
