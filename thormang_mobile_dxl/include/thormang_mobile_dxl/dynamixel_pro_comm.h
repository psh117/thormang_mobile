#ifndef DYNAMIXEL_PRO_COMM_H_
#define DYNAMIXEL_PRO_COMM_H_

// STD
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// USER
#include "thormang_mobile_dxl/dynamixel_pro_device.h"

class DynamxelProAdaptor
{
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler2_;
    motor_device *motor_;
    const int motorNum_;

    dynamixel::GroupSyncWrite *groupSyncWrite_;
    dynamixel::GroupSyncRead *groupSyncRead_;

public:
    DynamxelProAdaptor(const char* dev, motor_device* motor, int motorNum) :
        motor_(motor),
        motorNum_(motorNum)
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(dev); //"/dev/ttyUSB0"
        packetHandler2_ = dynamixel::PacketHandler::getPacketHandler(2.0);
        groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler2_,ADDR_PRO_GOAL_POSITION, 8);
        groupSyncRead_ = new dynamixel::GroupSyncRead(portHandler_,packetHandler2_, ADDR_PRO_PRESENT_POSITION, 8);
    }

    virtual ~DynamxelProAdaptor()
    {
        portHandler_->closePort();
    }

    int initPort(int baud)
    {
        // Open port
        if (portHandler_->openPort())
        {
            ROS_INFO("THRMNG DXL - Succeeded to open the dynamixel port!");
        }
        else
        {
            ROS_ERROR("THRMNG DXL - Failed to open the port!");
            return -1;
        }

        // Set port baudrate
        if (portHandler_->setBaudRate(baud))
        {
          ROS_INFO("THRMNG DXL - Succeeded to change the baudrate!");
        }
        else
        {
            ROS_ERROR("THRMNG DXL - Failed to change the baudrate!");
            return -1;
        }
        return 0;
    }

    int getModel()
    {
        for(int i=0; i<motorNum_; i++)
        {
            int _comm_result = COMM_TX_FAIL;
            _comm_result = packetHandler2_->read2ByteTxRx(portHandler_,
                                          motor_[i].id,
                                          ADDR_PRO_MODEL_NUMBER,
                                          &motor_[i].model_num,
                                          &motor_[i].error);
            if(_comm_result != COMM_SUCCESS)
            {
                ROS_ERROR("ID = %d: Model Number Read Failed.", motor_[i].id);
                if (_comm_result != COMM_SUCCESS)
                    packetHandler2_->printTxRxResult(_comm_result);

                return -1;
            }
            else
            {
                ROS_INFO("ID = %d: Model = %d", motor_[i].id, motor_[i].model_num);
            }
        }
        return 0;
    }

    void torqueOn(int torque)
    {

        for(int i=0; i<motorNum_; i++)
        {
            int _comm_result = packetHandler2_->write1ByteTxRx(portHandler_,
                                                              motor_[i].id,
                                                              ADDR_PRO_TORQUE_ENABLE,
                                                              torque,
                                                              &motor_[i].error);
            if (_comm_result != COMM_SUCCESS)
                packetHandler2_->printTxRxResult(_comm_result);
        }

    }

    void packetExchange()
    {

        int _comm_result = groupSyncRead_->txRxPacket();
        if (_comm_result != COMM_SUCCESS)
            packetHandler2_->printTxRxResult(_comm_result);

        _comm_result = groupSyncWrite_->txPacket();
        if (_comm_result != COMM_SUCCESS)
            packetHandler2_->printTxRxResult(_comm_result);
    }

    void syncReadReady()
    {

        for(int i=0; i<motorNum_; i++)
        {
            groupSyncRead_->addParam(motor_[i].id);
        }
    }

    void parameterUpdate()
    {

        groupSyncWrite_->clearParam();

        for(int i=0; i<motorNum_; i++)
        {
            motor_[i].make_bytes();
            groupSyncWrite_->addParam(motor_[i].id, motor_[i].tx_bytes);
        }

    }

    void getDatas()
    {
        for(int i=0; i<motorNum_; i++)
        {
            bool available = groupSyncRead_->isAvailable(motor_[i].id,
                                      ADDR_PRO_PRESENT_POSITION, 8);
            if(available == true)
            {
                motor_[i].update_position(groupSyncRead_->getData(motor_[i].id,
                                                  ADDR_PRO_PRESENT_POSITION,4));
                motor_[i].update_velocity(groupSyncRead_->getData(motor_[i].id,
                                                  ADDR_PRO_PRESENT_VELOCITY,4));
                motor_[i].present_current = 0.0;
            }
            else
            {
                ROS_ERROR("ID = %d: sync read failed", motor_[i].id);
            }
        }

    }
};

#endif // DYNAMIXEL_PRO_COMM_H_
