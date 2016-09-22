
int init_port(dynamixel::PortHandler *portHandler)
{

    // Open port
    if (portHandler->openPort())
    {
        ROS_INFO("THRMNG DXL - Succeeded to open the dynamixel port!");
    }
    else
    {
        ROS_ERROR("THRMNG DXL - Failed to open the port!");
        return -1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(3000000))
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

