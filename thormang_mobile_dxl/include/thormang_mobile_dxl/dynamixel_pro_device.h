// Control table address
#define ADDR_PRO_MODEL_NUMBER           0       // 2
#define ADDR_PRO_OPERATING_MODE         11      // 1

#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_GOAL_VELOCITY          600
#define ADDR_PRO_PRESENT_POSITION       611
#define ADDR_PRO_PRESENT_VELOCITY       615

#define H54_RESOLUTION 501923
#define H42_RESOLUTION 303750
#define H54_GEAR_RATIO (H54_RESOLUTION/1000.0)
#define H42_GEAR_RATIO (H42_RESOLUTION/1000.0)
// gear ratio = res / 1000.0;
// rpm = value / gear ratio

#define RPM2RADPS (M_PI/30.0)
#define RADPS2RPM (30.0/M_PI)

#define DEG2RAD (M_PI/180.0)


enum OPERATING_MODE { OP_TORQUE, OP_VELOCITY, OP_RESERVED, OP_POSITION };
enum MODEL_NAME { H42_20_S300_R = 51200,
                  H54_100_S500_R = 53768,
                  H54_200_S500_R = 54024 };

enum MOTOR_NAME {LEFT_WHEEL = 0, RIGHT_WHEEL, LIDAR_MOTOR, MOTOR_NUM};

struct motor_device
{
    int id;
    uint16_t model_num;
    std::string joint_name;

    int mode;

    double goal_position;   // Radian
    double goal_velocity;   // Radian per sec

    double offset_position;

    uint8_t tx_bytes[8];
    uint8_t rx_bytes[8];

    double present_position;    // Radian
    double present_velocity;    // Radian per sec
    double present_current;

    uint8_t error;

    void update_position(int n_value)
    {
        if(model_num == H54_100_S500_R || model_num == H54_200_S500_R)
        {
            present_position = n_value * M_PI / (H54_RESOLUTION/2.) - offset_position;
        }
        else if(model_num == H42_20_S300_R)
        {
            present_position = n_value * M_PI / (H42_RESOLUTION/2.) - offset_position;
        }
    }

    void update_velocity(int n_value)
    {
        if(model_num == H54_100_S500_R || model_num == H54_200_S500_R)
        {
            present_velocity = n_value / H54_GEAR_RATIO * RPM2RADPS;
        }
        else if(model_num == H42_20_S300_R)
        {
            present_velocity = n_value / H42_GEAR_RATIO * RPM2RADPS;
        }
    }

    void make_bytes()
    {
        int n_goal_position = 0;
        int n_goal_velocity = 0;

        if(model_num == H54_100_S500_R || model_num == H54_200_S500_R)
        {
            n_goal_position = (goal_position+offset_position) * (H54_RESOLUTION/2.) / M_PI;
            n_goal_velocity = goal_velocity * RADPS2RPM * H54_GEAR_RATIO;
        }
        else if(model_num == H42_20_S300_R)
        {
            n_goal_position = (goal_position+offset_position) * (H42_RESOLUTION/2.) / M_PI;
            n_goal_velocity = goal_velocity * RADPS2RPM * H42_GEAR_RATIO;
        }

        tx_bytes[0] = DXL_LOBYTE(DXL_LOWORD(n_goal_position));
        tx_bytes[1] = DXL_HIBYTE(DXL_LOWORD(n_goal_position));
        tx_bytes[2] = DXL_LOBYTE(DXL_HIWORD(n_goal_position));
        tx_bytes[3] = DXL_HIBYTE(DXL_HIWORD(n_goal_position));

        tx_bytes[4] = DXL_LOBYTE(DXL_LOWORD(n_goal_velocity));
        tx_bytes[5] = DXL_HIBYTE(DXL_LOWORD(n_goal_velocity));
        tx_bytes[6] = DXL_LOBYTE(DXL_HIWORD(n_goal_velocity));
        tx_bytes[7] = DXL_HIBYTE(DXL_HIWORD(n_goal_velocity));
    }
};

