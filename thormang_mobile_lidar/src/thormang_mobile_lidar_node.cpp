#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_assembler/AssembleScans2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace laser_assembler;

namespace thormang_mobile
{

class LidarAssembler
{

    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;

    ros::Publisher joint_pub_;
    ros::Publisher pointcloud_pub_;

    ros::ServiceClient laser_assembler_sc_;

    double lidar_angle_;
    double lidar_goal_angle_;
    bool is_left_turn_;

    ros::Time turn_start_time_;
    ros::Time turn_end_time_;

    double angle_min_error_;

    sensor_msgs::JointState joint_msg_;

public:
    LidarAssembler() : nh_("/thormang_mobile"), is_left_turn_(false),
        angle_min_error_(0.088) // 1 degree
    {
        joint_sub_ = nh_.subscribe("joint_states", 5, &LidarAssembler::jointCallback, this);

        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_point_cloud", 100);
        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_set", 1);
        ros::service::waitForService("/assemble_scans2");
        laser_assembler_sc_ = nh_.serviceClient<AssembleScans2>("/assemble_scans2");


        joint_msg_.name.push_back(string("neck_lidar_joint"));
        //joint_msg_.velocity.push_back(0.885398);//(0.885398);
        joint_msg_.velocity.push_back(1.2);//(0.885398);
        joint_msg_.position.push_back(0.0);
    }

    void start_trigger()
    {
        motor_trigger();
    }

    void loop()
    {
        if(fabs(lidar_goal_angle_ - lidar_angle_) < angle_min_error_)
        {

            turn_end_time_ = ros::Time::now();
            AssembleScans2 srv;
            srv.request.begin = turn_start_time_;
            srv.request.end = turn_end_time_;

            // ROS_INFO("Trying to call: %d.%d ~ %d.%d",
            //          turn_start_time_.sec, turn_start_time_.nsec, turn_end_time_.sec, turn_end_time_.nsec);
            if (laser_assembler_sc_.call(srv))
            {
                pointcloud_pub_.publish(srv.response.cloud);
                // ROS_INFO("Service called. points = %d",(int)(srv.response.cloud.data.size()));
            }
            else
            {
              ROS_ERROR("Service call failed");
            }
            motor_trigger();
        }
    }

    void motor_move()
    {
        joint_msg_.position[0] = lidar_goal_angle_;
        joint_pub_.publish(joint_msg_);
    }
private:

    void motor_trigger()
    {
      if(is_left_turn_)
          lidar_goal_angle_ = 1.745329252; // 90 degree
      else
          lidar_goal_angle_ =  -1.745329252; // 90 degree

      /*
        if(is_left_turn_)
            lidar_goal_angle_ = 3.141592 + 0.4618;//2*(1.570796327 + 1.570796327); // 90 degree
        else
            lidar_goal_angle_ = -3.141592 + 0.4632;// -1.570796327 - 1.570796327; // 90 degree
*/
        is_left_turn_ = !is_left_turn_; // toggle

        turn_start_time_ = ros::Time::now();
    }

private:
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        int size = msg->name.size();
        int i;
        for(i=0; i<size ;i++)
        {
            if(msg->name[i] == "neck_lidar_joint") break;
        }

        if(i == size) return; // not exist

        lidar_angle_ = msg->position[i];
    }
};


}
using namespace thormang_mobile;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"thormang_mobile_lidar");


    LidarAssembler la;

    ros::Rate rate(10);

    la.start_trigger();

    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        la.loop();
        la.motor_move();

    }
    // thormang_mobile::LidarTF lidar_tf;
    ros::spin();
    return 0;
}
