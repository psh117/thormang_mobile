#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_assembler/AssembleScans2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

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

const double R1 = 0.0051523718;  // pitch
const double R2 = -0.0076767227949;  // yaw
const double R3 = -0.0177448575287;  // z-axis
// -0.0177448575287 -0.0076767227949 0.0051523718
namespace thormang_mobile
{

class LidarAssembler
{

  ros::NodeHandle nh_;



  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;
  geometry_msgs::TransformStamped rotation_tf_;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;
  ros::Subscriber joint_sub_;

  pcl::PointCloud<pcl::PointXYZ> assembled_cloud_;


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


    rotation_tf_.header.frame_id = "/camera";
    scan_sub_ = nh_.subscribe("/lidar_scan",10, &LidarAssembler::scanCallback, this,
                              ros::TransportHints().tcpNoDelay(true));
    joint_sub_ = nh_.subscribe("joint_states", 10, &LidarAssembler::jointCallback, this,
                               ros::TransportHints().tcpNoDelay(true));
    point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> ("lidar_point_cloud", 10);

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_set", 1);

    //joint_sub_ = nh_.subscribe("joint_states", 5, &LidarAssembler::jointCallback, this);
    //pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_point_cloud", 100);
    //ros::service::waitForService("/assemble_scans2");
    //laser_assembler_sc_ = nh_.serviceClient<AssembleScans2>("/assemble_scans2");


    joint_msg_.name.push_back(string("neck_lidar_joint"));
    //joint_msg_.velocity.push_back(0.885398);//(0.885398);
    joint_msg_.velocity.push_back(0.2);//(1.2);//(0.885398);
    //joint_msg_.velocity.push_back(0.3);//(0.885398);
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

      //turn_end_time_ = ros::Time::now();
      //AssembleScans2 srv;
      //srv.request.begin = turn_start_time_;
      //srv.request.end = turn_end_time_;

      // ROS_INFO("Trying to call: %d.%d ~ %d.%d",
      //          turn_start_time_.sec, turn_start_time_.nsec, turn_end_time_.sec, turn_end_time_.nsec);
      //if (laser_assembler_sc_.call(srv))
      //{
      //    pointcloud_pub_.publish(srv.response.cloud);
      // ROS_INFO("Service called. points = %d",(int)(srv.response.cloud.data.size()));
      //}
      //else
      //{
      //  ROS_ERROR("Service call failed");
      //}
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
  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
  {
    sensor_msgs::PointCloud2 _cloud_in, _cloud_out;
    projector_.transformLaserScanToPointCloud("/base_link", *scan, _cloud_in, tfListener_);
    bool _is_sweep_done = false;
    if (is_left_turn_) // lidar_goal_angle = -1.7
    {
      if(lidar_angle_ < -M_PI_2)
        _is_sweep_done = true;
      else if (lidar_angle_ > M_PI_2)
        return;
    }
    else  // lidar_goal_angle = 1.7
    {
      if(lidar_angle_ > M_PI_2) // sweep done
        _is_sweep_done = true;
      else if (lidar_angle_ < -M_PI_2)
        return;
    }

    // calibration

    tf::Quaternion _calib_q;
    _calib_q.setRPY(0,R1,R2);
    rotation_tf_.transform.rotation.x = _calib_q.x();
    rotation_tf_.transform.rotation.y = _calib_q.y();
    rotation_tf_.transform.rotation.z = _calib_q.z();
    rotation_tf_.transform.rotation.w = _calib_q.w();
    rotation_tf_.transform.translation.z = R3;

    tf2::doTransform(_cloud_in, _cloud_out, rotation_tf_);

    pcl::PointCloud<pcl::PointXYZ> _current_cloud;
    //_current_cloud
    pcl::fromROSMsg(_cloud_out,_current_cloud);

    if(_is_sweep_done)
    {
      // Point Cloud Publish
      sensor_msgs::PointCloud2 _assembled_cloud_ros;
      pcl::toROSMsg(assembled_cloud_, _assembled_cloud_ros);
      _assembled_cloud_ros.header.stamp = ros::Time::now();
      _assembled_cloud_ros.header.frame_id = "/camera";
      point_cloud_publisher_.publish(_assembled_cloud_ros);

      // Reset Assembeld Point Cloud
      assembled_cloud_.clear();
      motor_trigger();
      motor_move();
    }

    // Register to Assembled Point Cloud
    assembled_cloud_ = assembled_cloud_ + _current_cloud;

  }

  void jointCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    int size = msg->name.size();
    int i;
    for(i=0; i<size ;i++)
    {
      if(msg->name[i] == "head_lidar") break;
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
  la.motor_move();

  ros::spin();
  /*
  while(ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    //la.loop();
    //la.motor_move();

  }
  */
  // thormang_mobile::LidarTF lidar_tf;
  return 0;
}
