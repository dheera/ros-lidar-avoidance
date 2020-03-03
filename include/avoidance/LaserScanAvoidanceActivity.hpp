#ifndef _LaserScanAvoidanceActivity_hpp
#define _LaserScanAvoidanceActivity_hpp

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

namespace avoidance {

class LaserScanAvoidanceActivity {
  public:
	LaserScanAvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

	bool start();
	bool stop();
	bool spinOnce();
    void onScan(const sensor_msgs::LaserScanPtr& msg);
    void onWinning(const geometry_msgs::TwistPtr& msg);

  private:

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;

    std::string param_topic_vel;
    std::string param_topic_scan;
    std::string param_topic_multiplier;
    std::vector<double> param_ignore_regions;
    double param_angle_offset;
    double param_position_offset_x;
    double param_position_offset_y;
    double param_avoidance_width;
    double param_distance_stop;
    double param_distance_slow;

    double motion_angle;

	ros::Publisher pub_multiplier;
	ros::Subscriber sub_scan;
	ros::Subscriber sub_winning;
};

}

#endif /* _LaserScanAvoidanceActivity_hpp */

