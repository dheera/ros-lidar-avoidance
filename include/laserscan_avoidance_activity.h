#ifndef _laserscan_avoidance_activity_h
#define _laserscan_avoidance_activity_h

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class LaserScanAvoidanceActivity {
  public:
	LaserScanAvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

	bool start();
	bool stop();
	bool spinOnce();
    void onScan(const sensor_msgs::LaserScanPtr& msg);
    void onCmdVel(const geometry_msgs::TwistPtr& msg);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    std::string param_topic_cmd_vel;
    std::string param_topic_scan;
    std::string param_topic_speed_limit;
    std::vector<double> param_ignore_regions;
    double param_angle_offset;
    double param_position_offset_x;
    double param_position_offset_y;
    double param_avoidance_width;
    double param_speed_max; // maximum possible speed of robot
    double param_distance_slow; // distance at which speed limits start being issued, starting with the max speed
    double param_distance_stop; // distance at which speed limit is reduced to 0 (stop)

    double motion_angle;

    ros::Publisher pub_speed_limit;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_cmd_vel;
};

#endif /* _laserscan_avoidance_activity_h */
