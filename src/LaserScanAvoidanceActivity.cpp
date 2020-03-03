#include "avoidance/LaserScanAvoidanceActivity.hpp"

#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace avoidance {

LaserScanAvoidanceActivity::LaserScanAvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
	nh(_nh),
	nh_priv(_nh_priv)
{
  ROS_INFO("initializing");
  nh_priv.param("topic_vel", param_topic_vel, (std::string)"/cmd_vel/managed");
  nh_priv.param("topic_scan", param_topic_scan, (std::string)"/lidar/scan");
  nh_priv.param("topic_multiplier", param_topic_multiplier, (std::string)"/cmd_vel/multiplier");
  nh_priv.param("ignore_regions", param_ignore_regions, std::vector<double> {}); // xmin0, xmax0, ymin0, ymax0, xmin1, xmax1, ymin1, ymax1, ...
  nh_priv.param("angle_offset", param_angle_offset, (double)0.0);
  nh_priv.param("position_offset_x", param_position_offset_x, (double) 0.05);
  nh_priv.param("position_offset_y", param_position_offset_y, (double) 0.00);
  nh_priv.param("avoidance_width", param_avoidance_width, 0.3);
}

bool LaserScanAvoidanceActivity::start() {
  ROS_INFO("starting");

  if(!pub_multiplier) pub_multiplier = nh.advertise<std_msgs::Float32>(param_topic_multiplier, 1);

  if(!sub_scan) {
    sub_scan = nh.subscribe(param_topic_scan, 1, &LaserScanAvoidanceActivity::onScan, this);
  }

  if(!sub_winning) {
    sub_winning = nh.subscribe(param_topic_vel, 1, &LaserScanAvoidanceActivity::onWinning, this);
  }

  return true;
}

bool LaserScanAvoidanceActivity::spinOnce() {
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  return true;	
}

bool LaserScanAvoidanceActivity::stop() {
  ROS_INFO("stopping");

  if(pub_multiplier) pub_multiplier.shutdown();
  if(sub_scan) sub_scan.shutdown();
  if(sub_winning) sub_winning.shutdown();

  return true;
}

void LaserScanAvoidanceActivity::onWinning(const geometry_msgs::TwistPtr& msg) {
  if(abs(msg->linear.x) < 0.001 && abs(msg->linear.y) < 0.001) return;
  motion_angle = atan2(msg->linear.y, msg->linear.x);
}

void LaserScanAvoidanceActivity::onScan(const sensor_msgs::LaserScanPtr& msg) {
  double theta;
  double r;
  double x, y;
  int i, j;

  double multiplier_min = 1.0;

  for(i=0;i<msg->ranges.size();i++) {
    r = msg->ranges[i];
    if(r > 0.7) continue;

    theta = -param_angle_offset + msg->angle_min + msg->angle_increment * i;

    x = r*cos(theta) - param_position_offset_x;
    y = r*sin(theta) - param_position_offset_y;

    bool ignore_point = false;

    for(j=0;j<param_ignore_regions.size()/4;j++) {
      if(x > param_ignore_regions[j*4 + 0] && x < param_ignore_regions[j*4 + 1] && \
         y > param_ignore_regions[j*4 + 2] && y < param_ignore_regions[j*4 + 3]) {
        ignore_point = true;
        ROS_WARN_STREAM_THROTTLE(2, "ignore at x=" << x << " y=" << y);
      }
    }

    if(ignore_point) continue;

    if(cos(theta - motion_angle) > 0.5 && std::abs(r*sin(theta - motion_angle)) < param_avoidance_width/2) {
      if(r < 0.3) {
         ROS_WARN_STREAM_THROTTLE(2, "obstacle in path at x=" << x << " y=" << y);
      }
      multiplier_min = std::min(
              multiplier_min, 
              pow((std::max(r, 0.25) - 0.25) / 0.7, 0.5)
      );
    }
  }

  ROS_DEBUG_STREAM("multiplier = " << multiplier_min);

  std_msgs::Float32 msg_multiplier;
  msg_multiplier.data = multiplier_min;
  pub_multiplier.publish(msg_multiplier);

}

}
