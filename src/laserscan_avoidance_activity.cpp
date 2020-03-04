#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <laserscan_avoidance_activity.h>

LaserScanAvoidanceActivity::LaserScanAvoidanceActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
	nh(_nh),
	nh_priv(_nh_priv)
{
  ROS_INFO("initializing");
  nh_priv.param("topic_cmd_vel", param_topic_cmd_vel, (std::string)"cmd_vel");
  nh_priv.param("topic_scan", param_topic_scan, (std::string)"/lidar/scan");
  nh_priv.param("topic_speed_limit", param_topic_speed_limit, (std::string)"speed_limit");
  nh_priv.param("ignore_regions", param_ignore_regions, std::vector<double> {}); // xmin0, xmax0, ymin0, ymax0, xmin1, xmax1, ymin1, ymax1, ...

  // TODO: use tf instead of this
  nh_priv.param("angle_offset", param_angle_offset, (double)0.0);
  nh_priv.param("position_offset_x", param_position_offset_x, (double) 0.00);
  nh_priv.param("position_offset_y", param_position_offset_y, (double) 0.00);

  nh_priv.param("speed_max", param_speed_max, (double)2.0);
  nh_priv.param("avoidance_width", param_avoidance_width, 0.6);
  nh_priv.param("distance_stop", param_distance_stop, 0.5);
  nh_priv.param("distance_slow", param_distance_slow, 1.5);
}

bool LaserScanAvoidanceActivity::start() {
  ROS_INFO("starting");

  if(!pub_speed_limit) pub_speed_limit = nh.advertise<std_msgs::Float32>(param_topic_speed_limit, 1);

  if(!sub_scan) {
    sub_scan = nh.subscribe(param_topic_scan, 1, &LaserScanAvoidanceActivity::onScan, this);
  }

  if(!sub_cmd_vel) {
    sub_cmd_vel = nh.subscribe(param_topic_cmd_vel, 1, &LaserScanAvoidanceActivity::onCmdVel, this);
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

  if(pub_speed_limit) pub_speed_limit.shutdown();
  if(sub_scan) sub_scan.shutdown();
  if(sub_cmd_vel) sub_cmd_vel.shutdown();

  return true;
}

void LaserScanAvoidanceActivity::onCmdVel(const geometry_msgs::TwistPtr& msg) {
  // ROS_INFO_STREAM("vel = " << msg->linear.x << " " << msg->angular.z);
  if(std::abs(msg->linear.x) < 0.001 && std::abs(msg->linear.y) < 0.001) {
    motion_angle = 0.0; // arctan singularity
  } else {
    motion_angle = atan2(msg->linear.y, msg->linear.x);
  }
  // ROS_INFO_STREAM("motion_angle = " << motion_angle);
}

void LaserScanAvoidanceActivity::onScan(const sensor_msgs::LaserScanPtr& msg) {
  double theta;
  double r;
  double x, y;
  int i, j;

  double speed_limit_min = 100000000.0;

  for(i=0;i<msg->ranges.size();i++) {

    r = msg->ranges[i];
    theta = -param_angle_offset + msg->angle_min + msg->angle_increment * i;
    
    x = r * std::cos(theta) - param_position_offset_x;
    y = r * std::sin(theta) - param_position_offset_y;

    // compute new r, theta
    r = std::pow(x*x + y*y, 0.5);
    theta = std::atan2(y, x);

    if(std::pow(x,2) + std::pow(y,2) > std::pow(param_distance_slow,2)) continue;

    bool ignore_point = false;

    for(j=0;j<param_ignore_regions.size()/4;j++) {
      if(x > param_ignore_regions[j*4 + 0] && x < param_ignore_regions[j*4 + 1] && \
         y > param_ignore_regions[j*4 + 2] && y < param_ignore_regions[j*4 + 3]) {
        ignore_point = true;
        ROS_DEBUG_STREAM_THROTTLE(2, "point ignored at x=" << x << " y=" << y);
      }
    }

    if(ignore_point) continue;

    if(std::cos(theta - motion_angle) > 0.0 && // point is within +/- 90 degrees of motion angle
       std::abs(r*sin(theta - motion_angle)) < param_avoidance_width/2) // is within robot path
    {
      if(r < param_distance_stop + 0.1) {
         ROS_WARN_STREAM_THROTTLE(2, "obstacle in path at x=" << x << " y=" << y);
      }
      speed_limit_min = std::min(
              speed_limit_min, 
              param_speed_max * std::pow(
                (std::max(r, param_distance_stop) - param_distance_stop) / param_distance_slow,
                0.5)
      );
    }
  }

  std_msgs::Float32 msg_speed_limit;
  msg_speed_limit.data = speed_limit_min;
  pub_speed_limit.publish(msg_speed_limit);

}
