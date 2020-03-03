#include <avoidance/LaserScanAvoidanceActivity.hpp>

int main(int argc, char *argv[]) {
    ros::NodeHandle *nh = NULL;
    ros::NodeHandle *nh_priv = NULL;
    avoidance::LaserScanAvoidanceActivity* activity = NULL;

    ros::init(argc, argv, "avoidance_node");

    nh = new ros::NodeHandle( );
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHanlde");
        ros::shutdown( );
        return -1;
    }

    nh_priv = new ros::NodeHandle("~");
    if( !nh_priv ) {
        ROS_FATAL("Failed to initialize private NodeHanlde");
        delete nh;
        ros::shutdown( );
        return -2;
    }

    activity = new avoidance::LaserScanAvoidanceActivity(*nh, *nh_priv);
    if(!activity) {
      ROS_FATAL("Failed to initialize activity");
      delete nh_priv;
      delete nh;
      ros::shutdown();
      return -3;
    }

    if(!activity->start()) {
      ROS_ERROR("failed to start the activity");
    }

    ros::Rate rate(200);
    while(ros::ok()) {
      rate.sleep();
      ros::spinOnce();
      activity->spinOnce();
    }

    delete activity;
    delete nh_priv;
    delete nh;

    return 0;
}
