#include <nav_msgs/Odometry.h>      
#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__
void odom_callback(nav_msgs::Odometry odom); 

int ros_thread_entry();

#endif
