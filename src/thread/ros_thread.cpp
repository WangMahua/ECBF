#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "vins_uart/serial.hpp"
#include "ros_thread.h"
#include <mutex>


using namespace std;

mutex ros_mutex;

void odom_callback(nav_msgs::Odometry odom)
{
	ros_mutex.lock();
//	cout << odom.pose.pose.position.x << endl;
	send_pose_to_serial( 
				odom.pose.pose.position.x,
				odom.pose.pose.position.y,
				odom.pose.pose.position.z,
				odom.pose.pose.orientation.x,
				odom.pose.pose.orientation.y,
				odom.pose.pose.orientation.z,
				odom.pose.pose.orientation.w,
				odom.twist.twist.linear.x,
				odom.twist.twist.linear.y,
				odom.twist.twist.linear.z
			);
	ros_mutex.unlock();
}
int ros_thread_entry(){
	
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("vins_estimator/imu_propagate",1000,odom_callback);
	
	ros::spin();
	
	return 0;
}
