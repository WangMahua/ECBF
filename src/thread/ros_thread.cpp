#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "ecbf_uart/serial.hpp"
#include "ros_thread.h"
#include <mutex>

//ROS message
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


using namespace std;

mutex ros_mutex;

double pos[3] = {0,0,0};
double vel[3] = {0,0,0};
double last_pos[3] = {0,0,0};
double last_time = 0;
bool pose_init_flag = false;
nav_msgs::Path path;
ros::Publisher path_pub;

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		double now_time;
		if (pose_init_flag == false){
			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			last_pos[0] = pos[0];
			last_pos[1] = pos[1];
			last_pos[2] = pos[2];
			last_time = now_time;
			pose_init_flag = true;	
		}
		else{
//			now_time = msg->header.stamp.toSec();
			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			vel[0] =(pos[0] - last_pos[0])/0.0083;
			vel[1] =(pos[1] - last_pos[1])/0.0083;
			vel[2] =(pos[2] - last_pos[2])/0.0083;
			last_pos[0] = pos[0];
			last_pos[1] = pos[1];
			last_pos[2] = pos[2];
			last_time = now_time;
/*
			//pub path info 
		    	path.header = msg->header;
		    	path.poses.push_back(*msg);
		    	path_pub.publish(path);
*/
		}

}
void odom_callback(nav_msgs::Odometry odom)
{
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
}
int ros_thread_entry(){
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("vins_estimator/imu_propagate",1000,odom_callback);
	ros::Subscriber pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 1, pos_callback);
	path_pub = n.advertise<nav_msgs::Path>("ECBF_Path", 1);
	ros::spin();
	return 0;
}
