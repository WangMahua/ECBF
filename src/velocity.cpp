#include <iostream>
#include <std_msgs/Int32.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <ecbf_uart/rc_info.h>
#include <ecbf_uart/qp_info.h>

double pose[3] = {0,0,0};
double velocity[3] = {0,0,0};
double last_pose[3] = {0,0,0};
double last_time = 0;
bool pose_init_flag = false;
using namespace std;
void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		double now_time;
		if (pose_init_flag == false){
			now_time = ros::Time::now().toSec();
			pose[0] = msg->pose.position.x;
			pose[1] = msg->pose.position.y;
			pose[2] = msg->pose.position.z;
			last_pose[0] = pose[0];
			last_pose[1] = pose[1];
			last_pose[2] = pose[2];
			last_time = now_time;
			pose_init_flag = true;	
		}
		else{
//			now_time = msg->header.stamp.toSec();
			now_time = ros::Time::now().toSec();
			pose[0] = msg->pose.position.x;
			pose[1] = msg->pose.position.y;
			pose[2] = msg->pose.position.z;
			velocity[0] =(pose[0] - last_pose[0])/0.0083;
			velocity[1] =(pose[1] - last_pose[1])/0.0083;
			velocity[2] =(pose[2] - last_pose[2])/0.0083;
			last_pose[0] = pose[0];
			last_pose[1] = pose[1];
			last_pose[2] = pose[2];
			last_time = now_time;
		}

}
int main(int argc, char** argv)
{
        ros::init(argc, argv, "velocity");
	ros::NodeHandle n;
	ros::Subscriber pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 10, pos_callback);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	cout<<"start\n";
	
	ros::Rate rate(400);

	geometry_msgs::Vector3 vel_value;
	
	/* debug */
	ecbf_uart::rc_info debug_rc;	
	ecbf_uart::qp_info debug_qp;	

	while(ros::ok()){
			vel_value.x = velocity[0];
			vel_value.y = velocity[1];
			vel_value.z = velocity[2];

			vel_pub.publish(vel_value);
	ros::spinOnce();
	rate.sleep();	
	}

}
