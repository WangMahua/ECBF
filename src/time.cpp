#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 	std::cout << std::setprecision(20) << msg->header.stamp.toSec()<<std::endl;
}
 int main(int argc, char **argv){
	ros::init(argc, argv, "time");
  	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/vrpn_client_node/MAV3/pose", 1000, chatterCallback);

	std::cout << "hello\n";
	ros::spin();
	return 0;
}
