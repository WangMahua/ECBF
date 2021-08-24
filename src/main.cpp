#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<ecbf_uart/serial.hpp>
#include"thread/ros_thread.h"
#include"thread/imu_thread.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ecbf_uart");           
	serial_init((char *)"/dev/ECBF_UART", 115200);
	std::thread thread_imu(imu_thread_entry);	//get_imu data from stm32
	std::thread thread_ros(ros_thread_entry);	//push imu data to ROS and recieve position data from ROS

	thread_imu.join();
	thread_ros.join();

	return 0;
}
