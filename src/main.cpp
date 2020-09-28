#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<vins_uart/serial.hpp>
#include"thread/ros_thread.h"
#include"thread/imu_thread.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"vins_uart");           
	serial_init((char *)"/dev/ttyUSB0", 115200);
	std::thread thread_imu(imu_thread_entry);
	std::thread thread_ros(ros_thread_entry);

	thread_imu.join();
	thread_ros.join();

	return 0;
}
