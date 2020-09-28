#include <iostream>
#include <vins_uart/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>


using namespace std;

mutex imu_mutex;
imu_t imu;

uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = IMU_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[3], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		return 1; //error detected
	}
	float enu_acc_x, enu_acc_y, enu_acc_z;

	memcpy(&enu_acc_x, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&enu_acc_y, &buf[6], sizeof(float));
	memcpy(&enu_acc_z, &buf[10], sizeof(float));
	imu.acc[0] = enu_acc_x; //east
	imu.acc[1] = enu_acc_y; //north
	imu.acc[2] = enu_acc_z; //up
	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&imu.gyrop[0], &buf[14], sizeof(float));
	memcpy(&imu.gyrop[1], &buf[18], sizeof(float));
	memcpy(&imu.gyrop[2], &buf[22], sizeof(float));


	return 0;


}

void imu_buf_push(uint8_t c)
{
	if(imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			imu.buf[i - 1] = imu.buf[i];
		}

		/* save new byte to the last array element */
		imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		imu.buf[imu.buf_pos] = c;
		imu.buf_pos++;
	}
}


int imu_thread_entry(){
	ros::NodeHandle n;
	ros::Publisher sub = n.advertise<sensor_msgs::Imu>("topic_name", 5);
	char c;
	imu.buf_pos = 0;
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c);  
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+')
			{
				/*
				for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
					cout << imu.buf[i];
				cout<<endl;
				*/
				if(imu_decode(imu.buf)==0)
				{
					cout<<"imu_acc_x: "<<imu.acc[0]*100.0f << " imu_acc_y: " <<imu.acc[1]*100.0f << " imu_acc_z: "  << imu.acc[2]*100.0f ;
					cout<<"gyrop_x: " <<imu.gyrop[0]*100.0f << " gyrop_y: " <<imu.gyrop[1]*100.0f << "gyrop_z" << imu.gyrop[2]*100.0f <<endl; 
					//imu.buf_pos = 0;
				}
			}
		}
		
	}
}
