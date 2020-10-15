#include <iostream>
#include <vins_uart/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>

using namespace std;

mutex imu_mutex;
imu_t imu;
float imu_average[3] = {0,0,-9.8};
int N = 100;
double calc_deviation(float* x){
	double sum = 0;
	double average = 0;
	double result = 0;
	for(int i = 0 ; i<N ; i++ ){
		sum += x[i];
	}
	average = sum / N ;
	sum =0;
	for(int i = 0 ; i<N ; i++){
		sum += pow((x[i]-average),2);
	}
	result = sqrt(sum/N);
	return result;
	
}


uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = IMU_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	static float x_array_uart[100];
	static int flag = 0;
	static int deviation_flag = 0;
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
	
	if(flag<10){
		imu_average[0]+=imu.acc[0];
		imu_average[1]+=imu.acc[1];
		imu_average[2]+=imu.acc[2];
		flag++;
	}
	else if(flag==10){
		imu_average[0] /= 11;
		imu_average[1] /= 11;
		imu_average[2] /= 11;
		flag++;
	}

	if(deviation_flag<N){
		x_array_uart[deviation_flag] = imu.acc[0];
		deviation_flag++;
	}
	else if(deviation_flag==N){
		imu.deviation_acc = calc_deviation(x_array_uart);

		deviation_flag++;
	}
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

int flag_imu = 12;
void flag_imu_callback(std_msgs::Int32 data){
	flag_imu = data.data;
}
int imu_thread_entry(){
	sensor_msgs::Imu IMU_data;
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 5);
	ros::Subscriber sub = n.subscribe("flag_imu", 5,flag_imu_callback);

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
					if(flag_imu == 0)
					{
						IMU_data.header.stamp = ros::Time::now();
						IMU_data.header.frame_id = "base_link";
						IMU_data.linear_acceleration.x = imu.acc[0];
						IMU_data.linear_acceleration.y = imu.acc[1];
						IMU_data.linear_acceleration.z = imu.acc[2];
						IMU_data.angular_velocity.x = imu.gyrop[0]*M_PI/180.0f;
						IMU_data.angular_velocity.y = imu.gyrop[1]*M_PI/180.0f;
						IMU_data.angular_velocity.z = imu.gyrop[2]*M_PI/180.0f;
						IMU_data.angular_velocity_covariance={1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07};
						IMU_data.linear_acceleration_covariance={8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08};

						pub.publish(IMU_data);
					}
					if(flag_imu == 1)
					{
						IMU_data.header.stamp = ros::Time::now();
						IMU_data.header.frame_id = "base_link";
						IMU_data.linear_acceleration.x = imu_average[0];
						IMU_data.linear_acceleration.y = imu_average[1];
						IMU_data.linear_acceleration.z = imu_average[2];
						IMU_data.angular_velocity.x = 0;
						IMU_data.angular_velocity.y = 0;
						IMU_data.angular_velocity.z = 0;
						IMU_data.angular_velocity_covariance={1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07};
						IMU_data.linear_acceleration_covariance={8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08};

						pub.publish(IMU_data);
					}
//					cout<<"imu_acc_x: "<<imu.acc[0] << " imu_acc_y: " <<imu.acc[1] << " imu_acc_z: "  << imu.acc[2] ;
//					cout<<"gyrop_x: " <<imu.gyrop[0] << " gyrop_y: " <<imu.gyrop[1] << "gyrop_z" << imu.gyrop[2] <<endl; 
					//imu.buf_pos = 0;
//					cout<<"flag: "<<flag_imu<<endl;
//					cout<<"average:" << imu_average[0] <<" " <<imu_average[1] << " " << imu_average[2] << "deviation" << imu.deviation_acc << endl;
				}
			}
		}
		
	}
}
