#include <iostream>
#include <ecbf_uart/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include "osqp.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#define K1 3
#define K2 2

using namespace std;

mutex imu_mutex;
imu_t imu;
float pos[3] = {1,1,1};
float vel[3] = {1,1,1};
float now_time,last_time;
int time_init = 0;

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(time_init<1){
		now_time = (float)msg->header.stamp.sec + (float)msg->header.stamp.nsec;
		last_time = now_time;
		pos[0] = msg->pose.position.x;
		pos[1] = msg->pose.position.y;
		pos[2] = msg->pose.position.z;
		time_init++;
	}else{
		float delta_time ;
		float last_pos[3];
		last_pos[0] = pos[0];
		last_pos[1] = pos[1];
		last_pos[2] = pos[2];
		last_time = now_time;
		now_time = (float)msg->header.stamp.sec + (float)msg->header.stamp.nsec;
		pos[0] = msg->pose.position.x;
		pos[1] = msg->pose.position.y;
		pos[2] = msg->pose.position.z;
		delta_time = (float)now_time - (float)last_time;
		vel[0] = (pos[0]-last_pos[0])/(float)delta_time;
		vel[1] = (pos[1]-last_pos[1])/(float)delta_time;
		vel[2] = (pos[2]-last_pos[2])/(float)delta_time;	
	
	}

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

float* qp_solve(float* acc){
    float px = pos[0];
    float py = pos[1];
    float pz = pos[2];
    float vx = vel[0];
    float vy = vel[1];
    float vz = vel[2];

    // Load problem data
    c_float P_x[3] = {1.0, 1.0, 1.0, };
    c_int P_nnz = 3;
    c_int P_i[3] = {0, 1, 2, };
    c_int P_p[4] = {0, 1, 2, 3,};
    c_float q[3] = {-acc[0], -acc[1], -acc[2]};
    c_float A_x[3] = {1.0, 1.0, 1.0, };
    c_int A_nnz = 3;
    c_int A_i[3] = {0, 1, 2, };
    c_int A_p[4] = {0, 1, 2, 3, };
    c_float l[3] = {-K1*(1+px)-K2*(vx), -K1*(1+py)-K2*(vy), -K1*(1+pz)-K2*(vz), };
    c_float u[3] = {K1*(1-px)+K2*(vx),K1*(1-py)+K2*(vy), K1*(1-pz)+K2*(vz), };
/*
	cout << "px:"<< px<<"\n";
	cout << "py:"<< py<<"\n";
	cout << "pz:"<< pz<<"\n";
	cout << "vx:"<< vx<<"\n";
	cout << "vy:"<< vy<<"\n";
	cout << "vz:"<< vz<<"\n";
	*/
	c_int n = 3;
    	c_int m = 3;

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
	acc[0] = work->solution->x[0];
	acc[1] = work->solution->x[1];
	acc[2] = work->solution->x[2];

	return acc;
}

#define g 9.81

int imu_thread_entry(){
	char c;
	imu.buf_pos = 0;
	float rc_throttle,acc[3];
	float rc_roll,rc_pitch,rc_yaw;
	float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
	float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};
	float force=0;
	float m = 1.42;
	float pose[3];
	float velocity[3];
	float roll_d,pitch_d,yaw_d,force_d;
	float acc_x,acc_y,acc_z;
	ros::NodeHandle n;
	ros::Publisher qp_pub = n.advertise<geometry_msgs::Twist>("qp", 1); 
	ros::Subscriber pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 1, pos_callback);
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
					rc_roll = -imu.acc[0]*M_PI/180.0;
					rc_pitch = -imu.acc[1]*M_PI/180.0;
					rc_yaw = imu.acc[2]*M_PI/180.0;
					rc_throttle = imu.gyrop[0];
					rc_yaw = -90*M_PI/180.0;
					rc_yaw = 0;
					force = 0 ;
					for(int i = 0;i<6;i++){
						force += per2thrust_coeff[5-i]*pow(rc_throttle*0.01,i);
					}
					force = force/1000*4*g;
					force = force<0?0:force;
/*
					cout <<"roll:" << rc_roll<<'\n';
					cout <<"pitch:" << rc_pitch<<'\n';
					cout <<"yaw:" << rc_yaw<<'\n';
					cout <<"throttle:" << rc_throttle<<'\n';
*/
						
					acc_x = g*(rc_roll*cos(rc_yaw)+rc_pitch*sin(rc_yaw));
					acc_y = g*(rc_roll*sin(rc_yaw)-rc_pitch*cos(rc_yaw));
					acc_z = force/m-g;

					
					float acc_d[3] ;
					acc_d[0] = acc_x;
					acc_d[1] = acc_y;
					acc_d[2] = acc_z;
					ros::spinOnce();

					cout << "acc[0]:"<<acc_d[0]<<'\n';
					cout << "acc[1]:"<<acc_d[1]<<'\n';
					cout << "acc[2]:"<<acc_d[2]<<'\n';

					qp_solve(acc_d);
					cout << "qp acc[0]:"<<acc_d[0]<<'\n';
					cout << "qp acc[1]:"<<acc_d[1]<<'\n';
					cout << "qp acc[2]:"<<acc_d[2]<<'\n';
				
					roll_d = (cos(rc_yaw)*acc_d[0]+sin(rc_yaw)*acc_d[1])/g*180.0/M_PI;
					pitch_d = (-cos(rc_yaw)*acc_d[1]+sin(rc_yaw)*acc_d[0])/g*180.0/M_PI;
					force_d = m*(acc_d[2] +g);
					force_d = force_d /4 *1000 /9.81;
/*
					cout << "roll_d:"<<roll_d<<'\n';
					cout << "pitch_d:"<<pitch_d<<'\n';
					*/
					double throttle_d=0;
                                     	for(int i = 0;i<6;i++){
                                        	 throttle_d += thrust2per_coeff[5-i]*pow(force_d,i)*100;
                               		}
/*
					cout << "force_d:"<<force_d<<"\t thrust:"<<throttle_d<<'\n';

*/

				}
			}
		}
		
	}
}
