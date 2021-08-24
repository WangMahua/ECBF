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
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <ecbf_uart/rc_info.h>
#include <ecbf_uart/qp_info.h>

#define K1 2
#define K2 3
#define X_UPPER_BOUND 1
#define X_LOWER_BOUND -1
#define Y_UPPER_BOUND 1
#define Y_LOWER_BOUND -1
#define Z_UPPER_BOUND 100
#define Z_LOWER_BOUND 0.5
#define ECBF_THESHOLD 0.1
#define VEL_UPPER_BOUND 8
#define VEL_LOWER_BOUND -8

#define g 9.81

using namespace std;

mutex imu_mutex;
imu_t imu;
double pos[3] = {0,0,0};
double velocity[3] = {0,0,0};
double last_velocity[3] = {0,0,0};
double last_pose[3] = {0,0,0};
double last_time,now_time = 0.0;
bool pose_init_flag = false;
int rc_ch7;
ros::Publisher vel_pub;

geometry_msgs::Vector3 vel_value;

double bond(double value){
	if (value>VEL_UPPER_BOUND)return VEL_UPPER_BOUND;
	else if (value<VEL_LOWER_BOUND)return VEL_LOWER_BOUND;	
	else return value; 
}
void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
		
		if (pose_init_flag == false){
			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;
			last_pose[0] = pos[0];
			last_pose[1] = pos[1];
			last_pose[2] = pos[2];
			last_time = now_time;
			pose_init_flag = true;	
		}
		else{
			now_time = msg->header.stamp.toSec();
//			now_time = ros::Time::now().toSec();
			pos[0] = msg->pose.position.x;
			pos[1] = msg->pose.position.y;
			pos[2] = msg->pose.position.z;

			double delta_time = now_time - last_time;

			velocity[0] =(pos[0] - last_pose[0])/(float)delta_time;
			velocity[1] =(pos[1] - last_pose[1])/(float)delta_time;
			velocity[2] =(pos[2] - last_pose[2])/(float)delta_time;
			

			
/*
			velocity[0] =(pos[0] - last_pose[0])/0.0083;
			velocity[1] =(pos[1] - last_pose[1])/0.0083;
			velocity[2] =(pos[2] - last_pose[2])/0.0083;
*/
			last_pose[0] = pos[0];


			velocity[0] = bond(velocity[0]);
			velocity[1] = bond(velocity[1]);
			velocity[2] = bond(velocity[2]);

			if(abs(velocity[0])>7 || abs(velocity[1])>7 || abs(velocity[2])>7){
				cout << "delta time: " << delta_time << '\n';
				cout << "velocity[0]: " << velocity[0] << '\n';
				cout << "velocity[1]: " << velocity[1] << '\n';
				cout << "velocity[2]: " << velocity[2] << '\n';
				
				cout << "delta_pos[0]: " << pos[0] - last_pose[0] << '\n';
				cout << "delta_pos[1]: " << pos[1] - last_pose[1] << '\n';
				cout << "delta_pos[2]: " << pos[2] - last_pose[2] << '\n';
				cout << "---\n";
			}

			last_pose[1] = pos[1];
			last_pose[2] = pos[2];
			last_time = now_time;

			last_velocity[0] = velocity[0];
			last_velocity[1] = velocity[1];
			last_velocity[2] = velocity[2];

			vel_value.x = velocity[0];
			vel_value.y = velocity[1];
			vel_value.z = velocity[2];
			vel_pub.publish(vel_value);	
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
	memcpy(&rc_ch7, &buf[18], sizeof(int));
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
    float vx = velocity[0];
    float vy = velocity[1];
    float vz = velocity[2];

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
    c_float l[3] = {-K1*(px-X_LOWER_BOUND)-K2*(vx), -K1*(py-Y_LOWER_BOUND)-K2*(vy), -K1*(pz-Z_LOWER_BOUND)-K2*(vz), };
    c_float u[3] = {K1*(X_UPPER_BOUND-px)-K2*(vx),K1*(Y_UPPER_BOUND-py)-K2*(vy), K1*(Z_UPPER_BOUND-pz)-K2*(vz), };
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



int main(int argc, char **argv){
	char c;
	imu.buf_pos = 0;
	float rc_throttle,acc[3];
	float rc_roll,rc_pitch,rc_yaw;
	float per2thrust_coeff[6] = {930.56,-3969,4983.2,-1664.5,482.08,-7.7146};
	float thrust2per_coeff[6] = {-1.11e-15,-3.88e-12,1.09e-8,-8.63e-6,3.62e-3,0};
	float force=0;
	float m = 1.39;
	float velocity[3];
	float roll_d,pitch_d,yaw_d,force_d,throttle_d;
	float acc_x,acc_y,acc_z;
	float acc_d[3] ;
	int boundary_flag,qp_flag ;
	ros::init(argc, argv, "ecbf_main");
	serial_init((char *)"/dev/ECBF_UART", 115200);
	ros::NodeHandle n;
	
	ros::Publisher qp_pub = n.advertise<geometry_msgs::Twist>("qp", 1); 
	ros::Publisher debug_rc_pub = n.advertise<ecbf_uart::rc_info>("rc_info", 1); 
	ros::Publisher debug_qp_pub = n.advertise<ecbf_uart::qp_info>("qp_info", 1); 
	ros::Subscriber pos_sub = n.subscribe("/vrpn_client_node/MAV1/pose", 1, pos_callback);
	vel_pub = n.advertise<geometry_msgs::Vector3>("vel_info", 1); 
	cout<<"start\n";
	ros::Rate rate(400);

	/* debug */
	ecbf_uart::rc_info debug_rc;	
	ecbf_uart::qp_info debug_qp;	

	while(ros::ok()){
		
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+')
			{

			//	for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
			//		cout << imu.buf[i];
			//	cout<<endl;
				if(imu_decode(imu.buf)==0)
				{
                			ros::spinOnce();
					rc_roll = -imu.acc[0]*M_PI/180.0;
					rc_pitch = -imu.acc[1]*M_PI/180.0;
					rc_yaw = imu.acc[2]*M_PI/180.0;
					rc_throttle = imu.gyrop[0];
					//rc_ch7 = imu.gyrop[1];
					rc_yaw = -90*M_PI/180.0;
					rc_yaw = 0;
					force = 0 ;
					for(int i = 0;i<6;i++){
						force += per2thrust_coeff[5-i]*pow(rc_throttle*0.01,i);
					}
					force = force/1000*4*g;
					force = force<0?0:force;

//					cout <<"roll:" << rc_roll<<'\n';
//					cout <<"pitch:" << rc_pitch<<'\n';
//					cout <<"yaw:" << rc_yaw<<'\n';
//					cout <<"throttle:" << rc_throttle<<'\n';
					

					cout <<"rc_mode: "<< rc_ch7 << '\n';

					if(rc_ch7>1.1 && pos[2]>ECBF_THESHOLD){ // rc mode change & height > threshold
						acc_x = g*(rc_roll*cos(rc_yaw)+rc_pitch*sin(rc_yaw));
                                        	acc_y = g*(rc_roll*sin(rc_yaw)-rc_pitch*cos(rc_yaw));
                                        	acc_z = force/m-g;


                                        	
                                        	acc_d[0] = acc_x;
                                        	acc_d[1] = acc_y;
                                        	acc_d[2] = acc_z;

                                        	cout << "acc[0]:"<<acc_d[0]<<'\n';
                                       		cout << "acc[1]:"<<acc_d[1]<<'\n';
                                        	cout << "acc[2]:"<<acc_d[2]<<'\n';
						ros::spinOnce();
                                        	qp_solve(acc_d);
                                        	cout << "qp acc[0]:"<<acc_d[0]<<'\n';
                                        	cout << "qp acc[1]:"<<acc_d[1]<<'\n';
                                        	cout << "qp acc[2]:"<<acc_d[2]<<'\n';

                                        	roll_d = -((cos(rc_yaw)*acc_d[0]+sin(rc_yaw)*acc_d[1])/g*180.0/M_PI);
                                        	pitch_d = -((-cos(rc_yaw)*acc_d[1]+sin(rc_yaw)*acc_d[0])/g*180.0/M_PI);
                                        	force_d = m*(acc_d[2] +g);
                                        	force_d = force_d /4 *1000 /9.81;

                                        	cout << "roll_d:"<<roll_d<<'\n';
                                        	cout << "pitch_d:"<<pitch_d<<'\n';

						throttle_d = 0;
                                        	for(int i = 0;i<6;i++){

                                                	throttle_d += thrust2per_coeff[5-i]*pow(force_d,i)*100;
                                        	}

                                        	cout << "force_d:"<<force_d<<"\t thrust:"<<throttle_d<<'\n';

						qp_flag=0;
						if(abs(pitch_d-imu.acc[1])>0.2 || abs(roll_d-imu.acc[0])>0.2){
							qp_flag=15;
						}

					}else{
						qp_flag=0;
						
					       	cout <<"not triggered!\n";	
						roll_d = imu.acc[0];
						pitch_d = imu.acc[1];
						throttle_d = imu.gyrop[0];
					
					}

					boundary_flag = 0;
					if(abs(pos[0])>1 || abs(pos[1])>1){
						boundary_flag = 10;
					}
	
				
					//debug
					debug_rc.rc_roll = imu.acc[0];
					debug_rc.rc_pitch = imu.acc[1];
					debug_rc.rc_throttle = imu.gyrop[0];
					debug_rc.qp_roll = roll_d;
					debug_rc.qp_pitch = pitch_d;
					debug_rc.qp_throttle = throttle_d;
					debug_rc_pub.publish(debug_rc);

					debug_qp.acc_x_old = acc_x;
					debug_qp.acc_y_old = acc_y;
					debug_qp.acc_z_old = acc_z;
					debug_qp.acc_x_new = acc_d[0];
					debug_qp.acc_y_new = acc_d[1];
					debug_qp.acc_z_new = acc_d[2];
					debug_qp_pub.publish(debug_qp);

					//send sol to uart
					send_pose_to_serial(roll_d,pitch_d,throttle_d,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
					//send_pose_to_serial(imu.acc[0],imu.acc[1],imu.gyrop[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0);

					vel_value.x = velocity[0];
					vel_value.y = velocity[1];
					vel_value.z = velocity[2];
					vel_pub.publish(vel_value);


				}
			}
		}


					//rate.sleep();
					
	}
}