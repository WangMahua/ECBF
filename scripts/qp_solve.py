#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import osqp
import numpy as np
from scipy import sparse

# K1 K2 > 0 && K1*K1 >4*K2
K1 = 3
K2 = 2

def callback(data):
	# Define problem data
	acc_x = data.linear.x
	acc_y = data.linear.y
	acc_z = data.linear.z 
	
	px = 0
	py = 0
	pz = 0
	vx = 0
	vy = 0
	vz = 0

	P = sparse.csc_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	q = np.array([-acc_x, -acc_y, -acc_z])
	A = sparse.csc_matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
	l = np.array([-K1*(1-px)-K2*(vx), -K1*(1-py)-K2*(vy), -K1*(1-pz)-K2*(vz)])
	u = np.array([K1*(1-px)+K2*(vx),K1*(1-py)+K2*(vy), K1*(1-pz)+K2*(vz)])

	# Create an OSQP object
	prob = osqp.OSQP()

	# Setup workspace and change alpha parameter
	prob.setup(P, q, A, l, u, alpha=1.0)

	# Solve problem
	res = prob.solve()
	print(res)
    
def listener():

    rospy.init_node('qp_solve', anonymous=True)
    rospy.Subscriber("qp", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

