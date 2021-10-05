 #!/usr/bin/env python

import threading
import rospy
import time
import sys
import serial
from thread.cal_vel import thread_vel
from thread.uart import thread_uart
from thread.lidar_qp import thread_lidar

def main():
    rospy.init_node('main')
    com = serial.Serial('/dev/ECBF_UART',115200)
    
    first_thread = threading.Thread(target = thread_vel, args=("Hi",))
    second_thread = threading.Thread(target = thread_uart, args=("Hello",))
    third_thread = threading.Thread(target = thread_lidar, args=("Hey",))
    first_thread.start()
    second_thread.start()
    third_thread.start()
    rospy.spin()


if __name__ == '__main__' :
    main()