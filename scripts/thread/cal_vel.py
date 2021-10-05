 #!/usr/bin/env python

import threading
import rospy
import time
import sys



def thread_vel(x):
    time.sleep(0.5)
    while not rospy.is_shutdown():
        print("This is the vel thread",x)
    
