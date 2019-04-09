#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import numpy as np
import tf2_ros
import tf


class BangBang(object):
    """docstring for BangBang"""
    def __init__(self):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.rate = rospy.Rate(1)

    def run(self):


    	####### translation in theta ############
        # for i in range(40):
        #     self.turn(1, -1)
        #     # if rospy.is_shutdown():
        #     #     self.cmd(0,0)
        #     #     break
        #     # self.strafe()
        #     self.rate.sleep() 
        #     self.rate.sleep()
        #     self.cmd(-2, 0)
        #     self.rate.sleep()
        # self.cmd(0, 0)
        ###### translation in x #############
        # for i in range(20):
        #     self.cmd(1, 0)
        #     self.rate.sleep()
        # self.cmd(0, 0)
    	###### translation in y ############
        for i in range(2):
            self.rate.sleep()
            if rospy.is_shutdown():
                self.cmd(0,0)
                break
            self.strafe()


    def strafe(self):
        # self.turn(1, 1)
        # self.rate.sleep()
        # self.cmd(2, 0)
        # self.rate.sleep()
        # self.rate.sleep()
        # self.turn(1, -1)
        # self.rate.sleep()
        # self.cmd(-2, 0)
        # self.rate.sleep()
        # self.rate.sleep()
        # self.cmd(0, 0)

        self.turn(0.25, 1)
        self.rate.sleep()
        self.cmd(0.5, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.turn(0.25, -1)
        self.rate.sleep()
        self.cmd(-0.5, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.cmd(0, 0)




        

    def turn(self, mag, d):
        self.cmd(mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, -d*mag)
        self.rate.sleep()
        self.cmd(mag, -d*mag)
        self.rate.sleep()
        self.cmd(0, 0)

    def go_to_xy_position(self, x, y):
    	phi =  np.arctan([])



    def cmd(self, u1, u2):
    	"""
    	BicycleCommandMsg: The commands to a bicycle model robot (v, phi)
		float64 linear_velocity
		float64 steering_rate
		"""
        self.pub.publish(BicycleCommandMsg(u1, u2))
        print(BicycleCommandMsg(u1, u2))
        print('################')

if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)
    
    b = BangBang()
    b.run()




