#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
from scipy.integrate import quad
import sys
from copy import copy

import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import tf2_ros
import tf



class BangbangPlanner():
    def __init__(self, l, max_phi, max_u1, max_u2):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose

        Parameters
        ----------
        l : float
            length of car
        """
        self.l = l
        self.max_phi = max_phi
        self.max_u1 = max_u1
        self.max_u2 = max_u2

        self.state = None
        self.path = []
        self.t = 0
        self.kp = 3



    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))
        # if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
        #     raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")

        # if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
        #     raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.state = start_state
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )

        e_theta = goal_state.theta - self.state.theta
        # print(e_theta)
        while e_theta > 0.02:
        # for i in range(10):
            self.turn(0.5, -1, dt)
            # self.turn(e_theta * self.kp, -1, dt)
            e_theta = goal_state.theta - self.state.theta 
        while e_theta < -0.02:
            self.turn(0.5, 1, dt)
            # self.turn(e_theta * self.kp, 1, dt)
            e_theta = goal_state.theta - self.state.theta


        e_y = goal_state.y - self.state.y
        print(e_y)
        while e_y > 0.1:
            self.strafe(0.25, -1, dt)
            e_y = goal_state.y - self.state.y          
        while e_y < -0.1:
            self.strafe(0.25, 1, dt)
            e_y = goal_state.y - self.state.y


        e_x = goal_state.x - self.state.x
        while e_x > 0.02:
            self.straight(0.2,1, dt)
            e_x = goal_state.x - self.state.x          
        while e_x < -0.02:
            self.straight(0.2,-1, dt)
            e_x = goal_state.x - self.state.x



        # e_x = goal_state.x

        # self.cmd(0, 0, dt)
        return self.path



    def turn(self, mag, d, dt): 
        self.cmd(mag, d*mag, dt)
        self.cmd(-mag, d*mag, dt)
        self.cmd(-mag, -d*mag, dt)
        self.cmd(mag, -d*mag, dt)
        self.cmd(0, 0, 0)

    def strafe(self, mag, d, dt):
        # self.turn(1, 1, dt)
        # self.cmd(2, 0, dt)
        # self.turn(1, -1, dt)
        # self.cmd(-2, 0, dt)
        # self.cmd(0, 0, dt)

        # self.turn(1, d, dt)
        # self.cmd(2, 0, dt)
        # self.turn(1, -d, dt)
        # self.cmd(-2, 0, dt)
        # self.cmd(0, 0, 0)


        #JOhnny 
        one = mag
        two = 2*mag

        self.turn(one, d, dt)
        self.cmd(two, 0, dt)
        self.turn(one, -d, dt)
        self.cmd(-two, 0, dt)
        self.cmd(0, 0, 0)

        #JOhnny



    def straight(self, mag, d, dt):
        self.cmd(mag*d, 0, dt)

    def cmd(self, u1, u2, dt):
        """
        BicycleCommandMsg: The commands to a bicycle model robot (v, phi)
        float64 linear_velocity
        float64 steering_rate
        """
        # self.path.append((self.t, u1, u2))
        
        curr_state = self.state
        # for i, (self.t, u1, u2) in enumerate(path):
        cmd_u = BicycleCommandMsg(u1, u2)
        self.path.append([self.t, cmd_u, curr_state])
        print([self.t, cmd_u, curr_state])

        self.state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )
        # print(self.state)
        self.t += dt








    