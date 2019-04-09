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

class SinusoidPlanner():
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

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=5):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You 
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi 

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # This bit hasn't been exhaustively tested, so you might hit a singularity anyways
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))
        # if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
        #     raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")

        if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )

        x_path =        self.steer_x(
                            start_state, 
                            goal_state, 
                            0, 
                            dt, 
                            delta_t
                        )
        phi_path =      self.steer_phi(
                            x_path[-1][2], 
                            goal_state, 
                            x_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        alpha_path =    self.steer_alpha(
                            phi_path[-1][2], 
                            goal_state, 
                            phi_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        # start_state_v = self.state2v(alpha_path[-1][2])
        # goal_state_v = self.state2v(goal_state)
       
        # start_path = alpha_path
        # end_state = start_path[-1]
        # end_path[-1]

        # delta_y = goal_state.y - alpha_path[-1][2].y 
        # goal_state1 = BicycleStateMsg(goal_state.x, 0, goal_state.theta, goal_state.phi)
        # # goal_state1.y = 0
        # alpha_path1 = alpha_path
        # y_path = []
        # rem = delta_y
        # print('remy', rem)
        # max_y_perunit = 0.3
        # while rem > max_y_perunit: 
        #     print('rem', rem)
        #     goal_state1.y += max_y_perunit
        #     y_path +=        self.steer_y(
        #                     alpha_path1[-1][2], 
        #                     goal_state1, 
        #                     alpha_path1[-1][0] + dt, 
        #                     dt, 
        #                     delta_t
        #                 )     
        #     alpha_path1 = y_path
        #     # fix_theta = alpha_path1[-1][2]
        #     # fix_theta_time = alpha_path1[-1][0]

        #     # y_path +=    self.steer_alpha(
        #     #                 fix_theta, 
        #     #                 goal_state, 
        #     #                 fix_theta_time + dt, 
        #     #                 dt, 
        #     #                 delta_t
        #     #             )

        #     rem -= max_y_perunit
        #     print("did a ypath")
        # y_path +=        self.steer_y(
        #                     alpha_path1[-1][2], 
        #                     goal_state, 
        #                     alpha_path1[-1][0] + dt, 
        #                     dt, 
        #                     delta_t
        #                 )     

        y_path =        self.steer_y(
                            alpha_path[-1][2], 
                            goal_state, 
                            alpha_path[-1][0] + dt, 
                            dt, 
                            delta_t
                            )
        path = []
        for p in [x_path, phi_path, alpha_path, y_path]:
            path.extend(p)

        print("done!")
        return path

    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        start_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_phi(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the phi direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # ************* IMPLEMENT THIS
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_phi = goal_state_v[1] - start_state_v[1]

        v1 = 0
        v2 = delta_phi/delta_t

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_alpha(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the alpha direction.  
        Remember dot{alpha} = f(phi(t))*u_1(t) = f(frac{a_2}{omega}*sin(omega*t))*a_1*sin(omega*t)
        also, f(phi) = frac{1}{l}tan(phi)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_alpha = goal_state_v[2] - start_state_v[2]

        omega = 2*np.pi / delta_t

        a2 = min(1, self.phi_dist*omega)
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
        phi_fn = lambda t: (a2/omega)*np.sin(omega*t) + start_state_v[1]
        integrand = lambda t: f(phi_fn(t))*np.sin(omega*t) # The integrand to find beta
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        a1 = (delta_alpha*omega)/(np.pi*beta1)

              
        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)


    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the y direction. 
        Remember, dot{y} = g(alpha(t))*v1 = frac{alpha(t)}{sqrt{1-alpha(t)^2}}*a_1*sin(omega*t)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        
        # ************* IMPLEMENT THIS
        # will likely have to put in some logic to break this into multiple manuevers to avoid hitting limits, consider how big a2 and a1 are allowed to be
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_y = goal_state_v[3] - start_state_v[3]

        omega = 2*np.pi / delta_t

        a2 = min(1, self.phi_dist*omega)
        # a2 = min(1, self.max_u2)
        ######### TODO #################
        # do all the integration and math stuffs in here
        # anything using a1 has to be in a "loop?" for  binary search
        # so for each "loop?" we will have a set function for alpha(t)
        # pretty sure that alpha(2pi/w) needs to be the start point and not violate any bounds
        
        # let's do the solving for g(alpha(t)) first
        g = lambda alpha: alpha/np.sqrt(1-alpha**2) # from cannonical model
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model - note how a function is declared/constructed in python
        phi_fn = lambda t: (a2/(2*omega))*np.sin(2*omega*t) + start_state_v[1]

        # these bounds for a1 limit the x-motion during sinusoids to 4m, which seems like a reasonable limit
        a1_max = 2*omega # def possible to change this
        a1_min = -2*omega

        #a1 = min(1, self.phi_dist*omega)
        a1 = 0 # initial value
        alpha_integrand = lambda tau: f(phi_fn(tau))*a1*np.sin(omega*tau) # The integrand to find beta      
        integrand = lambda t: g(quad(alpha_integrand, 0, t)[0])*np.sin(omega*t)
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]
        y = np.pi*a1*beta1 / omega

        num = 0

        tolerance = .001 # allows us to change how accuracte we want to be
        while abs(delta_y - y) >= tolerance:
            if delta_y - y >= tolerance and num < 100:
                a1_min = a1 # notice how I've implemented binary search
                a1 = (a1+a1_max)/2
            if delta_y - y <= -tolerance and num < 100:
                a1_max = a1
                a1 = (a1+a1_min)/2
            # if num == 100:
            #     a1_max = 2*omega # def possible to change this
            #     a1_min = -2*omega
            #     a1 = 0
            # if delta_y - y <= tolerance and num >= 100:
            #     a1_min = a1 # notice how I've implemented binary search
            #     a1 = (a1+a1_max)/2
            # if delta_y - y >= -tolerance and num >= 100:
            #     a1_max = a1
            #     a1 = (a1+a1_min)/2

            alpha_integrand = lambda tau: f(phi_fn(tau))*a1*np.sin(omega*tau) # The integrand to find beta      
            integrand = lambda t: g(quad(alpha_integrand, 0, t)[0])*np.sin(omega*t)
            beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]
            y = np.pi*a1*beta1 / omega
            num += 1
            # if num >= 200:
            #     raise ValueError("something messed up in binary search")
        
        ################################

        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(2*omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray` 
            x, phi, alpha, y
        """
        return np.array([state.x, state.phi, np.sin(state.theta), state.y])

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)
            list of (time, v1, v2) commands
        start_state : :obj:`BicycleStateMsg`
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state.theta)
            u2 = v2
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]

            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        return path