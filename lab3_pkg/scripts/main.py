#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import sys
import argparse
from matplotlib import pyplot as plt

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg

from lab3.planners import SinusoidPlanner, BangbangPlanner

class Exectutor(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(10) # for turtlebot # this is in Hz
        # self.rate = rospy.Rate(100) # for turtlesim
        self.state = BicycleStateMsg()
        self.actual_state = []
        rospy.on_shutdown(self.shutdown)

    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """
        if len(plan) == 0:
            return
        print("EXECUTE")
        for (t, cmd, state) in plan:
            cmd.linear_velocity = cmd.linear_velocity*1.2
            cmd.steering_rate = cmd.steering_rate
            self.cmd(cmd)
            # print("@@@@@@@@@@", cmd)
            self.rate.sleep()
            self.actual_state.append(self.state)
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : :obj:`BicycleCommandMsg`
        """
        self.pub.publish(msg)

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = msg

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd(BicycleCommandMsg())

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0.0, help='Desired position in x')
    parser.add_argument('-y', type=float, default=0.0, help='Desired position in y')
    parser.add_argument('-theta', type=float, default=0.0, help='Desired turtlebot angle')
    parser.add_argument('-phi', type=float, default=0.0, help='Desired angle of the (imaginary) steering wheel')
    return parser.parse_args()

def plot_results(plans, states, dt):

    times = []
    x_values = []
    y_values = []
    theta_values = []
    phi_values = []
    actual_x = []
    actual_y = []
    actual_theta = []
    actual_phi = []
    time = 0
    max_theta = 0
    num = 0
    for i in range(len(plans)):
        _, commandMsg, stateMsg = plans[i]
        time += dt
        times.append(time)
        x_values.append(stateMsg.x)
        y_values.append(stateMsg.y)
        theta_values.append(stateMsg.theta)  
        phi_values.append(stateMsg.phi)
        state = states[i]
        actual_x.append(state.x)
        actual_y.append(state.y)
        actual_theta.append(state.theta)
        actual_phi.append(state.phi)

    plt.figure()
    values = ('x', 'y', 'theta', 'phi')
    plt.subplot(3,2,1)
    plt.plot(times, actual_x, label='Actual')
    plt.plot(times, x_values, label='Desired')
    plt.xlabel("Time (t)")
    plt.ylabel("x trajectory")

    plt.subplot(3,2,2)
    plt.plot(times, actual_y, label='Actual')
    plt.plot(times, y_values, label='Desired')
    plt.xlabel("Time (t)")
    plt.ylabel("y trajectory")

    plt.subplot(3,2,3)
    plt.plot(times, actual_theta, label='Actual')
    plt.plot(times, theta_values, label='Desired')
    plt.xlabel("Time (t)")
    plt.ylabel("change in theta")

    plt.subplot(3,2,4)
    plt.plot(times, actual_phi, label='Actual')
    plt.plot(times, phi_values, label='Desired')
    plt.xlabel("Time (t)")
    plt.ylabel("change in phi")

    plt.subplot(3,2,5)
    plt.plot(actual_x, actual_y, label='Actual')
    plt.plot(x_values, y_values, label='Desired')
    plt.xlabel("x")
    plt.ylabel("y")

    print "Close the plot window to continue"
    plt.show()


if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    args = parse_args()

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()
    
    ex = Exectutor()

    print "Initial State"
    print ex.state


    p = SinusoidPlanner(0.3, 0.3, .8, 3)
    plan = []
    # p = BangbangPlanner(0.3, 0.3, 2, 3)
    # goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    # plan = p.plan_to_pose(ex.state, goalState, 0.01, 5)

    ############ fix y #####################
    # rem = args.y
    # last_y = 0
    # max_y_perunit = .2
    # while rem > max_y_perunit:
    #     print("in while!!!!!!!!!!!!!!!!")
    #     goalState = BicycleStateMsg(0, max_y_perunit, 0, 0)
    #     # plan += p.plan_to_pose(ex.state, goalState, 0.01, 5)
    #     plan1 = p.plan_to_pose(ex.state, goalState, 0.1, 7)
    #     for i in range(len(plan1)):
    #         plan1[i][2].y += last_y
    #     last_y = plan1[len(plan1)-1][2].y
    #     plan += plan1
    #     print("length of each plan:", len(plan))
    #     rem -= max_y_perunit
    # goalState = BicycleStateMsg(args.x, rem, args.theta, args.phi)

    # plan1 = p.plan_to_pose(ex.state, goalState, 0.1, 7)
    # for i in range(len(plan1)):
    #     plan1[i][2].y += last_y
    # plan += plan1


    ############ fix theta #####################
    rem_theta = args.theta
    last_theta = 0
    # print('math', np.pi/6)
    while rem_theta > np.pi/6:
        print("in while!!!!!!!!!!!!!!!!", rem_theta)
        goalState = BicycleStateMsg(0, 0, np.pi/6, 0)
        plan1 = p.plan_to_pose(ex.state, goalState, 0.1, 5)
        for i in range(len(plan1)):
            plan1[i][2].theta += last_theta
        last_theta = plan1[len(plan1)-1][2].theta
        plan += plan1
        # print("length of each plan:", len(plan))
        rem_theta -= np.pi/6

    goalState = BicycleStateMsg(args.x, args.y, rem_theta, args.phi)

    plan1 = p.plan_to_pose(ex.state, goalState, 0.1, 5)
    for i in range(len(plan1)):
        plan1[i][2].theta += last_theta
    plan += plan1


    # max_y_perunit = 0.2
    # while goalState.y - ex.state.y > max_y_perunit:
    #     goal = ex.state
    #     # goal = BicycleStateMsg(ex.state.x, max_y_perunit, ex.state.th, 0)
    #     print(goal)
    #     goal.y += max_y_perunit
    #     plan += p.plan_to_pose(ex.state, goal, 0.01, 5)
    #     print("goal.y ", goal.y)
    #     print("length of each plan:", len(plan))


    # # print("length of Final plan:", len(plan))
    # print(plan)
    
    print "Predicted Initial State"
    print plan[0][2]
    print "Predicted Final State"
    print plan[-1][2]

    ex.execute(plan)
    print "Final State"
    print ex.state
    plot_results(plan, ex.actual_state, 0.1)

