#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
from copy import copy

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg

from lab3.planners import SinusoidPlanner

class Executor(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(100)
        self.state = BicycleStateMsg()
        self.state_np = np.ones(4) * float('inf')
        self.states = []
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

        for (t, cmd, state) in plan:
            self.cmd(cmd)
            self.rate.sleep()
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
        self.state_np = np.array([msg.x, msg.y, msg.theta, msg.phi])
        self.states.append(self.state_np)
        # print'Current state:'
        # print(self.state)

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd(BicycleCommandMsg())

    def format_path_to_plot(self, path):
        t = [p[0] for p in path]
        commands = np.array([np.array([p[1].linear_velocity, p[1].steering_rate]) for p in path])
        states = np.array([np.array([p[2].x, p[2].y, p[2].theta, p[2].phi]) for p in path])
        return t, commands, states

    def plot_traj(self, pred_path, true_states, goal_state, dt):
        true_states = np.array(true_states)
        pred_t, pred_commands, pred_states = self.format_path_to_plot(pred_path)
        pred_t = dt * np.arange(0, len(pred_t))

        true_t = np.linspace(pred_t[0], pred_t[-1], len(true_states))

        f, axarr = plt.subplots(5, 1)
        for i in range(5):
            if i == 4:
                axarr[i].scatter([true_states[:, 0][0]], [true_states[:, 1][0]], color='k', label='start')
                axarr[i].scatter([true_states[:, 0][-1]], [true_states[:, 1][-1]], color='r', label='end')
                axarr[i].scatter([goal_state[0]], [goal_state[1]], color='b', marker='*', label='goal')
                axarr[i].plot(true_states[:, 0], true_states[:, 1], color='b', label='true')
                axarr[i].plot(pred_states[:, 0], pred_states[:, 1], color='g', label='predicted')
            else:
                axarr[i].plot(pred_t, pred_states[:, i], linestyle=':', color='g', label='predicted')
                axarr[i].plot(true_t, true_states[:, i], linestyle='--', color='b', label='true')

        ylabels = ['x', 'y', 'theta', 'phi']
        for i, ax in enumerate(axarr.flat):
            if i != 4:
                ax.set(xlabel='time (s)', ylabel=ylabels[i])
            else:
                ax.set(xlabel='x', ylabel='y')

        plt.legend()
        plt.show()

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

if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    args = parse_args()

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()
    
    ex = Executor()

    print "Initial State"
    print ex.state

    l, max_phi, max_u1, max_u2 = 0.3, 0.3, 2, 3
    p = SinusoidPlanner(l, max_phi, max_u1, max_u2)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    goal_state_np = np.array([args.x, args.y, args.theta, args.phi])
    dt = 1e-2
    delta_t = 4

    total_plan = []

    while not (np.abs(ex.state_np - goal_state_np) < 0.1).all():
    # for i in range(3):
        # import pdb; pdb.set_trace()
        print 'PLANNING'
        plan = p.plan_to_pose(ex.state, goalState, dt, delta_t)
        total_plan.extend(plan)

        print "Predicted Initial State"
        print plan[0][2]
        print "Predicted Final State"
        print plan[-1][2]

        ex.execute(plan)
        print "Final State"
        print ex.state

    ex.plot_traj(total_plan, ex.states, goal_state_np, dt)
