#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad
import sys
from copy import copy

import rospy
from std_srvs.srv import Empty as EmptySrv
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import tf2_ros
import tf

class BangBang(object):
    """docstring for BangBang"""
    def __init__(self):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.rate = rospy.Rate(1)
        self.subscriber = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.state_listener)
        self.state = np.array([0,0,0,0])
        self.goal_state = np.array([0, 1, 0, 0])
        self.functions = [self.g1, self.strafe, self.turn, self.g2]
        self.function_names = ['x', 'y', 'theta', 'phi']
        # self.Kp = np.array([10., 50., 50., 1.])
        self.Kp = np.array([1., 2., 25., 1.])
        self.states = []
        
        # self.full_state = np.array([0,0,0,0])
        # self.state = np.array([0])
        # self.goal_state = np.array([np.pi / 4])
        # self.functions = [self.turn]
        # self.Kp = np.array([1])

    def state_listener(self, msg):
        # Real turtlebot uses theta in [-pi, pi]. Convert to [0, 2*pi]
        theta = msg.theta
        if theta < 0:
            theta += 2 * np.pi
        self.state = np.array([msg.x, msg.y, theta, msg.phi])
        self.states.append(self.state)
        # self.full_state = np.array([msg.x, msg.y, msg.theta, msg.phi])
        # self.state = np.array([msg.theta])

        # if self.state[2] > np.pi:
        #     self.state[2] -= 2 * np.pi

    def format_path_to_plot(self, path):
        t = [p[0] for p in path]
        commands = np.array([np.array([p[1].linear_velocity, p[1].steering_rate]) for p in path])
        states = np.array([np.array([p[2].x, p[2].y, p[2].theta, p[2].phi]) for p in path])
        return t, commands, states

    def plot_traj(self):
        true_states = np.array(self.states)

        f, axarr = plt.subplots(5, 1)
        for i in range(5):
            if i == 4:
                axarr[i].scatter([true_states[:, 0][0]], [true_states[:, 1][0]], color='k', label='start')
                axarr[i].scatter([true_states[:, 0][-1]], [true_states[:, 1][-1]], color='r', label='end')
                axarr[i].plot(true_states[:, 0], true_states[:, 1], color='g', label='true')
                axarr[i].scatter([self.goal_state[0]], [self.goal_state[1]], color='b', marker='*', label='goal')
            else:
                axarr[i].plot(true_states[:, i], color='b', label='true')
                axarr[i].axhline(self.goal_state[i], linestyle='--', color='g', label='goal')

        ylabels = ['x', 'y', 'theta', 'phi']
        for i, ax in enumerate(axarr.flat):
            if i != 4:
                ax.set(xlabel='time (s)', ylabel=ylabels[i])
            else:
                ax.set(xlabel='x', ylabel='y')

        plt.legend()
        plt.show()



    def run(self):
        err = self.goal_state - self.state
        while not rospy.is_shutdown():
            while not (np.abs(err) < 0.1).all():
                for i in range(len(err)):
                    while not np.abs(err[i]) < 0.1: 
                        print 'Minimizing {0}'.format(self.function_names[i])

                        function = self.functions[i]
                        d = np.sign(err[i])
                        if i == 2 or i == 1:
                            d *= -1
                        mag = self.Kp[i] * np.abs(err[i]) 
                        
                        if i == 0 or i == 1:
                            d *= np.sign(np.cos(self.state[2]))

                        # Decrease theta mag
                        if i == 2:
                            mag = 0.8

                        # if i == 1:
                        #     mag *= 0.001
                        #     mag = 0
                        # else:
                        mag *= 0.25
                        mag = max(0.18, mag)

                        print 'magnitude: {0}'.format(mag)
                        function(mag, d)
                        err = self.goal_state - self.state
                        print 'Minimizing error {0}'.format(i)
                        print 'state: {0}'.format(self.state)
                        print 'goal : {0}'.format(self.goal_state)
                        print 'error: {0}'.format(err)
                        # print 'x: {} y: {} phi: {}'.format(self.full_state[0], self.full_state[1], self.full_state[3])
            self.cmd(0,0)
            print 'Done!'

            self.plot_traj()

            break

    def g1(self, mag, d):
        print 'Running g1'
        self.cmd(d * mag, 0)
        self.rate.sleep()
        self.cmd(0, 0)

    def g2(self, mag, d):
        print 'Running g2'
        self.cmd(0, d * mag)
        self.rate.sleep()
        self.cmd(0, 0)

    def strafe(self, mag, d):
        print 'Running strafe'
        self.turn(mag, d)
        self.rate.sleep()
        self.cmd(2*mag, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.turn(mag, -d)
        self.rate.sleep()
        self.cmd(-2*mag, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.cmd(0, 0)
        

    def turn(self, mag, d):
        print 'Running turn'
        self.cmd(mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, d*mag)
        self.rate.sleep()
        self.cmd(-mag, -d*mag)
        self.rate.sleep()
        self.cmd(mag, -d*mag)
        self.rate.sleep()
        self.cmd(0, 0)

    def cmd(self, u1, u2):
        self.pub.publish(BicycleCommandMsg(u1, u2))

if __name__ == '__main__':
    rospy.init_node('bangbangcontroller', anonymous=False)

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()

    b = BangBang()
    b.run()