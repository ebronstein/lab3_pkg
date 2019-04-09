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

        self.alternate_sinusoid_planner = AlternateModelSinusoidPlanner(l, max_phi, max_u1, max_u2)

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
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

        # copy goal state
        goal_state = BicycleStateMsg(goal_state.x, goal_state.y, goal_state.theta, goal_state.phi)

        # This bit hasn't been exhaustively tested, so you might hit a singularity anyways
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))

        if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
            # raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")
            print 'SINGULARITY DETECTED'

            theta_dir = np.sign(goal_state.theta - start_state.theta) # which direction theta is changing

            # Use alternate model if not near 0 or pi
            if not(abs(start_state.theta) < 0.1 or abs(start_state.theta - np.pi) < 0.1):
                print 'USING ALTERNATE MODEL.'
                # Move goal theta so it's not near a singularity
                if abs(goal_state.theta) < 0.1 or abs(goal_state.theta - np.pi) < 0.1:
                    old_goal_theta = goal_state.theta
                    goal_state.theta -= theta_dir * 0.2
                    print 'ADJUSTED GOAL THETA FROM {0} TO {1} SO IT IS NOT AT SINGULARITY.'.format(old_goal_theta, goal_state.theta)
                return self.alternate_sinusoid_planner.plan_to_pose(start_state, goal_state, dt=dt, delta_t=delta_t)
            else:
                # Move away from the alternate model's singularity using this model
                goal_state.theta = start_state.theta + theta_dir * 0.2
                print 'USING REGULAR MODEL TO MOVE TO THETA = {0}'.format(goal_state.theta)

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
        v2 = delta_phi / delta_t

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

        # a2 = 0.3 * min(1, self.phi_dist*omega)
        a2 = 1.

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
        
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        y_start = start_state_v[3]
        y_goal = goal_state_v[3]
        delta_y = y_goal - y_start
        omega = 2*np.pi / delta_t

        # a2 for turtlesim
        a2 = 2 * np.sign(np.cos(start_state.theta))# * np.sign(delta_y)
        # a2 for turtlebot
        # a2 = 2 * np.sign(delta_y)# * np.sign(np.cos(start_state.theta))
        print 'A2: {0}'.format(a2)

        f = lambda phi: (1. / self.l) * np.tan(phi) # This is from the car model
        g = lambda alpha: alpha / np.sqrt(1. - alpha**2)

        def alpha_fn(a1, a2, t):
            alpha_part = lambda tau: f((a2 / (2.*omega)) * np.sin(2*omega*tau))
            alpha_integrand = lambda tau: alpha_part(tau) * a1 * np.sin(omega*tau) 
            return quad(alpha_integrand, 0, t)[0] + start_state_v[2]

        def beta1_fn(a1, a2):
            def beta1_integrand(t): 
                alpha_val = alpha_fn(a1, a2, t)
                # if np.abs(alpha_val) < 1e-6:
                # print '\tALPHA INTEGRAL WAS {0}'.format(alpha_val)
                return g(alpha_val) * np.sin(omega * t)
            beta1 = (omega / np.pi) * quad(beta1_integrand, 0, delta_t)[0]
            # if np.abs(beta1) < 1e-6:
            # print 'BETA INTEGRAL WAS {0}\n'.format(beta1)
            return beta1

        def y_fn(a1, a2):
            beta1 = beta1_fn(a1, a2)
            curr_goal_y = np.pi * a1 * beta1 / omega + y_start
            return curr_goal_y
            # goal_y_list.append(curr_goal_y)
            # if np.abs(curr_goal_y - goal_state_v[3]) < 0.01:
            #     break

        a1_max = self.find_max_a1(y_fn, a2)
        a1_min = 0
        print 'MAX A1: {0}'.format(a1_max)
        
        if np.sign(np.cos(start_state.theta)) < 0:
            a1, y_opt = self.linear_search(a1_min, a1_max, a2, y_goal, y_fn)
            # import pdb; pdb.set_trace()
        else:
            a1, y_opt = self.binary_search(a1_min, a1_max, a2, y_goal, y_fn, 1e-8)
              
        v1 = lambda t: a1*np.sin(omega*t)
        v2 = lambda t: a2*np.cos(2*omega*t)

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt

        return self.v_path_to_u_path(path, start_state, dt)

    def find_max_a1(self, y_fn, a2):
        a1 = 0.01
        y = y_fn(a1, a2)
        while np.isfinite(y):
            a1 *= 2
            y = y_fn(a1, a2)
        return a1

    def linear_search(self, a1_min, a1_max, a2, y_goal, y_fn):
        a1_vals = np.linspace(a1_min, a1_max, 100)
        y_vals = np.array([y_fn(a1, a2) for a1 in a1_vals])
        y_vals[np.isnan(y_vals)] = float('inf')
        opt_idx = np.argmin(np.abs(y_vals - y_goal))
        a1_opt = a1_vals[opt_idx]
        y_opt = y_vals[opt_idx]
        import pdb; pdb.set_trace()
        return a1_opt, y_opt

    def binary_search(self, a1_min, a1_max, a2, y_goal, y_fn, eps):
        a1_curr = (a1_min + a1_max) / 2.
        y_curr = y_fn(a1_curr, a2)
        print 'current a1: {0}'.format(a1_curr)
        print 'current y:  {0}'.format(y_curr)
        print 'goal y:     {0}'.format(y_goal)

        def while_condition(y_curr):
            if not np.isfinite(y_curr):
                return True
            else:
                return np.abs(y_curr - y_goal) > eps

        while while_condition(y_curr):
            # if np.isfinite(y_curr):
            # if np.isfinite(y_curr) and (y_curr < y_goal):
            #     a1_min = a1_curr
            # else:
            #     a1_max = a1_curr

            if np.isfinite(y_curr):
                if np.sign(a1_curr) == 1:
                    if y_curr < y_goal:
                        a1_min = a1_curr
                    else:
                        a1_max = a1_curr
                else:
                    if y_curr < y_goal:
                        a1_min = a1_curr
                    else:
                        a1_max = a1_curr
            else:  
                if np.sign(a1_curr) == 1:
                    a1_max = a1_curr
                else:
                    a1_min = a1_curr

            a1_curr = (a1_min + a1_max) / 2.
            y_curr = y_fn(a1_curr, a2)
            print 'current a1: {0}'.format(a1_curr)
            print 'current y:  {0}'.format(y_curr)
            print 'goal y:     {0}'.format(y_goal)
            # if a1_curr == a1_min or a1_curr == a1_max:
            #     break

        return a1_curr, y_curr

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


class AlternateModelSinusoidPlanner():
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

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
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

        y_path =        self.steer_y(
                            start_state, 
                            goal_state, 
                            0, 
                            dt, 
                            delta_t
                        )
        phi_path =      self.steer_phi(
                            y_path[-1][2], 
                            goal_state, 
                            y_path[-1][0] + dt, 
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
        x_path =        self.steer_x(
                            alpha_path[-1][2], 
                            goal_state, 
                            alpha_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )     

        path = []
        for p in [x_path, phi_path, alpha_path, x_path]:
            path.extend(p)
        return path

    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
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
        delta_y = goal_state_v[0] - start_state_v[0]

        v1 = delta_y/delta_t
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
        v2 = delta_phi / delta_t

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

        # a2 = 0.3 * min(1, self.phi_dist*omega)
        a2 = 1.

        f = lambda phi: (-1./self.l)*np.tan(phi) # This is from the car model
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


    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
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
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        x_start = start_state_v[3]
        x_goal = goal_state_v[3]
        delta_x = x_goal - x_start
        omega = 2*np.pi / delta_t

        # a2 = 2 * np.sign(delta_x)
        f = lambda phi: (-1. / self.l) * np.tan(phi) # This is from the car model
        g = lambda alpha: alpha / np.sqrt(1. - alpha**2)

        def alpha_fn(a1, a2, t):
            alpha_part = lambda tau: f((a2 / (2.*omega)) * np.sin(2*omega*tau))
            alpha_integrand = lambda tau: alpha_part(tau) * a1 * np.sin(omega*tau) 
            return quad(alpha_integrand, 0, t)[0] + start_state_v[2]

        def beta1_fn(a1, a2):
            def beta1_integrand(t): 
                alpha_val = alpha_fn(a1, a2, t)
                # if np.abs(alpha_val) < 1e-6:
                # print '\tALPHA INTEGRAL WAS {0}'.format(alpha_val)
                return g(alpha_val) * np.sin(omega * t)
            beta1 = (omega / np.pi) * quad(beta1_integrand, 0, delta_t)[0]
            # if np.abs(beta1) < 1e-6:
            # print 'BETA INTEGRAL WAS {0}\n'.format(beta1)
            return beta1

        def x_fn(a1, a2):
            if abs(a1 * a2) > 0.05:
                return float('inf')
            beta1 = beta1_fn(a1, a2)
            return np.pi * a1 * beta1 / omega + x_start

        # a1_max = self.find_max_a1(x_fn)

        # a1, x_opt = self.binary_search(a1_min, a1_max, x_goal, x_fn, 1e-8)
        # a1, x_opt = self.linear_search(a1_min, a1_max, a2, x_goal, x_fn)
        a1, a2, x_opt = self.grid_search(x_goal, x_fn)
        
        v1 = lambda t: a1*np.sin(omega*t)
        v2 = lambda t: a2*np.cos(2*omega*t)

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt

        return self.v_path_to_u_path(path, start_state, dt)

    def find_max_a1(self, x_fn):
        a1 = 1e-6
        y = x_fn(a1)
        while np.isfinite(y):
            a1 *= 2
            y = x_fn(a1)
        return a1

    def grid_search(self, x_goal, x_fn):
        a1_max = 1
        a1_min = -a1_max
        a2_max = 1
        a2_min = -a2_max

        a1_vals = np.linspace(a1_min, a1_max, 50)
        a2_vals = np.linspace(a2_min, a2_max, 50)

        a1_mesh, a2_mesh = np.meshgrid(a1_vals, a2_vals)

        x_fn_vec = np.vectorize(x_fn)
        x_vals = x_fn_vec(a1_mesh, a2_mesh)

        # import pdb; pdb.set_trace()

        x_vals[np.isnan(x_vals)] = float('inf') # replace NaNs with infinity
        opt_idx_flat = np.argmin(np.abs(x_vals - x_goal))
        opt_a1_idx, opt_a2_idx = np.unravel_index(opt_idx_flat, x_vals.shape)
        opt_a1 = a1_vals[opt_a1_idx]
        opt_a2 = a2_vals[opt_a2_idx]
        opt_x = x_vals[opt_a1_idx, opt_a2_idx]
        return opt_a1, opt_a2, opt_x

    def linear_search(self, a1_min, a1_max, a2, x_goal, x_fn):
        a1_vals = np.linspace(a1_min, a1_max, 100)
        x_vals = np.array([x_fn(a1, a2) for a1 in a1_vals])
        opt_idx = np.argmin(np.abs(x_vals - x_goal))
        a1_opt = a1_vals[opt_idx]
        x_opt = x_vals[opt_idx]
        import pdb; pdb.set_trace()
        return a1_opt, x_opt

    def binary_search(self, a1_min, a1_max, x_goal, x_fn, eps):
        a1_curr = (a1_min + a1_max) / 2.
        y_curr = x_fn(a1_curr)

        def while_condition(y_curr):
            if not np.isfinite(y_curr):
                return True
            else:
                return np.abs(y_curr - x_goal) > eps

        while while_condition(y_curr):
            # if np.isfinite(y_curr):
            # if np.isfinite(y_curr) and (y_curr < x_goal):
            #     a1_min = a1_curr
            # else:
            #     a1_max = a1_curr

            if np.isfinite(y_curr):
                if np.sign(a1_curr) == 1:
                    if y_curr < x_goal:
                        a1_min = a1_curr
                    else:
                        a1_max = a1_curr
                else:
                    if y_curr < x_goal:
                        a1_min = a1_curr
                    else:
                        a1_max = a1_curr
            else:  
                if np.sign(a1_curr) == 1:
                    a1_max = a1_curr
                else:
                    a1_min = a1_curr

            a1_curr = (a1_min + a1_max) / 2.
            y_curr = x_fn(a1_curr)
            print 'current a1: {0}'.format(a1_curr)
            print 'current y:  {0}'.format(y_curr)
            print 'goal y:     {0}'.format(x_goal)
            # if a1_curr == a1_min or a1_curr == a1_max:
            #     break

        return a1_curr, y_curr

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (y,phi,alpha,x)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray` 
        """
        return np.array([state.y, state.phi, np.cos(state.theta), state.x])

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
            u1 = v1/np.sin(state.theta)
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