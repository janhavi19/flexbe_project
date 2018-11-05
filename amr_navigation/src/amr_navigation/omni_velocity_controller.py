#!/usr/bin/env python

PACKAGE = 'amr_navigation'

from math import *
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance

class OmniVelocityController(VelocityController):

    def __init__(self, l_max_vel, l_tolerance, l_acc, a_max_vel, a_tolerance, a_acc):

        # Assigning the max values and tolerance values to class variables
        self._l_max_vel = l_max_vel
        self._l_tolerance = l_tolerance
        self._a_max_vel = a_max_vel
        self._a_tolerance = a_tolerance
        self._l_acc = l_acc
        self._a_acc = a_acc

        # Calculating the deceleration distance
        self._l_deceleration_dist = (l_max_vel **2)/(2*l_acc)
        self._a_deceleration_dist = (a_max_vel **2)/(2*a_acc)

    def compute_velocity(self, actual_pose):

        dx = self._target_pose.x - actual_pose.x 
        dy = self._target_pose.y - actual_pose.y

        # Getting the shortest euclidean distance between the destination and the robot
        linear_dist = get_distance(self._target_pose, actual_pose)

        # Getting the angular distance between the destination and the robot
        angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)
        change_taken_per_unit= (angular_dist)/linear_dist

        # If Target pose is reached
        if (abs(linear_dist)<self._l_tolerance and abs(angular_dist)<self._a_tolerance):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        # If Target is not reached
        if (abs(linear_dist)>self._l_tolerance or abs(angular_dist)>self._a_tolerance):


        # checking if the vehicle is within deceleration range
            if (abs(linear_dist) < self._l_deceleration_dist or abs(angular_dist) < self._a_deceleration_dist):
                
                # Calculating velocity for deceleration
                l_vel = sqrt(2 * self._l_acc* linear_dist)
            else:
                # Else move with max velocity
                l_vel = self._l_max_vel

            angular_dist = get_shortest_angle(atan2(dy, dx), actual_pose.theta)

            vel_x = cos(angular_dist)*l_vel # velocity component in x direction
            vel_y = sin(angular_dist)*l_vel # velocity component in y direction

            vel_z = self._a_max_vel*change_taken_per_unit * l_vel # velocity component in z direction

            if vel_z > self._a_max_vel:
                vel_z = self._a_max_vel


            return Velocity(vel_x, vel_y, vel_z)

        else:
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()


    """
    ========================= YOUR CODE HERE =========================

    Instructions: put here all the functions that are necessary to
    implement the VelocityController interface. You may
    use the DiffVelocityController as an example.

    Implement the constructor to accept all the necessry parameters
    and implement compute_velocity() method

    You are free to write any helper functions or classes you might
    need.

    ==================================================================

    """