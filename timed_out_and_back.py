#!/usr/bin/env python

""" timed_out_and_back.py - Version 1.2 2014-12-14
    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist
from math import pi
import math

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # How fast will we update the robot's movement?
        self.rate = 50

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)

        # Set the forward linear speed to 0.2 meters per second
        self.linear_speed = 0.2

        # Set the rotation speed to 1.0 radians per second
        self.angular_speed = 1.0

        nan_msg = "Not a number!"
        ukn_msg = "Unknown operation, please enter T, R or Q!"
        while True:
            user_input = raw_input("Enter T to translate, R to rotate, or Q to quit: ")

            if user_input == 'T':
                dist_str = raw_input("Enter a distance (in meters, negative numbers for reversing): ")
                if is_number(dist_str):
                    self.translations(dist_str)
                else:
                    print nan_msg

            elif user_input == 'R':
                ang_str = raw_input("Enter a angle (in degrees, positive being counterclockwise): ")
                if is_number(ang_str):
                    self.rotations(ang_str)
                else:
                    print nan_msg

            elif user_input == 'Q':
                break

            else:
		try:
                   print ukn_msg
		except KeyboardInterrupt:
		   print "Interrupted!"


    def translations(self, dist_str):
        distance = float(dist_str)
        self.linear_speed = abs(self.linear_speed) * (distance / abs(distance))
        linear_duration = abs(distance / self.linear_speed)

        # Start translation
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        ticks = int(linear_duration * self.rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop robot
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def rotations(self, ang_str):
        ang_rad = math.radians(float(ang_str))
        self.angular_speed = abs(self.angular_speed) * (ang_rad / abs(ang_rad))
        angular_duration = abs(ang_rad / self.angular_speed)

        # Start rotation
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed
        ticks = int(angular_duration * self.rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop robot
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
