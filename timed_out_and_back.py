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
        
        # Used to compare with the sign of distance to determine direction
	self.preDir = 1.0
        self.preAng = 1.0

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
        
        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.2
        
        # Set the travel distance to 1.0 meters
        # goal_distance = 1.0
        
        # How long should it take us to get there?
        # linear_duration = goal_distance / linear_speed
        
        # Set the rotation speed to 1.0 radians per second
        self.angular_speed = 1.0
        
        # Set the rotation angle to Pi radians (180 degrees)
        # goal_angle = pi
        
        # How long should it take to rotate?
        # angular_duration = goal_angle / angular_speed
     
        # Loop through the two legs of the trip  
        '''for i in range(2):
            # Initialize the movement command
            move_cmd = Twist()
            
            # Set the forward speed
            move_cmd.linear.x = linear_speed
            
            # Move forward for a time to go the desired distance
            ticks = int(linear_duration * rate)
            
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            
            # Stop the robot before the rotation
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            
            # Now rotate left roughly 180 degrees  
            
            # Set the angular speed
            move_cmd.angular.z = angular_speed

            # Rotate for a time to go 180 degrees
            ticks = int(goal_angle * rate)
            
            for t in range(ticks):           
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                
            # Stop the robot before the next leg
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)    
            
        # Stop the robot
        self.cmd_vel.publish(Twist())'''
    
    def Translations(self, dis):
	print "Will perform a translation for " + dis
        
	if float(dis) * self.preDir >= 0:
		print "Positive translations!"
        	linear_duration = float(dis) / self.linear_speed
        	move_cmd = Twist()
        	move_cmd.linear.x = self.linear_speed
        	ticks = int( linear_duration * self.rate)
        	for t in range(ticks):
        		self.cmd_vel.publish(move_cmd)
                	self.r.sleep()
        	move_cmd = Twist()
        	self.cmd_vel.publish(move_cmd)
        	rospy.sleep(1)
	else:
		print "Negative Distance!"
		move_cmd = Twist()
		angular_duration = pi / self.angular_speed
		
                move_cmd.angular.z = self.angular_speed
                ticks = int(angular_duration * self.rate)
                for t in range(ticks):           
                	self.cmd_vel.publish(move_cmd)
              		self.r.sleep()
        	
		move_cmd = Twist()
        	self.cmd_vel.publish(move_cmd)
        	rospy.sleep(1)

        	linear_duration = abs(float(dis)) / self.linear_speed
        	move_cmd = Twist()
        	move_cmd.linear.x = self.linear_speed
        	ticks = int( linear_duration * self.rate)
        	for t in range(ticks):
        		self.cmd_vel.publish(move_cmd)
                	self.r.sleep()
        	move_cmd = Twist()
        	self.cmd_vel.publish(move_cmd)
        	rospy.sleep(1)
			
    
    def Rotations(self, ang):
	print "Will perform rotation for: " + ang + " degree"
	rad = math.radians(float(ang))
	
	if rad >= 0:
		print "Turn counterclockwise for radians " + str(abs(rad))
		
		move_cmd = Twist()
                move_cmd.angular.z = self.angular_speed
		angular_duration = abs(rad) / self.angular_speed
                
		ticks = int(angular_duration * self.rate)
                for t in range(ticks):           
                	self.cmd_vel.publish(move_cmd)
              		self.r.sleep()
        	
		move_cmd = Twist()
        	self.cmd_vel.publish(move_cmd)
        	rospy.sleep(1)
	else:
		print "Turn clockwise for radians for radians " + str(abs(rad))
		
		move_cmd = Twist()
                move_cmd.angular.z = - self.angular_speed
		angular_duration = abs(rad) / self.angular_speed

                ticks = int(angular_duration * self.rate)
                for t in range(ticks):           
                	self.cmd_vel.publish(move_cmd)
              		self.r.sleep()
        	
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
    

    user_input = raw_input("Enter a T for translation, R for rotation, or Q for quit: ")
    
    if user_input == 'T':
	print "Translations!"
	dis = raw_input("Enter a distance for the translations: ")
        if is_number(dis):
		print "Is a number!"
                robot = OutAndBack()
		robot.Translations(dis)
        else:
		print "Not a number!"
    
    elif user_input == 'R':
	print "Rotations!"
	ang = raw_input("Enter a degree for the rotation: ")
        if is_number(ang):
		print "Is a number!"
                robot = OutAndBack()
		robot.Rotations(ang)
        else:
		print "Not a number!"
	
    elif user_input == 'Q':
	print "Quit!"
    else:
	print "Unknown"
    
    '''try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")'''

