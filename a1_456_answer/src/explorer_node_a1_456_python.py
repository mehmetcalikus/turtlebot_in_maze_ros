#!/usr/bin/env python
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 

## Common ROS headers.
import rospy
## Required for some printing options
import sys

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##
from tf import transformations
import math
from math import isnan
#mehmet calikus
#150150042

pre_region = {
	'fright' : 10, 
	'front'  : 10,
	'fleft'  : 10,
}
regions_ = {
	
	'fright' : 0, 
	'front'  : 0,
	'fleft'  : 0,
}

state_ = 0
state_dict_ = {
	0: 'find the wall',
	1: 'turn left',
	2: 'follow the wall'
}

def change_state(state):
	global state_, state_dict_
	if state is not state_:
		print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
		state_ = state



def take_action():
	global pre_region
	global regions_
	regions = regions_
	msg = Twist()
	linear_x = 0
	angular_z = 0
	
	state_description = ''
	
	d = 1
	

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		state_description = 'case 1 - nothing'
		change_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		state_description = 'case 2 - front'
		change_state(1)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 3 - fright'
		change_state(2)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		state_description = 'case 4 - fleft'
		change_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 5 - front and fright'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		state_description = 'case 6 - front and fleft'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		state_description = 'case 7 - front and fleft and fright'
		change_state(1)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		state_description = 'case 8 - fleft and fright'
		change_state(0)
	else:
		if pre_region['fright'] < 1.6 : 
			change_state(1)

		else:	
			change_state(0)
		state_description = 'unknown case'
		rospy.loginfo(regions)

	pre_region = regions#when all reagon variables such as front fright and flest is nan beacuse of the <0.45
						#robot keeps its previous state and hits the wall 	
 						#pre_region holds last value before becoming nan if all region variable are nan we are controling
 						#pre_region value and we take better result.
 						#moreover in this way we do not need to use isnan library because of nan values..


def find_wall():
	msg = Twist()
	msg.linear.x = 0.4
	msg.angular.z = -0.1
	print ('find_wall')
	return msg

def turn_left():
	msg = Twist()
	msg.angular.z = 0.3
	print ('turn_left')
	return msg

 

def follow_the_wall():	
	msg = Twist()
	msg.linear.x = 0.5
	print ('follow_the_wall')
	return msg



def laser_callback(data):
	

	## Lets publish that command so that the robot follows it
	global motor_command_publisher
	
	global regions_

	regions_ = {
	'fright': min(min(data.ranges[0:213]), 10),
	'front': min(min(data.ranges[214:427]), 10) ,
	'fleft': min(min(data.ranges[428:640]), 10) 
	}

	take_action()	
	global state_
	if state_ == 0:
		temp = find_wall()
	elif state_ == 1:
		temp = turn_left()
	elif state_ == 2:
		temp = follow_the_wall()
		pass
	else:
		rospy.logerr('Unknown state!')
	motor_command_publisher.publish(temp)
	

	## Alternatively we could have looked at the laser scan BEFORE we made this decision
	## Well Lets see how we might use a laser scan
	## Laser scan is an array of distances
	print ('Number of points in laser scan is: ', len(data.ranges))
	print ('The distance to the rightmost scanned point is: ', data.ranges[0])
	print ('The distance to the leftmost scanned point is: ', data.ranges[-1])
	print ('The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2])
	## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
	print ('The minimum angle scanned by the laser is: ', data.angle_min)
	print ('The maximum angle scanned by the laser is: ', data.angle_max)
	print ('The increment in the angles scanned by the laser is: ', data.angle_increment)
	## angle_max = angle_min+angle_increment*len(data.ranges)
	print ('The minimum range (distance) the laser can perceive is: ', data.range_min)
	print ('The maximum range (distance) the laser can perceive is: ', data.range_max)
	
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
	chatty_map = False
	if chatty_map:
		print "-------MAP---------"
		## Here x and y has been incremented with five to make it fit in the terminal
		## Note that we have lost some map information by shrinking the data
		for x in range(0,data.info.width-1,5):
			for y in range(0,data.info.height-1,5):
				index = x+y*data.info.width
				if data.data[index] > 50:
					## This square is occupied
					sys.stdout.write('X')
				elif data.data[index] >= 0:
					## This square is unoccupied
					sys.stdout.write(' ')
				else:
					sys.stdout.write('?')
			sys.stdout.write('\n')
		sys.stdout.flush()
		print "-------------------"
	
## This is the method we initilize everything
def explorer_node():
	## We must always do this when starting a ROS node - and it should be the first thing to happen
	rospy.init_node('amble')
	
	## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
	global motor_command_publisher
	motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
	
	## Here we set the function laser_callback to recieve new laser messages when they arrive
	rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
	
	## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
	rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
	
	## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
	rospy.spin()
	
if __name__ == '__main__':
	explorer_node()
