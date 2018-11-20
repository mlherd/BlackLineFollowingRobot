#!/usr/bin/env python

# Simple Line Following Robot Simulator
# V1.0
# Created by Melih Erdogan for Intelligent Robotics 1 Assingment 3 
# Last Update - 11/13/2018
#
#
# Subscribed Topics
# /motion_command - String
#
# Published Topics
# /sebsorL - Int
# /sensorM - Int
# /sensorR - Int
# /bumper - Int
# /location - Int

# Import Libraries 

import rospy
from random import randint
from std_msgs.msg import Int32
from std_msgs.msg import String
import time
import sys

#flags used to for robot
global robot_map
global start
global counter
global busy

#sensors
global sensor1
global sensor2
global sensor3
global bumper
global location

#other gloabal variables
global motion_commandList

#initilize the variables
location = 0
counter = 0
start = False
sensor1 = 0
sensor2 = 1
sensor3 = 0
bumper = 0
busy = False
motion_commandList = ["Go_Forward","Turn_Left","Turn_Right","Stop"]
robot_map = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

# The definition of map values
# 0=  black line is in the middle
# 1=  black line is in the middle and left
# 2=  black line is in the left
# 3=  black line is in the middle and right
# 4=  black line is in the right
# 5=  there is a wall

# update sensor data
# sensor1 = Left Sensor
# sensor2 = Middle Sensor
# sensor3 = Right Sensor
# bumper = Bumper Sensor
#create publishers
pubSensorL = rospy.Publisher('/sensorL', Int32, queue_size=10)
pubSensorR = rospy.Publisher('/sensorR', Int32, queue_size=10)
pubSensorM = rospy.Publisher('/sensorM', Int32, queue_size=10)
pubSensorB = rospy.Publisher('/bumper', Int32, queue_size=10)
pubLocation = rospy.Publisher('/location', Int32, queue_size=10)

#end method for the node
#infinite loop
def end():
	a = True
	while(a == True):
		a = True

#callback method for the motion command subscriber
def callback(data):
	if busy == False:
		robo_brain(data.data)
	else:
		print ("-I am still moving. I can do only one motion at a time.")

#update all sensor values
def sensor_update():
	global sensor1
	global sensor2
	global sensor3
	global bumper
	
	if robot_map[counter] == 0:
		sensor1 = 0
		sensor2 = 1
		sensor3 = 0
		bumper = 0
	elif robot_map[counter] == 1:
		sensor1 = 1
		sensor2 = 1
		sensor3 = 0
		bumper = 0
	elif robot_map[counter] == 2:
		sensor1 = 1
		sensor2 = 0
		sensor3 = 0
		bumper = 0
	elif robot_map[counter] == 3:
		sensor1 = 0
		sensor2 = 1
		sensor3 = 1
		bumper = 0
	elif robot_map[counter] == 4:
		sensor1 = 0
		sensor2 = 0
		sensor3 = 1
		bumper = 0
	elif robot_map[counter] == 5:
		sensor1 = 1
		sensor2 = 1
		sensor3 = 1
		bumper = 1
	return True

#robot brain method
def robo_brain(command):
	#define the used global variables
	global counter
	global match
	global start
	global sensor1
	global sensor2
	global sensor3
	global bumper
	global location
	global busy
	
	#print counter
	#update motion command and get new map location value
	motion_command = command
	next_map = robot_map[counter]	
	
	#print counter for debugging
	#print (counter)

	busy = True
	if motion_command in motion_commandList:	#check if the command is in the list of motion commands
		if start == True:
			# compare motion command and map data
			# move the robot by increasing counter
			# give user feedback
			if motion_command == "Go_Forward" and next_map == 0:
				counter = counter + 1
				print ("-I am moving forward")
			elif motion_command == "Turn_Left" and next_map == 1:
				counter = counter + 1
				print ("-I am turning left")
			elif motion_command == "Turn_Left" and next_map == 2:
				counter = counter + 1
				print ("-I am turning left")
			elif motion_command == "Turn_Right" and next_map == 3:
				counter = counter + 1
				print ("-I am turning right")
			elif motion_command == "Turn_Right" and next_map == 4:
				counter = counter + 1
				print ("-I am turning right")
			elif motion_command == "Stop" and next_map == 5:
				print ("-I stopped")
				print ("-YAY! I completed the map")
				end()
			else:
				if next_map == 5:
					print ("-Ouch! I hit the wall. You should have told me to stop")
				elif motion_command == "Stop":
					print("-I can't stop")
					print ("-Come on! The map is not complete yet!") 			
				else:
					print ("-Oops.. I lost the black line")
					print ("-I found the line again. Let's keep moving.")

			# update sensor values depending on next map value. Not needed here, keep here testing
			sensor_update()
			print ("-I am ready for the next command")
	
			#print (location)
		# check if the robot is at the start location and alive
		elif start == False:
			if motion_command == "Go_Forward":
				print ("-YES! I am alive and moving forward. Let's complete this map")
				counter = counter + 1			
				start =  True
			else:
				print ("-Ohh.. You forgot to tell me to go forward first!")
		# comsider robot needs 3 secs to complete any motion
		# update location value		
		time.sleep(3)
		sensor_update()		
		location = counter
		#print counter
	else:
		print ("-I don't understand what you say! You send me a wrong command type")
	busy = False # robot brain is not busy anymore
	#print busy flag for debugging	
	#print (busy)

# initialize the robot node
def robot():
	#initilize the node
	rospy.init_node("robot", anonymous=True)
	#set the publishing rate
	rate = rospy.Rate(10) # 10Hz
	
	#create a subscriber
	rospy.Subscriber("motion_command", String, callback)

	#publish sensor values
	while not rospy.is_shutdown():	
		pubSensorL.publish(sensor1)
		pubSensorR.publish(sensor3)
		pubSensorM.publish(sensor2)
		pubSensorB.publish(bumper)
		pubLocation.publish(location)
	rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	print("Black Line Following Robot Simulator V1.0")
	# create a map and add starting and ending points
	for i in range(0,15):
		robot_map [i] = randint(0,4)
	robot_map [0] = 0	
	robot_map [14] = 5
	# print the map for debugging
	# print (robot_map)
	try:
		robot()
	except rospy.ROSInterruptException():
		pass
