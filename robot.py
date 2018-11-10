#!/usr/bin/env python

# Simple line following robot simulation
# Created by Melih Erdogan for Intelligent Robotics 1 Assingment 3 
# 11/09/2018

import rospy
from random import randint
from std_msgs.msg import Int32
from std_msgs.msg import String
import time

#variables used to for robot
global robot_map
global start
global match
global counter

global sensor1
global sensor2
global sensor3
global bumper
global location

#initilize the variables
location = 0
counter = 0
match = False
start = False
robot_map = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
sensor1 = 0
sensor2 = 1
sensor3 = 0
bumper = 0

# map values
# 0=  black line is in the middle
# 1=  black line is in the middle and left
# 2=  black line is in the left
# 3=  black line is in the middle and right
# 4=  black line is in the right
# 5=  there is a wall

#create publishers
pubSensorL = rospy.Publisher('sensorL', Int32, queue_size=10)
pubSensorR = rospy.Publisher('sensorR', Int32, queue_size=10)
pubSensorM = rospy.Publisher('sensorM', Int32, queue_size=10)
pubSensorB = rospy.Publisher('bumper', Int32, queue_size=10)
pubLocation = rospy.Publisher('location', Int32, queue_size=10)

def callback(data):

# initialize used variables
	global counter
	global match
	global start
	global sensor1
	global sensor2
	global sensor3
	global bumper
	global location
	motion_command = data.data
	next_map = robot_map[counter]

# print counter for debugging
#	print (counter)

	if start == True:
		# compare motion command and map data
		# give user feedback
		if motion_command == "Go_Forward" and next_map == 0:
			match = True
		elif motion_command == "Turn_Left" and next_map == 1:
			match = True
		elif motion_command == "Turn_Left" and next_map == 2:
			match = True
		elif motion_command == "Turn_Right" and next_map == 3:
			match = True
		elif motion_command == "Turn_Right" and next_map == 4:
			match = True
		elif motion_command == "Stop" and next_map == 5:
			match = True
		else:
			match = False
			if next_map == 5:
				print ("-Ouch! I hit the wall. You should have told me to stop")
			elif motion_command == "Stop":
				print("-I can't stop") 			
			else:
				print ("-Ups.. I lost the black line")
				print ("-I found the line again. Let's keep moving.")

	# move the robot and give user feedback
	if start == False:
		if motion_command == "Go_Forward":
			print ("YES! I am alive and moving forward")
			counter = counter + 1
			start =  True
		else:
			print ("-Ohh.. You forgot to tell me to go forward first!")
	else:
		if motion_command == "Go_Forward" and match == True:
			counter = counter + 1
			print ("-I am moving forward")
		elif motion_command == "Turn_Left" and match == True:
			counter = counter + 1
			print ("-I am turning left")
		elif motion_command == "Turn_Right" and match == True:
			counter = counter + 1
			print ("-I am turning right")
		elif motion_command == "Stop" and match == True:
			print ("-I stopped")
			print ("-YAY! I completed the map")
		elif motion_command == "Stop" and match == False:
			print ("-Come on! The map is not complete yet!")
	
	# publish sensor data to the topics
	# sensor1 = Left Sensor
	# sensor2 = Middle Sensor
	# sensor3 = Right Sensor
	# bumper = Bumper Sensor
	time.sleep(5)
	location = counter
	# update sensor values depending on next map value
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
	print ("-I am ready for the next command")

# initialize the robot node
def robot():
	rospy.init_node("robot", anonymous=True)
	rate = rospy.Rate(2) # 10hz
	
	rospy.Subscriber("motion_command", String, callback)

	while not rospy.is_shutdown():	
		pubSensorL.publish(sensor1)
		pubSensorR.publish(sensor3)
		pubSensorM.publish(sensor2)
		pubSensorB.publish(bumper)
		pubLocation.publish(location)
	rate.sleep()

if __name__ == '__main__':
# create a map and add starting and ending points
	print("Black Line Following Robot Simulator V1.0")
	for i in range(0,15):
		robot_map [i] = randint(0,4)
	robot_map [0] = 0	
	robot_map [14] = 5
# publish initial sensor values
#	print (robot_map)
	try:
		robot()
	except rospy.ROSInterruptException():
		pass
