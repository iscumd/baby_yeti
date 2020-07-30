#!/usr/bin/env python

"""
Need to do conversion so we can compare encoder feedback with setpoint made by controller
Need to set up proper update() function so that it suits the robot
Need to determine if the sabertooth code needs to be implemented
Need to determine how many times PID must be called for one Setpoint (how many function calls for a left turn?)
Need to determine if function is too intensive for Jetson (Nate fan disapproves)
Need to remove global variables
"""

import time
import math
import rospy #for talker/ listener
import serial #to send to Sabertooth
from geometry_msgs.msg import Twist #for talker/ listener
from baby.msg import channel_msgs #custom message for left and right encoder rpms
#import matplotlib.pyplot as plt
from simple_pid import PID

joyLeft, joyRight = 0, 0 #values from joystick [-180, 180] integer
leftVel, rightVel = 0, 0 #values from encoders (-180, 180) integer we need to update this by limiting the inputs to [-180, 180] to avo
fowards = True


class Motor:
    #we need to take in controller input for setpoint
    #we need to take in encoder ticks, find linear speed (current state)
    #essentially make motor go brr in a smooth manner (purr)
	def __init__(self):
		self.lspeed = 0
		self.rspeed = 0

	def lupdate(self, effort, dt): # effort is result of pid function
		self.lspeed = leftVel
		# change speed in attempt to match setpoint
		self.lspeed += 1 * effort * dt
		leftInt = ((0.35*self.lspeed) +64)
		if leftInt == 0:
			leftInt = 1
		serial_port.write(chr(leftInt))
		return self.speed

	def rupdate(self, effort, dt): # effort is result of pid function
		self.rspeed = leftVel
		# change speed in attempt to match setpoint
		self.rspeed += 1 * effort * dt
		rightInt = ((0.35*rightVel) +64) +128 # it can go faster going left
		serial_port.write(chr(rightInt))
		return self.rspeed

def controlInputCallback(msg):
	x = msg.linear.x      # -1 < x < 1
	if x >= 0:
		fowards = True
	else:
		fowards = False
	z = msg.angular.z * 2 # -1 < z < 1
	joyLeft, joyRight = twistToWheelSpeed(x, z) #these int values will be setpoint for the PI Controller

	#we can use this to implement publisher
	#joyLeft, joyRight = sendWheelSpeedToMotors(left, right) #these int values will be setpoint for the PI Controller

def twistToWheelSpeed(x, z): #from sabertooth
	if z > 0.2:
	#left
		if z > 0.9:
			left, right = z, -1 * z
		else:
			left, right = z, x * z * -1
	elif z < -0.2:
	#right
		if z < -0.9:
			right, left = z, -1 * z #changed from left, right
		else:
			right, left = x, x * z * -1 #changed from left, right
	else:
		left, right = x, x

	scaledLeft = int(left*180) #[-180, 180] in integer
	scaledRight = int(right*180) #[-180, 180]
	return scaledLeft, scaledRight


# (0,~180) rpms converted to (64,127)
# Y=0.35+64
#m, b = 0.35, 64
#rightVel = (m*rightChannel+b)
def encoderCallback(msg):
	# now we work in (-180, 180) integer
	leftEncoder = msg.left
	rightEncoder = msg.right
	# we need this to make encoders [-180, 180] as opposed to [0, 180]
	if fowards == True:
		leftVel = leftEncoder
		rightVel = rightEncoder
	elif fowards == False:
		leftVel = (-1*leftEncoder)
		rightVel = (-1*rightEncoder)

if __name__ == '__main__':
	# keep track of values for plotting
	#x, y = [], [] #these are for plotting
	#setpoint = []
	serial_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=19200,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	)
	rospy.init_node('PIController', anonymous=True)
	rate = rospy.Rate(10) # The others are 10Hz too

	motor = Motor()

	lspeed = motor.lspeed
	rspeed = motor.rspeed

	ljoySpeed = joyLeft
	rjoySpeed = joyRight

	lpid = PID(5, 0.01, 0, setpoint=ljoySpeed) # don't need kd
	rpid = PID(5, 0.01, 0, setpoint=rjoySpeed) # don't need kd

	lpid.output_limits = (-180, 180)
	rpid.output_limits = (-180, 180)

	start_time = time.time()
	last_time = start_time

	rospy.Subscriber("/manual_control_vel", Twist, controlInputCallback)
	# int joystick inputs
	# these represent setpoint
	#print(joyLeft, joyRight)
	rospy.Subscriber("/encoderticks", channel_msgs, encoderCallback)
	# rpms
	# these will give actual speed
	#print(leftVel, rightVel)

	while True:
		#x += [current_time - start_time] #for plotting
		#y += [speed] #for plotting
		#setpoint += [pid.setpoint]
		current_time = time.time()
		dt = current_time - last_time

		lpower = lpid(lspeed)
		#print(lpower) #for debugging
		#time.sleep(0.1) #for debugging
		lspeed = motor.lupdate(lpower, dt)
		rpower = rpid(rspeed)
		rspeed = motor.rupdate(rpower, dt)

		last_time = current_time

"""
    plt.plot(x, y, label='measured')
    plt.plot(x, setpoint, label='target')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.legend()
    plt.show()
"""
