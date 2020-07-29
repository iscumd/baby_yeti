#!/usr/bin/python3
import time
import serial
import rospy
import queue
from baby.msg import channel_msgs

print("encoder input")
left, right = 0,0
lMarker, rMarker = 255, 254

if __name__ == '__main__':
	serial_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=19200,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	)

	rospy.init_node('rx', anonymous=True)
	pub = rospy.Publisher('encoders', channel_msgs, queue_size=10)
	rate = rospy.Rate(30)

	queue = []

	c = channel_msgs()

	l, r = False, False

	try:
		while True:
			encoderticks = serial_port.read()
			putty = int.from_bytes(encoderticks, byteorder='little')
			
			if putty == lMarker:
				l = True
			elif putty == rMarker:
				r = True
			elif l == True:
				left = putty
				c.left = left
				l = False
			elif r == True:
				right = putty
				c.right = right
				r = False
			else:
				print("garbage")

			c.left = left
			c.right = right
			print(c)

			if serial_port.inWaiting() >= 0:
				pub.publish('c')


	except KeyboardInterrupt:
		print("Exiting Program")
	finally:
		serial_port.close()
	pass

rospy.spin()
