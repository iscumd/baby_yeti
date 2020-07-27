#!/usr/bin/python3
import time
import serial
import rospy
from baby.msg import channel_msgs

print("encoder input")
left, right = 0,0
lMarker, rMarker = 254, 255

if __name__ == '__main__':
	serial_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=19200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )

	pub = rospy.Publisher('encoders', channel_msgs, queue_size=1)
	rospy.init_node('rx', anonymous=True)
	rate = rospy.Rate(30)

	q.Queue(maxsize = 3)
	incomming = 0
	garbage = 0
	c = channel_msgs()
	c.left = left
	c.right = right

	try:
		while True:
			encoderticks = serial_port.read()
			putty = int.from_bytes(encoderticks, byteorder='little')

			q.put_nowait(putty)
			incomming = q.get_nowait()
			if incomming != (lMarker or rMarker):
				garbage = q.get_nowait()
			elif incomming == lMakrer:
				left = q.get_nowait()
			elif incomiing == rMarker:
				right = q.get_nowait()
			else:
				print("error, check input data from encoders")

			if serial_port.inWaiting() >= 0:
				pub.publish(c)
				#print(c) #for debug

	except KeyboardInterrupt:
		print("Exiting Program")
	finally:
		serial_port.close()
	pass

	rospy.spin()
