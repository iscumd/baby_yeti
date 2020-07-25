#!/usr/bin/python3
import time
import serial
import rospy
from std_msgs.msg import Int64

print("wallah")

if __name__ == '__main__':
	serial_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=19200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        )
	
	pub = rospy.Publisher('encoders', Int64, queue_size=1)
	rospy.init_node('rx', anonymous=True)	
	rate = rospy.Rate(30)

	try:
		while True:	
			#if serial_port.inWaiting() > 0:
			encoderticks = serial_port.read()
			putty = int.from_bytes(encoderticks, byteorder='little')		
			if serial_port.inWaiting() >= 0:
				pub.publish(putty)
				print(putty)
			
	except KeyboardInterrupt:
		print("Exiting Program")
	finally:
		serial_port.close()
	pass
	
	rospy.spin()
	

