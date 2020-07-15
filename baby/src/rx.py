#!/usr/bin/python3
import time
import serial
import rospy

print("reading")

def readEncoders():
	try:
		while True:	
			if serial_port.inWaiting() > 0:
				encoderticks = serial_port.read(3)
				print(encoderticks)
				#penis = int.from_bytes(encoderticks, byteorder='little')
		
				#print(encoderticks)
			
	except KeyboardInterrupt:
		print("Exiting Program")

	except Exception as exception_error:
		print("Error occurred. Exiting Program")
		print("Error: " + str(exception_error))

	finally:
		serial_port.close()
	pass

if __name__ == '__main__':
	serial_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=19200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        )

	rospy.init_node('rx', anonymous=True)	
	rate = rospy.Rate(30)
	readEncoders()
	rospy.spin()




