#Nathan Lemus and Shawn Leones

#!/usr/bin/python3
import math
import time
import serial
import rospy #for talker/ listener
from geometry_msgs.msg import Twist #for talker/ listener

print("UART Demonstration Program")

#
def controlInputCallback(msg):
    x = msg.linear.x      # -1 < x < 1
    z = msg.angular.z * 2 # -1 < z < 1
    left, right = twistToWheelSpeed(x, z)
    sendWheelSpeedToMotors(left, right)
    print(left, right)

def twistToWheelSpeed(x, z):
    if z > 0.4:
	#right
	if z > 0.9:
        	left = -1 * z
        	right = z
	else:
		left = x * z
        	right = x
    elif z < -0.4:
	#left
        if z < -0.9:
		left = z
        	right = -1 * z
	else:
		left = x
        	right = x * z
    else:
        left, right = x, x
    return left, right

# -1 < left  < 1
# -1 < right < 1
def sendWheelSpeedToMotors(left, right):
    leftChr = (64+((127*left)/2))
    rightChr = left+128 # it can go faster going left
    serial_port.write(chr(int(leftChr)))
    serial_port.write(chr(int(rightChr)))

#function calls
if __name__ == '__main__':
    serial_port = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=19200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        )

    rospy.init_node('uart', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/manual_control_vel", Twist, controlInputCallback)
    rospy.spin()
