#Nathan Lemus and Shawn Leones

#!/usr/bin/python
import math
import serial
import rospy #for talker/ listener
from geometry_msgs.msg import Twist #for talker/ listener

#
def controlInputCallback(msg):
    x = msg.linear.x #msg.msg.linear.x?
    z = msg.angular.z
    fX = round(x)
    fZ = round(z)
    conversion(fX, fZ)


#rounds twist inputs to -1 => 1; step = 0.1
def round(f):
    f = (f*10)
    if f > 0:
        var = math.ceil(f)
    else:
        var = math.floor(f)
    return (var/10)



#duty cycle percentage for each channel
# 1-127, 64 is stop ch 1
# 128-255, 192 is stop ch 2
# 0 stops both
def conversion(i, j):
    if i >= 0:
        i = (64+((127*i)/2))
    else:
        i = (64-((127*i)/2))

    #sets left and right for no turning
    left = i
    right = i+128

    if j >= 0:
        j = (64+((127*j)/2))
    else:
        j = (64-((127*j)/2))

    if j > 65:
        left += j/2
        right -= j/2
    elif j < 63:
        left -= j/2
        right += j/2
    else:
        left = i
        right = i+128

    serial_port.write(left)
    serial_port.write(right)



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
