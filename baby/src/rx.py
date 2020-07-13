#!/usr/bin/python3
import math
import time
import serial
import rospy #for talker/ listener
from geometry_msgs.msg import Twist #for talker/ listener


#Receives wheel encoder counts from Arduino mega and converts to linear speed
print("Test for rx (receiving uart)")

def encoderRead()
    # Send a simple header
    serial_port.write("UART Demonstration Program\r\n".encode())
    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while True:
        if serial_port.inWaiting() > 0:
            data = serial_port.read()
            print(data)


except KeyboardInterrupt:
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    serial_port.close()
    pass

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
    encoderRead()
    rospy.spin()
