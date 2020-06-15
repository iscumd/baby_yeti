#Nathan Lemus and Shawn Leones
# Copyright (c) 2019-2020, NVIDIA CORPORATION. Liscence removed for brevity
# See NVIDIA/jetson_gpio
#Twist to PWM. output on gpio


#!/usr/bin/env python
import math #floor/ceil
import RPi.GPIO as GPIO
import time
import rospy #for talker/ listener
from geometry_msgs.msg import Twist #for talker/ listener

output_pins = { #gpio pins
    'out_a': 32,
    'out_b': 33, #need to configure 33 for pwm?
}

output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception('PWM not supported on this board')

pwmLeft = 0
pwmRight = 0


#rounds twist inputs to -1 => 1; step = 0.1
def floorCeiling(f):
    f = (f*10)
    if f > 0:
        var = math.ceil(f)
    else:
        var = math.floor(f)
    return (var/10)


#duty cycle percentage for each channel
def DutyCycleConversion(i, j):
    if i > 0:
        i = (50 + ((100 * i)/2))
    else:
        i = (50 - ((100 * i)/2))

    if j > 0:
        j = (50 + ((100 * j)/2))
    else:
        j = (50 - ((100 * j)/2))

    pwmLeft = i
    pwmRight = j

    if j > 50:
        pwmLeft += j/2
        pwmRight -= j/2
    else:
        pwmLeft -= j/2
        pwmRight += j/2


#twist to duty cycle
def callback(msg):
    x = msg.linear.x #msg.msg.linear.x?
    z = msg.angular.z
    rospy.loginfo('x: {}, z: {}'.format(x,z))
    fX = floorCeiling(x)
    fZ = floorCeiling(z)
    DutyCycleConversion(fX, fZ)


#subscriber
def listener():
    rospy.init_node('isc_joy/manual_control', anonymous=True)
    rospy.Subscriber("/manual_control_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




#main;
def main():
    # Pin Setup:, # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(pin_data['out_a'], pin_data['out_b'], GPIO.OUT, initial=GPIO.HIGH)
    p = GPIO.PWM(pin_data['out_a'], 500) #specification of sabertooth2x12??
    q = GPIO.PWM(pin_data['out_b'], 500) #500? 1000+ hz recommended by mfg
    p.start(50)
    q.start(50)
    #    p.ChangeDutyCycle(pwmLeft) #variable from callback
    #    q.ChangeDutyCycle(pwmRight)
    #p.stop()  see below
    #q.stop()  see below
    #del p   do we need these??
    #del q
    #GPIO.cleanup()  see below

#change this portion
    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            #time.sleep(0.25)
            #if val >= 100:  do we need to change based on an update to the subscriber???????
            #    incr = -incr
            #if val <= 0:
            #    incr = -incr
            #val += incr
            #p.ChangeDutyCycle(val)   --seperate function?
            p.ChangeDutyCycle(pwmLeft) #variable from callback
            q.ChangeDutyCycle(pwmRight)
    finally:
        p.stop()
        q.stop()
        GPIO.cleanup()



#function calls
if __name__ == '__main__':
    main()
    listener() #to subscribe to twist messages
    floorCeiling(f) #put "f" here?
    DutyCycleConversion(i, j) #put "i, j" here?
    callback(msg)
    listener()
