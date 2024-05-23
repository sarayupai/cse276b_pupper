########
# Name: movement.py
#
# Purpose: A sample Pupper controller. Will move the robot around a bit then stop.
#
# Author: Prof. Riek <lriek@ucsd.edu>, Prisha Anand, Advika Sonti, Sarayu Pai 
#
# Date: 30 April 2024
#
#####################

# Our custom interface, GoPupper. This specifies the message type (commands).
#from pupper_interfaces.srv import GoPupper
import client_go_pupper 

# Packages to let us create nodes and spin them up
import rclpy
#import time
import RPi.GPIO as GPIO
from rclpy.node import Node

###
# Method: Sample Controller Async
# Purpose: Constructor for the controler
#
######

# There are 4 areas for touch actions
# Each GPIO to each touch area
touchPin_Front = 6
touchPin_Left  = 3
touchPin_Right = 16
touchPin_Back  = 2

# Use GPIO number but not PIN number
GPIO.setmode(GPIO.BCM)

# Set up GPIO numbers to input
GPIO.setup(touchPin_Front, GPIO.IN)
GPIO.setup(touchPin_Left,  GPIO.IN)
GPIO.setup(touchPin_Right, GPIO.IN)
GPIO.setup(touchPin_Back,  GPIO.IN)


class SampleControllerAsync(Node):

    def __init__(self):
        # initalize
        super().__init__('movement')
        self.client = MinimalClientAsync()

    def sensor_movement(self):
        while True:
            # Store detection
            touchValue_Front = GPIO.input(touchPin_Front)
            touchValue_Back = GPIO.input(touchPin_Back)
            touchValue_Left = GPIO.input(touchPin_Left)
            touchValue_Right = GPIO.input(touchPin_Right)
            display_sting = ''
            
            # check right 
            if not touchValue_Right:
                display_sting += ' Right'
                self.client.send_move_request("move_right")
     				
     	    # check left 
            if not touchValue_Left:
                display_sting += ' Left'
                self.client.send_move_request("move_left")
                
            # check front
            if not touchValue_Front:
                display_sting += ' Front'
                self.client.send_move_request("move_forward")
     	        
     	    # check back
            if not touchValue_Back:
                display_sting += ' Back'
                self.client.send_move_request("move_backward")
                
            if display_sting == '':
                display_sting = 'No button touched'
            print(display_sting)
            # time.sleep(0.5)
