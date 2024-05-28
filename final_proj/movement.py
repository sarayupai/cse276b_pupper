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
from client_go_pupper import MinimalClientAsync

# Packages to let us create nodes and spin them up
import rclpy
#import time
import RPi.GPIO as GPIO
from rclpy.node import Node
#from pynput import keyboard

import pygame 


###
# Method: Sample Controller Async
# Purpose: Constructor for the controler
#
######

# There are 4 areas for touch actions
# Each GPIO to each touch area
'''
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
'''

class SampleControllerAsync(Node):

    def __init__(self, client):
        # initalize
        super().__init__('movement')
        self.client = client
        
        # TODO: keyboard listener
        #self.listener = keyboard.Listener(sensor_movement=self.sensor_movement)
        #self.listener.start()
        #self.get_logger().info('keyboard listener started')
    
    def sensor_movement(self, key):
        #while True:
            '''
            # Store detection
            touchValue_Front = GPIO.input(touchPin_Front)
            touchValue_Back = GPIO.input(touchPin_Back)
            touchValue_Left = GPIO.input(touchPin_Left)
            touchValue_Right = GPIO.input(touchPin_Right)
            '''
        display_string = ''
            
        # check right 
        # if not touchValue_Right:
        if key == pygame.K_RIGHT:
        #if key == keyboard.Key.right:
            display_string += ' Right'
            self.client.send_move_request("turn_right")
     				
     	# check left 
        # if not touchValue_Left:
        elif key = pygame.K_LEFT:
        #elif key == keyboard.Key.left:
            display_string += ' Left'
            self.client.send_move_request("turn_left")
                
        # check front
        # if not touchValue_Front:
        elif key = pygame.K_UP:
        #elif key == keyboard.Key.up:
            display_string += ' Front'
            self.client.send_move_request("move_forward")
     	        
     	# check back
        # if not touchValue_Back:
        elif key = pygame.Key_DOWN:
        #elif key == keyboard.Key.down:
            display_string += ' Back'
            self.client.send_move_request("move_backward")
                
        if display_string == '':
            display_string = 'No button touched'
            print(display_string)
            # time.sleep(0.5)

# main function to test move functionality             
def main():
    rclpy.init()
    client = MinimalClientAsync()
    mover = SampleControllerAsync(client)
    
    # TODO: erroing bc cant pass in key
    #mover.sensor_movement()

    
    pygame.init()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.KEYDOWN:
                mover.sensor_movement(event.key)

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    # (Probably a bit redundant with other code and can be simplified. But right now it works, so ¯\_(ツ)_/¯)
    while rclpy.ok():
        rclpy.spin_once(mover)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                client.get_logger().info(
                   'Result of command: %s ' %
                   (response))
            break

    # Destroy node and shut down
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
