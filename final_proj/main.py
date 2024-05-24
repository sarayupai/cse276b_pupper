#!/usr/bin/python
###############
# Name: main.py
# Description: Main file for Maze Pupper Trivia Game 
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
# Date: 22 May 2024
#############

#### Import ####
from movement import SampleControllerAsync
from audio import Audio 
from trivia import Trivia 
from client_go_pupper import MinimalClientAsync

# Packages to let us create nodes and spin them up 
import rclpy
#import time
from rclpy.node import Node

def main():
    rclpy.init()
    client = MinimalClientAsync()
    
    
    audio = Audio(client)
    #sample_controller = SampleControllerAsync(client)
    trivia = Trivia()
    
    audio.speak('Game start') 
    audio.speak(trivia.database[0])

    # send commands to do the conga dance
    #sample_controller.sensor_movement()
    
    
    
    
    # If detect color -> enter trivia
    	# question_key = trivia.get_question(color)
    	# get user input 
    	# guess = ??
    	# correct = trivia.check_answer(question_key, guess)
    	# if correct: 
    	    # call move function to move 
    	# else: 

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    while rclpy.ok():
        rclpy.spin_once(client)
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
