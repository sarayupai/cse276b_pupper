#!/usr/bin/python
###############
# Name: main.py
# Description: Main file for Maze Pupper Trivia Game 
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
# Date: 22 May 2024
#############

#### Import ####
from .movement import SampleControllerAsync
import rclpy
import time
from rclpy.node import Node

def main():
    rclpy.init()
    sample_controller = SampleControllerAsync()

    # send commands to do the conga dance
    sample_controller.sensor_movement()

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    while rclpy.ok():
        rclpy.spin_once(sample_controller)
        if sample_controller.future.done():
            try:
                response = sample_controller.future.result()
            except Exception as e:
                sample_controller.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                sample_controller.get_logger().info(
                   'Result of command: %s ' %
                   (response))
            break

    # Destroy node and shut down
    sample_controller.destroy_node()
    rclpy.shutdown()
    
    
    trivia = Trivia()
    
    # If detect color -> enter trivia
    	question_key = trivia.get_question(color)
    	# get user input 
    	guess = ??
    	correct = trivia.check_answer(question_key, guess)
    	if correct: 
    	    # call move function to move 
    	else: 
    	    
    	


if __name__ == '__main__':
    main()
