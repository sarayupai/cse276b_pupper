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
import os
import pygame 

# Packages to let us create nodes and spin them up 
import rclpy
import time
from rclpy.node import Node

def main():
    rclpy.init()

    # Initialize helper modules 
    client = MinimalClientAsync()
    movement_controller = SampleControllerAsync(client)
    audio = Audio(client)
    trivia = Trivia('database.json')

    # Start game 
    pygame.init()
    audio.speak('Game start')
    time.sleep(5.0)
    audio.stop_speak()
    
    #audio.speak(trivia.database[0])
    #time.sleep(5.0)
    #audio.stop_speak()

    # send commands to do move
    #movement_controller.sensor_movement()

    def get_user_answer():
        while True:
            for event in pygame.event.get():
                #if event.type == pygame.QUIT:
                    #pygame.quit()
                if event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4]:
                        return event.key 
    
    def trivia_mode(color):
        while(True):
            # get new question - ie key and question+answer content 
            question_key, question = trivia.get_question(color)
            # read out question to user 
            audio.speak(question)
            # get user answer + stop audio
            guess = get_user_answer()
    	    audio.stop_speak()
            correct = trivia.check_answer(color, question_key, guess)
            # if answer is correct, exit trivia mode, else repeat with new question 
    	    if correct: 
    	        return 
 
    
    '''
    # game loop 
    while(play):
        # TODO: COLOR DETECT CODE 
        if color_detect: 
            trivia_mode()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.KEYDOWN:
                mover.sensor_movement(event.key)
    '''

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
