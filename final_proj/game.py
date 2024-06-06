#!/usr/bin/python
###############
# Name: game.py
# Description: Main file for Maze Pupper Trivia Game 
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
# Date: 22 May 2024
#############

#### Import ####
from audio import Audio 
from trivia import Trivia 
from client_go_pupper import MinimalClientAsync
from teleop import Teleop
import sys 

# Packages to let us create nodes and spin them up 
import rclpy
import time
import random 
from rclpy.node import Node

from std_msgs.msg import Bool 
from std_msgs.msg import String

class Game(Node):
    def __init__(self, audio):
        # initialize 
        super().__init__('game')
        self.trivia = Trivia('/home/ubuntu/ros2_ws/src/final_proj/final_proj/database.json')
        self.audio = audio 
        self.teleop = Teleop()
        self.nod = Nod()

    def start_game(self):
        self.audio.speak('Game start')
        print('passed initial audio')
        self.audio.stop_speak()

    def end_game(self):
        score = self.trivia.get_score()
        self.audio.speak(f'Game end. Your score is {score}')
        self.audio.stop_speak()
    
    def trivia_mode(self, color):
        while(True):
            # get new question - ie key and question+answer content 
            question_key, question = self.trivia.get_question(color)
            
            while(True):
                # read out question to user 
            	self.audio.speak(question)
            	# get user answer + stop audio
            	guess = self.trivia.get_user_answer()
            	self.audio.stop_speak()
            	if guess != 'repeat':
            	    break
            
            correct = self.trivia.check_answer(color, question_key, guess)
            
            # if answer is correct, exit trivia mode, else repeat with new question 
            if correct: 
                self.audio.speak("That's correct! You may proceed through the maze")
                self.teleop.headnod(True)
                # self.nod.look_up()
                # self.nod.reset()
                # self.nod.look_down()
                # self.nod.reset()
                self.audio.stop_speak()
                return 
            else:
                self.audio.speak("That's not quite right! Let me give you another chance.")
                self.teleop.headnod(False)
                # self.nod.look_left()
                # self.nod.reset()
                # self.nod.look_right()
                # self.nod.reset()
                self.audio.stop_speak()
                
    
    # game loop: if color detected -> enter trivia mode, else allow movement 
    def game_loop(self, level):
        play = True 
        colors = ['red', 'green', 'blue']
        while(play):
            self.audio.speak("Entering move mode")
            print('enter move mode')
            self.teleop.poll_keys(level) # should exit after a set interval 
            
            self.audio.speak("Entering trivia mode")
            print('enter trivia mode')
            rand_color = random.randint(0,2)
            self.trivia_mode(colors[rand_color])

def main(level):
    rclpy.init()

    # Initialize helper modules 
    client = MinimalClientAsync()
    audio = Audio(client)

    game = Game(audio)
    #print('now trying to spin up')
    
    # Start game 
    game.start_game()

    # TODO: find a way tp quit the game from the game loop 
    game.game_loop(level)

    # End Game 
    game.end_game()
    
    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    while rclpy.ok():
        rclpy.spin_once(client)
        # rclpy.spin_once(game)
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
    # Ensure the correct number of arguments are provided
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <level> ")
        sys.exit(1)
    
    # Extract the arguments from sys.argv
    level = sys.argv[1]
    
    # Call the main function with the level selected 
    main(level)
