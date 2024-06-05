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
        #self.subscription = self.create_subscription(String, 'timer_control', self.game_loop, 10)
        #self.publisher = self.create_publisher(Bool, 'enable_movement', 10)
        self.trivia = Trivia('/home/ubuntu/ros2_ws/src/final_proj/final_proj/database.json')
        self.audio = audio 
        self.teleop = Teleop()

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
            	guess = self.trivia.get_user_answer() #TODO
            	self.audio.stop_speak()
            	if guess != 'repeat':
            	    break
            
            correct = self.trivia.check_answer(color, question_key, guess)
            
            # if answer is correct, exit trivia mode, else repeat with new question 
            if correct: 
                self.audio.speak("That's correct! You may proceed through the maze")
                self.teleop.headnod(True)
                self.audio.stop_speak()
                return 
            else:
                self.audio.speak("That's not quite right! Let me give you another chance.")
                self.teleop.headnod(False)
                self.audio.stop_speak()
                
    
    # game loop: if color detected -> enter trivia mode, else allow movement 
    def game_loop(self):
        play = True 
        colors = ['red', 'green', 'blue']
        while(play):
            self.audio.speak("Entering move mode")
            time.sleep(5.0)
            print('enter move mode')
            self.teleop.poll_keys() # should exit after a set interval 
            
            self.audio.speak("Entering trivia mode")
            time.sleep(5.0)
            print('enter trivia mode')
            # TODO: add logic to pick a color
            rand_color = random.randint(0,2)
            self.trivia_mode(colors[rand_color])

def main():
    rclpy.init()

    # Initialize helper modules 
    client = MinimalClientAsync()
    audio = Audio(client)

    game = Game(audio)
    #print('now trying to spin up')
    
    # Start game 
    game.start_game()

    # TODO: find a way tp quit the game from the game loop 
    game.game_loop()

    # End Game 
    game.end_game()
    #rclpy.spin(game)
    
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
    # trivia.destroy_node()
    # audio.destroy_node()
    #game.destroy_node()
    client.destroy_node()
    rclpy.shutdown()	


if __name__ == '__main__':
    main()
