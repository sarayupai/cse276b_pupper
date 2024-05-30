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

# Packages to let us create nodes and spin them up 
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool 
from std_msgs.msg import String


class Game(Node):
    def __init__(self, audio):
        # initialize 
        super().__init__('game')
        self.subscription = self.create_subscription(String, 'color_detection', self.game_loop, 10)
        self.publisher = self.create_publisher(Bool, 'enable_movement', 10)
        self.trivia = Trivia('/home/ubuntu/ros2_ws/src/final_proj/final_proj/database.json')
        self.audio = audio 
    
   # TODO 
    def get_user_answer(self):
        return 1
    
    def trivia_mode(self, color):
        while(True):
            # get new question - ie key and question+answer content 
            question_key, question = self.trivia.get_question(color)
            # read out question to user 
            self.audio.speak(question)
            time.sleep(10.0)
            # get user answer + stop audio
            guess = self.get_user_answer() #TODO
            self.audio.stop_speak()
            correct = self.trivia.check_answer(color, question_key, guess)
            # if answer is correct, exit trivia mode, else repeat with new question 
            if correct: return 
    
    # game loop: if color detected -> enter trivia mode, else allow movement 
    def game_loop(self, color):
        enable_movement_msg = Bool()
        if color.data == 'none':
            enable_movement_msg.data = True
            self.publisher.publish(enable_movement_msg)
        else:
            enable_movement_msg.data = False
            self.publisher.publish(enable_movement_msg)
            self.trivia_mode(color.data)

def main():
    rclpy.init()

    # Initialize helper modules 
    client = MinimalClientAsync()
    audio = Audio(client)

    # Start game
    audio.speak('Game start')
    print('passed initial audio')
    time.sleep(5.0)
    audio.stop_speak()
    
    # Not sure about the order here ...
    game = Game(audio)
    rclpy.spin(game)
    
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
    game.destroy_node()
    client.destroy_node()
    rclpy.shutdown()	


if __name__ == '__main__':
    main()
