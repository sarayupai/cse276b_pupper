########
# Name: game.py
# Student: Prisha Anand, Advika Sonti, Sarayu Pai
# Final Project: The Maze Pupper 
# 
# Description: Master engine dealing with the gaemeplay of the project. Handeles
# game life cycle, switching between trivia and movement modes, asking trivia questions
# and checking user response to triva questions. Takes in a command line argument 
# specifying the level for the game. 
#
# Date: 22 May 2024
# 
# How to Use: 
# colcon build --packages-select final_proj
# ros2 launch mini_pupper_bringup bringup.launch.py
# python game.py (level)
#####################

#### Import ####
from audio import Audio 
from trivia import Trivia 
from client_go_pupper import MinimalClientAsync
from teleop import Teleop
import sys 
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool 
from std_msgs.msg import String

class Game(Node):

    ####
    # Name: __init__
    # Purpose: Initialize the game node with trivia, audio, and teleop components.
    # @input: audio (Audio) - An instance of the Audio class
    ####
    def __init__(self, audio):
        # initialize 
        super().__init__('game')
        self.trivia = Trivia('/home/ubuntu/ros2_ws/src/final_proj/final_proj/database.json')
        self.audio = audio 
        self.teleop = Teleop()
        self.start_time = 0
    
    ####
    # Name: start_game
    # Purpose: Start the game by announcing the beginning and recording the start time.
    ####
    def start_game(self):
        self.audio.speak('Welcome to the Maze!')
        self.audio.stop_speak()
        # Start game timer
        self.start_time = time.time()

    ####
    # Name: end_game
    # Purpose: End the game by announcing the duration and score
    ####
    def end_game(self):
        end_time = time.time()
        length = end_time - self.start_time 
        minutes = int(length // 60)
        seconds = int(length % 60)
        num_correct, num_questions = self.trivia.get_score()
        self.audio.speak(f'Game over. You answered {num_correct} out of {num_questions} questions correctly. Your total time was {minutes} minutes and {seconds} seconds.')
        self.audio.stop_speak()

    ####
    # Name: trivia_ques
    # Purpose: Ask a trivia question to the user, get their answer, and check if it's correct
    # @input:  question (string) - The trivia question to be asked 
    #          color (string) - The color associated with the question
    #          question_key (string) - The key of the question in the trivia database
    # @return: correct (bool) - True if the user's answer is correct, False otherwise
    ####    
    def trivia_ques(self, question, color, question_key): 
        while(True):
            # read out question to user 
            self.audio.speak(question)
            # get user answer + stop audio
            guess = self.trivia.get_user_answer()
            self.audio.stop_speak()
            # re-read if repeat key pressed
            if guess != 'repeat':
                break
        # check if correct
        correct = self.trivia.check_answer(color, question_key, guess)
        return correct

    ####
    # Name: react_guess
    # Purpose: React to the user's guess by playing an audio cue, changing the display, and providing feedback. 
    # @input:  correct (bool) - Indicates whether the user's guess was correct or not
    ####     
    def react_guess(self, correct):
        if correct:
            self.audio.play_audio('correct.mp3')
            self.teleop.display_category(True)
            self.audio.stop_audio()
            self.audio.speak("That's correct!")
            self.audio.stop_speak()
        else:
            self.audio.play_audio('incorrect.mp3')
            self.teleop.display_category(False)
            self.audio.stop_audio()
            self.audio.speak("That's not quite right!")
            self.audio.stop_speak()

    ####
    # Name:
    # Purpose: 
    # @input:  
    # @return: 
    ####
    def trivia_mode(self, color, level):
        # display category on screen 
        self.teleop.category(color)
        # tracks whether the user answered the first question correctly 
        first_try = True
        while(True):
            # get new question - ie key and question+answer content
            question_key, question = self.trivia.get_question(color)
            correct = self.trivia_ques(question, color,  question_key)
            
            # if in level1 and user is wrong, give them a second chance at question 
            if level == 'level1' and not correct:
                self.audio.play_audio('incorrect.mp3')
                self.teleop.display_category(False)
                self.audio.stop_audio()
                self.audio.speak("You get one more chance!")
                self.audio.stop_speak()
                correct = self.trivia_ques(question, color, question_key)

            # if answer is correct, exit trivia mode, else repeat with new question 
            self.react_guess(correct)
            if correct:
                return first_try 
            else:
                first_try = False 
    
    ####
    # Name: game_loop
    # Purpose: Cycle between movement mode and trivia mode until the user quits
    # @input:  level (int) - The level of the game
    ####
    def game_loop(self, level):
        play = True 
        colors = ['red', 'green', 'blue']
        idx = 0
        first_try = False 
        
        while(play):
            # Enter move mode
            self.audio.speak("Entering move mode")
            status = self.teleop.poll_keys(level, first_try) 
            # Should exit move mode after a set interval, unless quit key pressed 
            if status == 'quit':
                break

            # Enter triiva mode 
            print('enter trivia mode')
            self.audio.speak("Entering trivia mode")
            # Cycle through the trivia categories
            first_try = self.trivia_mode(colors[idx], level)
            idx+=1
            if idx == 3: idx=0

####
# Name: main
# Purpose: Initialize the game, start and run the game loop, and then end the game.
# @input:  level (string) - The level of the game
####
def main(level):
    rclpy.init()

    # Initialize audio modules 
    client = MinimalClientAsync()
    audio = Audio(client)

    # Setup gameplay
    game = Game(audio)
    game.start_game()
    game.game_loop(level) # Begin game with specified level
    game.end_game()
    
    # Spin up client node
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
    # Ensure the correct number of arguments are provided
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <level#> ")
        sys.exit(1)
    
    # Extract the arguments from command line
    levels = ['level1', 'level2']
    level = sys.argv[1]
    if level not in levels:
        print("Please select a valid level")
        sys.exit(1)
    
    # Call the main function with the level selected 
    main(level)
