########
# Name: trivia.py
#
# Purpose: A trivia that creates a trivia database, selects questions, and checks answers.
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
#
# Date: 22 May 2024
#
#####################

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node
import json 
import random 

###
# Method: 
# Purpose: 
#
######
class Trivia(Node):

    def __init__(self, filename):
        # initalize
        super().__init__('trivia')
        # Read JSON data from a file
        with open(filename, 'r') as file:
            self.database = json.load(file)
        self.answer_mapping = {'7': '1', '9': '2', '1': '3', '3': '4', '0': 'repeat'} 
 
    ###
    # Name: get_question 
    # Purpose: get key and content of random question from category selected 
    # Arguments: self(reference the current class), category 
    # Outputs: key of question, question and answer content  
    def get_question(self, category):
        key = str(random.randint(1, 3))
        question = " ".join(self.database[category][key]['question'])
        return key, question

    ###
    # Name: check_answer 
    # Purpose: check whether the user answered the question correctly 
    # Arguments:  self (reference the current class), color (the question category), key (the id of the question), guess (the user's answer guess)
    #####
    def check_answer(self, color, key, guess):
        return self.database[color][key]["correct_answer"] == guess
    
    ###
    # Name: get_time 
    # Purpose: get the time, in seconds, it takes to read out a question
    # Arguments:  self (reference the current class), color (the question category), key (the id of the question)
    #####
    def get_time(self, color, key):
        return self.database[color][key]["time"]
        
    def get_user_answer(self):
        while(True):
            answer = input("What is your answer?")
            if answer[-1] in self.answer_mapping:
                return self.answer_mapping[answer[-1]]
            else:
                print("Answer is invalid")
