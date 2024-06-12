########
# Name: trivia.py
# Student: Prisha Anand, Advika Sonti, Sarayu Pai
# Final Project: The Maze Pupper 
# 
# Description: Serves a trivia manager. Creates a trivia database, selects questions
# and fetches them from the json file. Checks if the user inputted answer 
# corresponds to the correct answer. 
#
# Date: 22 May 2024
# 
#####################

import rclpy
from rclpy.node import Node
import json 
import random 

class Trivia(Node):

    ####
    # Name: __init__
    # Purpose: Initialize a Trivia object
    # @input: filename (string) - The path to the JSON database file
    ####
    def __init__(self, filename):
        # initalize
        super().__init__('trivia')
        # Read JSON data from a file
        with open(filename, 'r') as file:
            self.database = json.load(file)
        # Dictionary for answers and key pressed 
        self.answer_mapping = {'7': '1', '9': '2', '1': '3', '3': '4', '0': 'repeat'} 
        self.total = 0
        self.correct = 0
  
    ####
    # Name: get_question
    # Purpose: Get a random question from the trivia database for a specified category
    # @input: category (string) - The category of the question
    # @return: key (string) - The key of the selected question in the database
    #          question (string) - The text of the selected question
    ####
    def get_question(self, category):
        self.total += 1 
        key = str(random.randint(1, 8))
        question = " .".join(self.database[category][key]['question'])
        return key, question

    ####
    # Name: check_answer
    # Purpose: Check if the provided guess matches the correct answer for a given question
    # @input: color (string) - The color category of the question
    #         key (string) - The key of the question in the database
    #         guess (string) - The user's guess for the answer
    # @return: correct (bool) - True if the guess is correct, False otherwise
    ####
    def check_answer(self, color, key, guess):
        correct = (self.database[color][key]["correct_answer"] == guess)
        if correct: self.correct+=1 
        return correct 
    
    ####
    # Name: get_time
    # Purpose: Get the time limit for answering a question
    # @input: color (string) - The color category of the question
    #         key (string) - The key of the question in the database
    # @return: time_limit (float) - The time limit for answering the question
    ####
    def get_time(self, color, key):
        return self.database[color][key]["time"]
        
    ####
    # Name: get_user_answer
    # Purpose: Get the user's answer to a trivia question
    # @return: user_answer (string) - The user's answer mapped to a valid response
    ####
    def get_user_answer(self):
        while(True):
            answer = input("What is your answer?")
            if answer[-1] in self.answer_mapping:
                return self.answer_mapping[answer[-1]]
            else:
                print("Answer is invalid")

    ####
    # Name: get_score
    # Purpose: Compute score based on correct answers. 
    # @return: correct (int) - The number of correct answers
    #          total (int) - The total number of questions answered
    ####
    def get_score(self):
        return self.correct, self.total
