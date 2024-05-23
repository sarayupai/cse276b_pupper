########
# Name: trivia.py
#
# Purpose: A trivia database. Selects questions. checks answrrs .
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
#
# Date: 22 May 2024
#
#####################

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node

###
# Method: 
# Purpose: 
#
######

class Trivia(Node):

    def __init__(self):
        # initalize
        # super().__init__('trivia')
        # create database - ie pull from json + populate dict 
        self.database = ?? 

 
    ###
    # Name: get_question 
    # Purpose: get random question from category selected + read out q+a to user  
    # Arguments: self(reference the current class), category 
    # Outputs: key of question 
    def get_question(self, category):
    	self.database[category] 
    	
    	return key 
    	
       
    ###
    # Name: check_answer 
    # Purpose: check whether the user answered the question correctly 
    # Arguments:  self (reference the current class), key (the id of the question), quess (the user's answer guess)
    #####
    def check_answer(self, key, guess):
    	correct = False 
    
    	return correct 
        
