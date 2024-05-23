########
# Name: audio.py
#
# Purpose: provides text to speech audio support 
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
#
# Date: 23 May 2024
#
#####################

from gtts import gTTS 
import client_go_pupper 
# Packages to let us create nodes and spin them up 
import rclpy # ??
from rclpy.node import Node

class Audio(Node):

    def __init__(self):
        # initialize 
        super().__init__('audio')
        self.client = MinimalClientAsync()
        
    def __speak(self, text):
    	# Generate speech 
        tts = gTTS(text=text, lang='en')
        # Save the speech to a file 
        filename = 'speech.mp3'
        tts.save(filename)
        # Play the speech 
        self.client.send_audio_request(filename)       
