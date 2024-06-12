########
# Name: audio.py
# Student: Prisha Anand, Advika Sonti, Sarayu Pai
# Final Project: The Maze Pupper 
# 
# Description: Provides text to speech audio support, as well as support for 
# playing music snippets. Utilizes Google Text to Speech library in order to 
# translate text into an audio file, and utilizes the mini_pupper_music 
# audio functionality to play and stop the audio files.
#
# Date: 23 May 2024
# 
# How to Use: 
# ros2 launch mini_pupper_music music.launch.py
#####################

from gtts import gTTS 
from client_go_pupper import MinimalClientAsync
import rclpy
import os
import time
from rclpy.node import Node
from mutagen.mp3 import MP3

class Audio(Node):

    ####
    # Name: __init__
    # Purpose: Intiliaze an audio client.
    # @input: client, the audio client 
    ####
    def __init__(self, client):
        # initialize 
        super().__init__('audio')
        self.client = client
    
    ####
    # Name: speak
    # Purpose: Generate and play a speech audio file from the given text
    # @input: text (string) - The text string to be converted to speech
    ####
    def speak(self, text):
    	# Generate speech 
        tts = gTTS(text=text, lang='en')
        # Save the speech to a file 
        filename = 'speech.mp3'
        tts.save(filename)
        # Play the speech 
        self.client.send_audio_request('/home/ubuntu/ros2_ws/src/final_proj/final_proj/' + filename)
        # Sleep for duration of question 
        audio = MP3("speech.mp3")
        time.sleep(audio.info.length + 3.0)
    
    ####
    # Name: stop_speak
    # Purpose: Stop the currently playing audio and delete the audio file.    
    ####
    def stop_speak(self):
    	self.client.stop_audio_request()
    	# delete the file for name reuse
        if os.path.exists('/home/ubuntu/ros2_ws/src/final_proj/final_proj/speech.mp3'):
            os.remove('speech.mp3')
    
    ####
    # Name: play_audio
    # Purpose: Play an audio file specified by the filename
    # @input: filename (string) - The name of the audio file to be played
    ####
    def play_audio(self, filename): 
        self.client.send_audio_request('/home/ubuntu/ros2_ws/src/final_proj/final_proj/audio/' + filename)
    
    ####
    # Name: stop_audio
    # Purpose: Stop the audio that is currently playing    
    ####
    def stop_audio(self):
        self.client.stop_audio_request()
        

####
# Name: main
# Purpose: Initialize the ROS2 client and audio object, perform a speech test, and handle the client node lifecycle.     
####  
def main():
    rclpy.init()
    client = MinimalClientAsync()
    audio = Audio(client)
    
    # Test audio in isolation
    audio.speak("we are testing this")
    time.sleep(7.0)
    audio.stop_speak()
    
    # Spin up client node
    while rclpy.ok():
        rclpy.spin_once(audio)
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
