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
from client_go_pupper import MinimalClientAsync
# Packages to let us create nodes and spin them up 
import rclpy
import os
import time
from rclpy.node import Node
from mutagen.mp3 import MP3

class Audio(Node):

    def __init__(self, client):
        # initialize 
        super().__init__('audio')
        self.client = client
        
    def speak(self, text):
    	# Generate speech 
        tts = gTTS(text=text, lang='en')
        # Save the speech to a file 
        filename = 'speech.mp3'
        tts.save(filename)
        # Play the speech 
        self.client.send_audio_request('/home/ubuntu/ros2_ws/src/final_proj/final_proj/' + filename)
        # sleep for duration of question 
        audio = MP3("speech.mp3")
        time.sleep(audio.info.length + 3.0)
     
    def stop_speak(self):
    	self.client.stop_audio_request()
    	# delete the file now
    	if os.path.exists('/home/ubuntu/ros2_ws/src/final_proj/final_proj/speech.mp3'):
            os.remove('speech.mp3')
    
    def play_audio(self, filename): 
        self.client.send_audio_request('/home/ubuntu/ros2_ws/src/final_proj/final_proj/audio/' + filename)
    
    def stop_audio(self):
        self.client.stop_audio_request()
        

# To test audio functionality     
def main():
    rclpy.init()
    client = MinimalClientAsync()
    audio = Audio(client)
    
    # send commands to do the conga dance
    audio.speak("we are testing this")
    time.sleep(7.0)
    audio.stop_speak()
    
    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    # (Probably a bit redundant with other code and can be simplified. But right now it works, so ¯\_(ツ)_/¯)
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
