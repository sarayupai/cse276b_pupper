########
# Name: color_detection.py
#
# Purpose: 
#
# Author: Prisha Anand, Advika Sonti, Sarayu Pai 
#
# Date: 29 May 2024 ?? 
#
#####################

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String 
import cv2
from cv_bridge import CvBridge
import time
import random

###
# Method: 
# Purpose: 
#
######

# Creating a class for the echo camera node. Note that this class inherits from the Node class.
class echo_camera(Node):
	def __init__(self):
		#Initializing a node with the name 'echo_camera'
		super().__init__('echo_camera')
		
		#Subscribing to the /oak/rgb/image_raw topic that carries data of Image type
		# self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.echo_topic, 10)
		# self.subscription #this is just to remove unused variable warnings
		
		self.publisher = self.create_publisher(String, 'timer_control', 10)

	def time_loop(self):
		while(True):
		    self.get_logger().info('hopped in the time loop')
		    
		    # continually publish to move 
		    end_time = time.time() + 20 # 20 sec for now
		    detection_msg = String()
		    detection_msg.data = 'none'
		    while time.time() < end_time:
		        self.publisher.publish(detection_msg)
		 
		    # grab rand color
		    rand_color = random.randint(1,3)
		    self.get_logger().info('rand color is: ')
		    print(rand_color)
		    if rand_color == 1:
		        detection_msg.data = 'red'
		    elif rand_color == 2:
		        detection_msg.data = 'green'
		    else:
		        detection_msg.data = 'blue'
		    self.publisher.publish(detection_msg)
		    self.get_logger().info('iteration complete')	

# Main function 		
def main():
	# Initializing rclpy (ROS Client Library for Python)
	rclpy.init()
	
	#Create an object of the echo_camera class
	echo_obj = echo_camera()
	echo_obj.time_loop()

	#Destroy node when done 
	echo_obj.destroy_node()
	
	#Shutdown rclpy
	rclpy.shutdown()
	
if __name__ == "__main__":
    main()
	
    
