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
		self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.echo_topic, 10)
		self.subscription #this is just to remove unused variable warnings
		
		self.publisher = self.create_publisher(String, 'color_detection', 10)
		
		#CvBridge has functions that allow you to convert ROS Image type data into OpenCV images
		self.br = CvBridge()
	
		# Initialize color boundaries
		
		# Green
		self.green_rgb_lower = np.uint8([[[55, 40, 40]]]) # 40, 90, 40
		self.green_rgb_upper = np.uint8([[[75, 255, 255]]]) # 130, 255, 130
		self.green_hsv_lower = cv2.cvtColor(self.green_rgb_lower, cv2.COLOR_BGR2HSV)
		self.green_hsv_upper = cv2.cvtColor(self.green_rgb_upper, cv2.COLOR_BGR2HSV)
		# self.hsv_lower=np.matrix.flatten(cv2.cvtColor(self.green_rgb_lower, cv2.COLOR_BGR2HSV))
		# self.hsv_upper=np.matrix.flatten(cv2.cvtColor(self.green_rgb_upper, cv2.COLOR_BGR2HSV))
		
		# Red TODO
		# lower - 160, 40, 40
		# upper - 180, 255, 255
		
		# Blue TODO
		# lower - 
		# upper - 
		
		# self.rgb_lower = np.uint8([[[0, 0, 0]]])
		# self.rgb_upper = np.uint8([[[255, 255, 255]]])
		
		
	
	# Callback function to echo the video frame being received	
	def echo_topic(self, data):
		#Logging a message - helps with debugging later on
		self.get_logger().info('Receiving video frame')
		
		#Using the CvBridge function imgmsg_to_cv to convert ROS Image to OpenCV image. Now you can use this image to do other OpenCV things
		current_frame = self.br.imgmsg_to_cv2(data)
		
		# Convert the current frame from RGB to HSV
		hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
		
		# Create numpy arrays from the boundaries
		green_lower = np.array(self.green_rgb_lower,dtype=np.uint8)
		green_upper = np.array(self.green_rgb_upper,dtype=np.uint8)
		
		# Find all the pixels in the color range you want in the frame
		green_mask = cv2.inRange(hsv,green_lower,green_upper)
		
		# Mask out all other colors apart from the one you want to view
		masked_frame = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)
		
		# Check if any pixels within mask are non-zero 
		green_detected = np.any(green_mask > 30)
		
		# Publish the detection result 
		detection_msg = String()
		if green_detected:
	            detection_msg.data = 'green'
		else:
		    detection_msg.data = 'none'
		self.publisher.publish(detection_msg)
		
		#Using the imshow function to echo display the image frame currrently being published by the OAK-D
		cv2.imshow("color detected", np.hstack([current_frame, masked_frame]))

		#This shows each image frame for 1 millisecond, try playing around with different wait values to achieve the video framerate you want!
		cv2.waitKey(1)
		

# Main function 		
def main(args=None):
	# Initializing rclpy (ROS Client Library for Python)
	rclpy.init(args=args)
	
	#Create an object of the echo_camera class
	echo_obj = echo_camera()
	
	#Keep going till termination
	rclpy.spin(echo_obj)
	
	#Destroy node when done 
	echo_obj.destroy_node()
	
	#Shutdown rclpy
	rclpy.shutdown()
	
if __name__ == "__main__":
    main()
	
    
