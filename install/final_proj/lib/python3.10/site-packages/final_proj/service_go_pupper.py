########
# Name: service_go_pupper.py
#
# Purpose: Go Pupper Service. Sample code which will send movement commands from client to pupper.
#
# Usage: After conpling and sourcing the ~/ros2_ws/install/setup.bash , launch the service like this:
#         ros2 run go_pupper_srv service
#        (See client code for how to use)
#
# Author: Prof. Riek <lriek@ucsd.edu>
#
# Acknowledgements: Used some code from ROS 2 Tutorials and MangDang's ROS git repo 
#  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
#  https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2-dev/mini_pupper_dance/mini_pupper_dance/dance_server.py
#
# Date: 30 Apr 2024
#
# Prof. Riek notes: You likely don't need to change this file for Lab 2. But maybe in your final
#                   project you may want to change some things in this.
#
########

# Import the ROS2 interface we wrote, called GoPupper. This specifies the message type.
from pupper_interfaces.srv import GoPupper

# packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node

# The Twist package is what we use to move the robot
from geometry_msgs.msg import Twist

# To let us set timers
import time   # Is on our side, yes it is. And if you don't know the song, here you go! https://www.youtube.com/watch?v=8wDUhw15E-s

####
# Name: Minimal Service
#
# Purpose: "The MinimalService class constructor initializes the node with the name minimal_service. 
# Then, it creates a service and defines the type, name, and callback.""
#
# # Prof Riek Notes: You can call this method whatever you like, this is just the modified ROS tutorial code. 
####
class MinimalService(Node):

    # Constructor
    def __init__(self):
        # Initialize the node 
        super().__init__('minimal_service')

        # Create the service, defining its type (GoPupper), name (pup_command), and callback.
        self.srv = self.create_service(GoPupper, 'pup_command', self.pup_callback)
        
        # publish twist 
        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # timer interval (need to wait between messages)
        self.interval = 0.5  # .5 seconds


    #####
    # Name: pup_callback
    # Purpose: This will receive pupper movement messages and publish accordingly.
    # Arguments: self, the request (e.g., the command from client), the response (e.g., success)
    #####
    def pup_callback(self, request, response):
        # We'll be publishing a velocity message. Calling the Twist constructor zeroes it out.  
        velocity_cmd = Twist()

        ## Debug - if you're curious what message this method got, uncomment this out
        #print("In server pup_callback, got this command: %s" % request.command)

        ## Here is a set of conditionals - move forward, move_backward, etc, and they send their
        # respective linear velocity commands accordingly. See Lab 0 / Lab 1 to learn more about this. 
        if (request.command == 'move_forward'):
            velocity_cmd.linear.x = 0.5    # .5 in the linear X direction moves us forward
            self.vel_publisher_.publish(velocity_cmd)   # publish the command
            self.get_logger().info('Publishing: "%s"' % request.command)  # Log what happened
            time.sleep(self.interval)  # Wait and make sure the robot moved

        elif (request.command == 'move_backward'):
            velocity_cmd.linear.x = -0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.command)
            time.sleep(self.interval)   

        elif (request.command == 'move_left'):
            velocity_cmd.linear.y = 0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.command)
            time.sleep(self.interval)   

        elif (request.command == 'move_right'):
            velocity_cmd.linear.y = -0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.command)
            time.sleep(self.interval)   

        elif (request.command == 'turn_left'):
            velocity_cmd.angular.z = 1.0
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.command)
            time.sleep(self.interval)  

        elif (request.command == 'turn_right'):
            velocity_cmd.angular.z = -1.0
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.command)
            time.sleep(self.interval)   
    
        elif (request.command == 'stay'):
            time.sleep(self.interval)  # do nothing

        else:
            self.get_logger().info('Invalid command: "%s"' % request.command)
            time.sleep(self.interval)  # do nothing

        # Stop the robot from moving (set everyting to zero by calling the Twist constructor)
        velocity_cmd = Twist()
        self.vel_publisher_.publish(velocity_cmd)

        # Give a response. (Probably we should set this to false if the command was invalid per above logic,
        # but this is just demo code to give you an idea).  
        response.executed = True
        return response

####
# Name: Main
# Purpose: Main functoin to set up our service
#####
def main():
    # Initialize the python client library in ROS 2
    rclpy.init()

    # Instatiate the class & create the node for the service
    minimal_service = MinimalService()

    # Spin the node - this will handle call backs
    rclpy.spin(minimal_service)

    # Destroy the node when we're done with it
    minimal_service.destroy_node()
    
    # Shutdown  
    rclpy.shutdown()


if __name__ == '__main__':
    main()