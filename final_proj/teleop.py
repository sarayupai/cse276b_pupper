#!/usr/bin/env python3
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import select
import sys
import termios
import tty

import numpy as np
import rclpy

from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool 
import random
import time
import math 
from MangDang.mini_pupper.display import Display, BehaviorState
from resizeimage import resizeimage  # library for image resizing
from PIL import Image, ImageDraw, ImageFont # library for image manip.

MAX_WIDTH = 320   # max width of the LCD display

# Get access to the display so we can display things
disp = Display()

# Open math img 
imgLocMath = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right.jpg"
imgFileMath = Image.open(imgLocMath)

# Open world img 
imgLocWorld = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right.jpg"
imgFileWorld = Image.open(imgLocWorld)

# Open maze runner category 
imgLocMaze = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right.jpg"
imgFileMaze = Image.open(imgLocMaze)

# Open right img
imgLocRight = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right.jpg"
imgFileRight = Image.open(imgLocRight)

# Open left img
imgLocLeft = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/left.jpg"
imgFileLeft = Image.open(imgLocLeft)
		
# Open straight img
imgLocFront = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/straight.jpg"
imgFileFront = Image.open(imgLocFront)
        
# Open right img
imgLocRight = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right.jpg"
imgFileRight = Image.open(imgLocRight)
        
# Open correct img
imgLocCorrect = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/correct-face.jpg"
imgFileCorrect = Image.open(imgLocCorrect)
        
# Open incorrect img
imgLocIncorrect = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/incorrect-face.jpg"
imgFileIncorrect = Image.open(imgLocIncorrect)

# We likely also need to resize to the pupper LCD display size (320x240). 
# TODO: may need to resize to top 2/3 of screen bc of glitching 
#width_size = (MAX_WIDTH / float(imgFileRight.size[0]))
imgFileRight = resizeimage.resize_width(imgFileRight, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileLeft.size[0]))
imgFileLeft = resizeimage.resize_width(imgFileLeft, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileFront.size[0]))
imgFileFront = resizeimage.resize_width(imgFileFront, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileIncorrect.size[0]))
imgFileIncorrect = resizeimage.resize_width(imgFileIncorrect, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileCorrect.size[0]))
imgFileCorrect = resizeimage.resize_width(imgFileCorrect, MAX_WIDTH)
 
#width_size = (MAX_WIDTH / float(imgFileMath.size[0]))
imgFileMath = resizeimage.resize_width(imgFileMath, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileWorld.size[0]))
imgFileWorld = resizeimage.resize_width(imgFileWorld, MAX_WIDTH)

#width_size = (MAX_WIDTH / float(imgFileMath.size[0]))
imgFileMaze = resizeimage.resize_width(imgFileMaze, MAX_WIDTH)

newR = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/right1.png"
newL = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/left1.png"
newF = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/straight1.png"
newI = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/incorrect1.png"
newC = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/correct1.png"
newM = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/math1.png"
newW = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/world1.png"
newMR = "/home/ubuntu/ros2_ws/src/final_proj/final_proj/img/maze1.png"

# now output it (super inefficient, but it is what it is)
imgFileRight.save(newR, imgFileRight.format)
imgFileLeft.save(newL, imgFileLeft.format)
imgFileFront.save(newF, imgFileFront.format)
imgFileCorrect.save(newC, imgFileCorrect.format)
imgFileIncorrect.save(newI, imgFileIncorrect.format)
imgFileMath.save(newM, imgFileMath.format)
imgFileWorld.save(newW, imgFileWorld.format)
imgFileMaze.save(newMR, imgFileMaze.format)

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

MAX_SPEED = 0.9 
MIN_SPEED = 0.5 

class Teleop(Node):
    def __init__(self):
        super().__init__('champ_teleop')
        
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_lite_publisher = self.create_publisher(PoseLite, 'body_pose/raw', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 1)
        #self.subscription = self.create_subscription(Bool, 'enable_movement', self.poll_keys, 10)

        self.declare_parameter("gait/swing_height", 0)
        self.declare_parameter("gait/nominal_height", 0)
        self.declare_parameter("speed", 0.7)
        self.declare_parameter("turn", 1.0)
        
        self.swing_height = self.get_parameter("gait/swing_height").value
        self.nominal_height = self.get_parameter("gait/nominal_height").value

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value

        self.msg = """
--------------------
Moving around:
'↑' to move FORWARD 
'←' to turn LEFT
'↓' to turn BACK 
'→' to turn RIGHT
'X' to quit
--------------------
        """
        self.velocityBindings = {
                '8':(1,0,0,0), # front
                '4':(0,0,0,1), # left 
                '6':(0,0,0,-1), # right
                '2':(-1,0,0,0) # back
                #'5':(0,0,0,0), # stop
            }
            
        self.image = {
                '8': newF, # front
                '4': newL, # left 
                '6': newR, # right
                '2': newF, # back
                #'5': newF, # stop
                'red': newM,
                'green': newMR,
                'blue': newW
        }

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose_lite = PoseLite()
        body_pose_lite.x = 0
        body_pose_lite.y = 0
        body_pose_lite.roll = (not data.buttons[5]) *-data.axes[3] * 0.349066
        body_pose_lite.pitch = data.axes[4] * 0.174533
        body_pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332
        if data.axes[5] < 0:
            body_pose_lite.z = data.axes[5] * 0.5

        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.z = body_pose_lite.z

        quaternion = quaternion_from_euler(body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(body_pose)

    def poll_keys(self, level, first_try):
        self.settings = termios.tcgetattr(sys.stdin)
        x = 0
        y = 0
        z = 0
        th = 0
        roll = 0
        pitch = 0
        yaw = 0
        status = 0
        mode = 'continue'

        try:
            print(self.msg)
            print(self.vels( self.speed, self.turn))
            
            # TODO: add more advanced leveling
            if level == 'level1':
                interval = 50
            elif level == 'level2':
                interval = 35
                if first_try and self.speed < MAX_SPEED: 
                    self.speed += 0.1
                elif not first_try and self.speed > MIN_SPEED:
                    self.speed -= 0.1 

            end_time = time.time() + interval  
            while(time.time() < end_time):
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]
                    
                    twist = Twist()
                    twist.linear.x = x *self.speed
                    twist.linear.y = y * self.speed
                    twist.linear.z = z * self.speed
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = th * self.turn
                    self.velocity_publisher.publish(twist)
                    disp.show_image(self.image[key])
                else:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    self.velocity_publisher.publish(twist)
                    disp.show_image(newF)
                    if key == '5':
                        mode = 'quit'
                        return 

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return mode 

    def category(self, color):
        disp.show_image(self.image[color])
    
    # TODO: implement         
    def headnod(self, correct):
        body_pose = Pose()
        if correct:
            #TODO: up and down head nods causing robot to shut off 
            # reset
            disp.show_image(newC)
        else:
            disp.show_image(newI)
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def main():
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
