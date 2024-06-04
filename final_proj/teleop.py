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
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)
        
        self.swing_height = self.get_parameter("gait/swing_height").value
        self.nominal_height = self.get_parameter("gait/nominal_height").value

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value

        self.msg = """
--------------------
Moving around:
'w' to move FORWARD 
'a' to turn LEFT
's' to turn BACK
'd' to turn RIGHT
'q' to quit
--------------------
        """
        self.velocityBindings = {
                'w':(1,0,0,0), # front
                'a':(0,0,0,1), # left 
                'd':(0,0,0,-1), # right
                's':(-1,0,0,0), # back
                'q':(0,0,0,0), # stop
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

    def poll_keys(self):
        self.settings = termios.tcgetattr(sys.stdin)
        x = 0
        y = 0
        z = 0
        th = 0
        roll = 0
        pitch = 0
        yaw = 0
        status = 0
        cmd_attempts = 0

        try:
            print(self.msg)
            print(self.vels( self.speed, self.turn))

            end_time = time.time() + 5 
            while(time.time() < end_time):
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]
                    
                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x *self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist)

                    cmd_attempts += 1

                else:
                    cmd_attempts = 0
                    #if (key == '\x03'):
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    self.velocity_publisher.publish(twist)
            # not sure if this is needed
            '''else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self.velocity_publisher.publish(twist)'''

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
            
    def headnod(self, correct):
        body_pose = Pose()
        if correct:
            #TODO: up and down head nods causing robot to shut off 
            # reset
            quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
            quaternion = quaternion_from_euler(0.0, -0.3, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
            quaternion = quaternion_from_euler(0.0, 0.3, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
            # reset
            quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
        
            
        else:
        
            # reset
            quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
        
            quaternion = quaternion_from_euler(0.0, 0.0, 0.3)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
            quaternion = quaternion_from_euler(0.0, 0.0, -0.3)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
            
            # reset
            quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
            body_pose.orientation.x = quaternion[0]
            body_pose.orientation.y = quaternion[1]
            body_pose.orientation.z = quaternion[2]
            body_pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(body_pose)
            time.sleep(1.0)
        
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
