#!/usr/vin/env python3
#-*- coding: utf-8 -*-
#
#

import numpy
import math
import threading
import time
import rclpy
from rclpy.node import Node
#import rosparam # !!!!
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message --
from happymimi_msgs2.srv import StrTrg
from happymimi_recognition_msgs2.srv import PositionEstimator
from mimi_mani_msgs2.srv import ArmControl # !!!!

class MotorController(Node):
  def __init__(self):
    self.create_service(DynamixelStateList, 
                        '/dynamixel_workbench/dynamixel_state',
                        self.getMotorStateCB)
    self.motor_pub = self.create_publisher(Float64MultiArray,
                                           'dynamixel_workbench/joint_trajectory',
                                           queue_size = 10)
    self.motor_angle_pub = self.create_publisher(Float64MultiArray,
                                                 '/servo/angle_list',
                                                 queue_size = 10)
    # ServiceClient
    self.motor_client = self.create_client(DynamixelCommand,
                                           '/dynamixel_workbench/dynamixel_command')
    while not self.motor_client.wait_for_service(1.0):
      self.get_logger().warn('waiting for service... /dynamixel_workbench/dynamixel_command')
    dyna_cmd_future = self.motor_client.call_async(request)
    rclpy.spin_until_future_complete(self, dyna_cmd_future)
    
    # Motor parameters
    self.origin_angle = self.declare_parameter('/mimi_specification/Origin_Angle')
    self.gear_ratio = self.declare_parameter('/mimi_specification/Gear_Ratio')
    self.current_pose = [0]*6
    self.torque_error = [0]*6
    self.rotation_velocity = [0]*6
    #timer_period = 0.5
    self.create_timer(0.5, self.motorPub)

  def getMotorStateCB(self, state):
    for i in range(len(state.dynamixel_state)):
      self.current_pose[i] = state.dynamixel_state[i].present_position
      self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
      self.torque_error[i] = state.dynamixel_state[i].present_current # !!!
     
  # !!!
  def motorAnglePub(self, event):
    deg_origin_angle = list(map(self.stepToDeg, self.origin_angle))
    deg_current_pose = list(map(self.stepToDeg, self.current_pose))
    current_deg_list = [x-y for (x,y) in zip(deg_current_pose, deg_origin_angle)]
    current_deg_list = [round(x, 1) for x in current_deg_list]
    current_deg_list[2] *= -1
    current_deg_list[5] *= -1
    pub_deg_list = Float64MultiArray(data=current_deg_list)    
    self.motor_angle_pub.publish(pub_deg_list)
  
  # !!!  
  def degToStep(self, deg):
    return int((deg+180)/360.0*4095)
  
  def stepToDeg(self, step):
  
  def motorPub(self, joint_name, joint_angle, execute_time = 0.8):
  
  def setPosition(self, motor_id, position_value):
  
  def setCurrent(self, motor_id, current_value):
    
class JointController(MotorController):
  def __init__(self):






def main(args=None):
  rclpy.init(args=args)  # initialize node
  node = Node("motor_controller2")
  
  experiment = ManipulateArm()
  rclpy.spin(node)
  
if __name__ == '__main__':
  main()