#!/usr/vin/env python3
#-*- coding: utf-8 -*-
#
#

import numpy as np
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
    

if __name == '__main__':
  # !!!!
  # initialize node
  rclpy.init()
  node = Node("motor_controller2")
  

  experiment = ManipulateArm()