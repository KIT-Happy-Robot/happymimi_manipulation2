#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
import roslib.packages #!!!
import os
import sys
import time
import math
import numpy
import threading
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from happymimi_msgs2.srv import StrTrg
from mimi_mani_msgs2.msg import *
# !!!
motor_controller2_path = roslib.packages.get_pkg_dir('dynamixel_controller2')
sys.path.insert(0, os.path.join(motor_controller2_path, 'src/'))
from motor_controller2 import ManipulateArm
#!!!
teleop_path = roslib.packages.get_dir_pkg('mimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control2 import BaseControl

class GraspingActionServer(ManipulateArm): #!!!
  def __init__(self):
    super(GraspingActionServer, self).__init__()
    # Sub
    self.create_subscription(String, '/current_location', self.navigationPlaceCB)
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_mux/inpub/teleop', queue_size=1)
    self.navigation_place = 'Null'
    self.target_place = self.get_parameter