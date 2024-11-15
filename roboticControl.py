# xarm_api_interface.py
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append("C:\\Users\\23149\\Documents\\GitHub\\xArm-Python-SDK\\xarm\\x3")
from xarm.wrapper import XArmAPI
import math
from xarm.x3 import XArm, Studio

def initialize_arm(port):
    arm = XArmAPI(port)
    arm.connect()
    return "Arm initialized and connected on port " + port
