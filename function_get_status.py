import os
import sys
import time
#import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI
port = "192.168.1.197"

def initialize_arm(port): #connect arm
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    return "Arm initialized and connected on port " + port

def get_status(port):
    arm = XArmAPI(port)
 #create dictionary or list to mapping the code 
    return f"""
default_is_radian:{arm.default_is_radian}
version:{arm.version}
state:{arm.state}    1-in motion ; 2-sleeping ; 3-suspended ; 4-stopping
mode:{arm.mode}
error code: {arm.error_code}
warn code:{arm.warn_code}
collision sensitivity:{arm.collision_sensitivity}
world offset:{arm.world_offset}
gravity direction:{arm.gravity_direction}
============TCP============
position:{arm.position}
tcp_jerk:{arm.tcp_jerk}
tcp_load:{arm.tcp_load}
tcp_offset:{arm.tcp_offset}
tcp_speed_limit:{arm.tcp_speed_limit}
tcp_acc_limit:{arm.tcp_acc_limit}
===========JOINT===========
angles:{arm.angles}
joint_jerk:{arm.joint_jerk}
joint_speed_limit:{arm.joint_speed_limit}
joint_acc_limit:{arm.joint_acc_limit}
joints_torque:{arm.joints_torque}
"""

def string_test(a,b):
    value = a+b
    return f"""value is: {value}
value is: {value}"""


print(string_test(1,2))
print(type(string_test(1,2)))
print(get_status(port))
arm = XArmAPI(port)
print(type(arm.error_code))
