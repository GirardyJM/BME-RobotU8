import os
import sys
import time
#import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI
port = "192.168.1.197"
arm = XArmAPI(port)

def initialize_arm(port=port): #connect arm
    arm = XArmAPI(port)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.move_gohome(wait=True)
    return "Arm initialized and connected on port " + port

def get_status(port):
    arm = XArmAPI(port) #don't have to get this object if the object is already include 
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

# def string_test(a,b):
#     value = a+b
#     return f"""value is: {value}
# value is: {value}"""


# print(string_test(1,2))
# print(type(string_test(1,2)))
# print(get_status(port))
# arm = XArmAPI(port)
# print(type(arm.error_code))
def test(ip,speed):
    arm = XArmAPI(ip)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    arm.move_gohome(wait=True)
    arm.set_position(x=400, y=-50, z=150, roll=-180, pitch=0, yaw=0, speed=speed, is_radian=False, wait=True)

    # set mode: cartesian online trajectory planning mode
    # the running command will be interrupted when the next command is received
    arm.set_mode(7)
    arm.set_state(0)
    time.sleep(1)


    for i in range(10):
        # run on mode(7)
        # the running command will be interrupted, and run the new command
        arm.set_position(x=400, y=-150, z=150, roll=-180, pitch=0, yaw=0, speed=speed, wait=False)
        time.sleep(1)
        # the running command will be interrupted, and run the new command
        arm.set_position(x=400, y=100, z=150, roll=-180, pitch=0, yaw=0, speed=speed, wait=False)
        time.sleep(1)

    # set_mode: position mode
    arm.set_mode(0)
    arm.set_state(0)
    arm.move_gohome(wait=True)
    arm.disconnect()

#test(port,10)
def move_arm(x,y,z,roll,pitch,yaw,speed,is_radian=False,wait=False,port="192.168.1.197"):

    arm = XArmAPI(port)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    # x=300
    # y=0
    # z=150
    # roll=-180
    # pitch=0
    # yaw=0
    # speed=100
    # is_radian=False
    # wait=True
    arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=speed,is_radian=is_radian,wait=wait)
    return(list(arm.get_position()[1]))

'''#initialize_arm(port) #connect arm
#move_arm(300,0,150,-180,0,0,20,is_radian=False,wait=True,port=port)
#arm = XArmAPI(port)
#arm.set_position(x=300, y=0, z=150, roll=-3.1415926, pitch=0, yaw=0, speed=100, is_radian=True, wait=True)
move_arm(300,0,150,-180,0,0,50,is_radian=False,wait=False)
#print(arm.get_position(), arm.get_position(is_radian=True))
#print(type(arm.set_position(x=300, y=0, z=150, roll=-3.1415926, pitch=0, yaw=0, speed=100, is_radian=True, wait=True)))
#print((move_arm(300,0,150,-180,0,0,20,is_radian=False,wait=False)))
print((move_arm(300,0,150,-180,0,0,50,is_radian=False,wait=False)))'''

initialize_arm(port)
move_arm(300,0,150,-180,0,0,50,is_radian=False,wait=False)
arm.set_position(x=300, y=200, z=250, roll=-3.1415926, pitch=0, yaw=0, speed=200, is_radian=True, wait=True)
initialize_arm(port)
arm.disconnect()
