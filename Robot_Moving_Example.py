#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI("192.168.1.12")
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.move_gohome(wait=True)

speed =5
arm.set_servo_angle(angle=[90, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[90, 0, -60, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[90, -30, -60, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, -30, -60, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, 0, -60, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))


arm.move_gohome(wait=True)
speed = math.radians(5)
arm.set_servo_angle(angle=[math.radians(90), 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[math.radians(90), 0, math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[math.radians(90), math.radians(-30), math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, math.radians(-30), math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, 0, math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.set_servo_angle(angle=[0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))


arm.move_gohome(wait=True)
arm.disconnect()


#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: read force raw data and external data
"""

import sys
import time
import math
import socket
import struct
from xarm.wrapper import XArmAPI
#######################################################

if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################

arm = XArmAPI(ip, enable_report=True)
arm.motion_enable(enable=True)
arm.ft_sensor_enable(0)

arm.clean_error()
arm.clean_warn()
arm.ft_sensor_enable(1)
time.sleep(0.5)
arm.ft_sensor_set_zero()

while arm.connected and arm.error_code == 0:
    # ft_raw_force and ft_ext_force will update by reporting socket
    #print('raw_force: {}'.format(arm.ft_raw_force))
    print('exe_force (x,y,z,Rx,Ry,Rz): {}'.format(arm.ft_ext_force))

    # # get_ft_sensor_data() will get the last ext_force
    # code, ext_force = arm.get_ft_sensor_data()
    # if code == 0:
    #     print('exe_force: {}'.format(ext_force))
    time.sleep(0.2)

arm.ft_sensor_enable(0)
arm.disconnect()

