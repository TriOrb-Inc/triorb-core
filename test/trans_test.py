#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2023 TriOrb Co. Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import sys
import os
import logging

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from triorb_core import *

formatter = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(level=logging.DEBUG, format=formatter)


def check_data_types():
    logging.info("--- Data types ---")
    logging.info(TriOrbBaseSystem())
    logging.info(TriOrbBaseDevice())
    logging.info(TriOrbBaseSensor())
    logging.info(TriOrbBaseError())
    logging.info(TriOrbBaseState())
    logging.info(TriOrbDrive3Pose())
    logging.info(TriOrbDrive3Vector())
    logging.info("<<<")

def check_robot_functions(com1):
    import time
    vheicle = robot(com1)
    logging.info(vheicle.codes)
    #_tx = vheicle.tx(code_array=[ RobotCodes.SYSTEM_INFORMATION,
    #                            [RobotCodes.STARTUP_SUSPENSION, 0x02],
    #                            [RobotCodes.STANDARD_ACCELERATION_TIME, TriOrbDrive3Pose(1000,1000,1000)],
    #                            ])
    #logging.info(vheicle.byteList_to_string(_tx))
    #assert vheicle.byteList_to_string(_tx) == '0x00 0x01 0x00 0x00 0x01 0x03 0x02 0x05 0x03 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x0d 0x0a'

    is_start_end=1
    vel_test    =0
    pos_test    =0
    configs     =0
    info_test   =0
    get_status  =0


    if is_start_end:
        vheicle.wakeup()
        time.sleep(2)
        print()

    """
    tx = vheicle.tx(code_array=[[ RobotCodes.SYSTEM_INFORMATION, TriOrbBaseSystem()],
                               [RobotCodes.STARTUP_SUSPENSION, 0x02],
                               [RobotCodes.STANDARD_ACCELERATION_TIME, np.uint32(1000)],
                               ])
    print(vheicle.rx())
    #print(vheicle.clear_rx())
    """

    """
    tx = vheicle.tx(code_array=[
                                [ RobotCodes.MOVING_SPEED_RELATIVE, TriOrbDrive3Pose(0.3,0,0)],
                                 [RobotCodes.ACCELERATION_TIME, TriOrbDrive3Vector(2000,2000,2000)],
                               ])
    print(vheicle.rx())
    time.sleep(5)
    print( vheicle.set_vel_relative(x=-0.5, y=0.0, w=0.0) )
    time.sleep(5)
    """

    print( vheicle.set_torque(3000) )
    print( vheicle.set_torque(2000) )
    print( vheicle.set_torque(3000) )

    if configs:
        params={"acc":2000.0, "dec":2000.0, "std-vel":1.5}
        #params={"acc":1.0, "dec":1.0, "std-vel":0.5}
        vheicle.read_config()
        vheicle.write_config(params)
        vheicle.read_config()
        print()

    if vel_test:
        print( vheicle.set_vel_relative(x=-0.5, y=0.0, w=0.0) )
        print()
        time.sleep(10)
        print( vheicle.set_vel_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read only
        print()
        time.sleep(5)
        print( vheicle.set_vel_relative(x=0.5, y=0.0, w=0.0) )
        print()
        time.sleep(10)
        print( vheicle.set_vel_relative(x=0xFFFFFFFF, y=0.0, w=0.0) )#read only
        print()
        time.sleep(2.5)
        vheicle.brake()
        print()

        # not impolemented
        vheicle.set_vel_absolute(x=0.5, y=0.0, w=0.0)
        time.sleep(2.5)
        vheicle.set_vel_absolute(x=0xFFFFFFFF, y=0.0, w=0.0) #read mode
        time.sleep(2.5)
        vheicle.brake()
        print()

    if pos_test:
        vheicle.write_config()
        print( vheicle.set_pos_relative(x=-3, y=3.0, w=0.0) )
        time.sleep(1.0)
        print(vheicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
        time.sleep(5)
        vheicle.join()

        print(vheicle.set_pos_relative(x=5, y=0.0, w=0.0))
        time.sleep(1.0)
        print(vheicle.set_pos_relative(x=0XFFFFFFFF, y=0.0, w=0.0)) #read mode
        time.sleep(5)
        vheicle.join()
        
        print(vheicle.set_pos_absolute(x=-3, y=3.0, w=0.0))
        time.sleep(1.0)
        print(vheicle.set_pos_absolute(x=0xFFFFFFFF, y=0.0, w=0.0)) #read mode
        time.sleep(5)
        vheicle.join()

        print(vheicle.set_pos_absolute(x=5, y=0.0, w=0.0))
        time.sleep(1.0)
        print(vheicle.set_pos_absolute(x=0XFFFFFFFF, y=0.0, w=0.0)) #read mode
        time.sleep(5)
        vheicle.join()
        print()


    if info_test:
        print(vheicle.get_info())
        print(vheicle.get_device_status())
        print(vheicle.get_sensor_info())
        print()

    if get_status:
        print(vheicle.get_error_info())
        time.sleep(0.5)
        print(vheicle.get_operating_status())
        time.sleep(0.5)
        print(vheicle.get_voltage())
        time.sleep(0.5)
        print(vheicle.get_power())
        time.sleep(0.5)
        print()

    if is_start_end:
        vheicle.sleep()
    exit()



if __name__ == '__main__':
    #check_data_types()
    check_robot_functions("COM32")

