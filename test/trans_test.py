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
#logging.basicConfig(level=logging.DEBUG, format=formatter)
logging.basicConfig(level=logging.ERROR, format=formatter)


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
    vehicle = robot(com1)
    logging.info(vehicle.codes)

    #_tx = vehicle.tx(code_array=[ RobotCodes.SYSTEM_INFORMATION,
    #                            [RobotCodes.STARTUP_SUSPENSION, 0x02],
    #                            [RobotCodes.STANDARD_ACCELERATION_TIME, TriOrbDrive3Pose(1000,1000,1000)],
    #                            ])
    #logging.info(vehicle.byteList_to_string(_tx))
    #assert vehicle.byteList_to_string(_tx) == '0x00 0x01 0x00 0x00 0x01 0x03 0x02 0x05 0x03 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x0d 0x0a'
    """
    query = []
    query.append( [RobotCodes(0x0301), 0x02] )                      # 励磁
    query.append( [RobotCodes(0x0313), TriOrbDrive3Pose(1,-1,0)] )  # 相対姿勢制御 
    query.append( [b'\x05\x03', b'\xe8\03\x00\x00']      )          # 標準加速度設定 

    vehicle.tx(code_array=query) 
    print(vehicle.rx_bytes())

    #query = []  
    #query.append([RobotCodes(0x0009), b'\x00\x01'])  # モータードライバのステータス取得(ID1) 
    #val = TriOrbBaseState(motor_id=2) 
    #query.append([b'\x09\x00', val])  # モータードライバのステータス取得(ID2) 
    #vehicle.tx(code_array=query) 
    #print(vehicle.rx())

    time.sleep(5)
    vehicle.tx( [[b'\x01\x03', b'\x01']] )                         # 無励磁
    vehicle.rx()
    exit()
    """

    vehicle.clear_rx()
    #M = np.array( [ [ 3000.,  6000, 650],
    #                [-6000,      0, 650],
    #                [ 3000., -6000, 650] ], dtype=np.float32 )
    #print( vehicle.initialize_config() )
    #exit()
    #vehicle.initialize_config()
    #exit()

    v = np.zeros((3,3), dtype=np.float32)
    #v = np.identity(3, dtype=np.float32)
    #command = [[RobotCodes.KINEMATICS_TRANS,   TriOrbDriveMatrix(v)]]
    #command = [[RobotCodes.KINEMATICS,   TriOrbDriveMatrix(v)]]
    #vehicle.tx(command)
    #print( vehicle.rx() )
    #exit()
    #command = [RobotCodes.KINEMATICS,   TriOrbDriveMatrix(M)]
    #command = [RobotCodes.KINEMATICS_TRANS,   TriOrbDriveMatrix(v)]
    #vehicle.tx(command)
    #print( vehicle.rx() )
    #exit()

    #command = []

    ss = 0
    for i in range(10000):
        st = time.time()
        vehicle.get_info()
        ss += time.time()-st
    print(ss/10000)
    exit()


    is_start_end=1
    odom        =0
    vel_test    =0
    pos_test    =0
    configs     =0
    info_test   =0
    get_status  =0

    #try:
    if True:
        if is_start_end:
            vehicle.wakeup()
            time.sleep(2)
            print()
            command = [[RobotCodes(0x1010), b"\x00"]]
            vehicle.tx(command)
            print(vehicle.rx())
            time.sleep(2)

        if odom:
            print(vehicle.set_odometry(10,5,3.14))
            print(vehicle.get_pos())

        if configs:
            #params={"acc":2000.0, "dec":2000.0, "std-vel":1.5}
            params={"acc":200.0, "dec":200.0, "std-vel":0.3, "torque":1000}
            vehicle.read_config()
            vehicle.write_config(params)
            vehicle.read_config()
            print()

        if vel_test:
            spd = 7.0
            print( vehicle.set_vel_relative(vx=0.0, vy=0.0, vw=spd) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()

            print( vehicle.set_vel_relative(vx=0.0, vy=0.0, vw=-spd) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()

            print( vehicle.set_vel_relative(vx=-spd, vy=0.0, vw=0.0, acc=100, dec=100) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()

            print( vehicle.set_vel_relative(vx= spd, vy=0.0, vw=0.0, acc=100, dec=100) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()

            print( vehicle.set_vel_relative(vx= 0.0, vy=spd, vw=0.0) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()

            print( vehicle.set_vel_relative(vx= 0.0, vy=-spd, vw=0.0) )
            print()
            time.sleep(3)
            vehicle.brake()
            vehicle.join()


        if pos_test:
            params={"acc":200, "dec":200, "std-vel":0.5}
            print( vehicle.write_config(params) )
            pos = 0.5
            time.sleep(3)

            print( vehicle.set_pos_relative(x=pos, y=pos, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=-pos, y=-pos, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()
            #"""
            print( vehicle.set_pos_relative(x=0.0, y=0, w=-pos) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=0.0, y=0.0, w=pos) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=pos, y=0, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=-pos, y=0, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=0, y=pos, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()

            print( vehicle.set_pos_relative(x=0, y=-pos, w=0.0) )
            #print(vehicle.set_pos_relative(x=0xFFFFFFFF, y=0.0, w=0.0) ) #read mode
            vehicle.join()


        if info_test:
            print(vehicle.get_info())
            print(vehicle.get_device_status())
            print(vehicle.get_sensor_info())
            print()

        if get_status:
            print(vehicle.get_motor_status())
            time.sleep(0.5)
            #print(vehicle.get_error_info())
            #time.sleep(0.5)
            #print(vehicle.get_operating_status())
            #time.sleep(0.5)
            #print(vehicle.get_voltage())
            #time.sleep(0.5)
            #print(vehicle.get_power())
            #time.sleep(0.5)
            #print()

        if is_start_end:
            vehicle.sleep()

    #except:
    #    vehicle.sleep()
    exit()



if __name__ == '__main__':
    #check_data_types()
    check_robot_functions("COM32")

