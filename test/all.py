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

def check_robot_functions():
    vheicle = robot('COM9')
    logging.info(vheicle.codes)
    _tx = vheicle.tx(code_array=[ RobotCodes.SYSTEM_INFORMATION,
                                [RobotCodes.STARTUP_SUSPENSION, 0x02],
                                [RobotCodes.STANDARD_ACCELERATION_TIME, TriOrbDrive3Pose(1000,1000,1000)],
                                ])
    logging.info(vheicle.byteList_to_string(_tx))
    assert vheicle.byteList_to_string(_tx) == '0x00 0x01 0x00 0x00 0x01 0x03 0x02 0x05 0x03 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x00 0x00 0x7a 0x44 0x0d 0x0a'
    vheicle.wakeup()
    vheicle.sleep()
    vheicle.join()
    vheicle.brake()
    vheicle.get_pos()
    vheicle.read_config()
    vheicle.write_config()
    vheicle.set_pos_absolute(x=0.0, y=0.0, w=0.0)
    vheicle.set_pos_relative(x=0.0, y=0.0, w=0.0)
    vheicle.set_vel_absolute(x=0.0, y=0.0, w=0.0)
    vheicle.set_vel_relative(x=0.0, y=0.0, w=0.0)


if __name__ == '__main__':
    check_data_types()
    check_robot_functions()
