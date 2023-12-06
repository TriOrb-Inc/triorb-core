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



def write_config(com1):
    import time
    vehicle = robot(com1)

    vehicle.wakeup()
    time.sleep(2)
    params={ "std-vel":0.45 ,"std-rot":1.0 }
    print(vehicle.read_config())
    time.sleep(2)
    print(vehicle.write_config(params))
    time.sleep(2)
    #vehicle.write_config(params)
    print(vehicle.read_config())
    time.sleep(2)
    print()



def getbutton(com1):
    import time
    vehicle = robot(com1)
    vehicle.wakeup()
    while True:
        time.sleep(1)
        print(vehicle.get_motor_status("state"))

if __name__ == '__main__':
    #check_data_types()
    #write_config("COM32")
    getbutton("COM32")

