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

import logging
logger = logging.getLogger(__name__)
from enum import Enum
import numpy as np
import struct
import serial
import serial.tools.list_ports

from .core_types import *

UART_BAUDRATE = 115200
UART_BYTESIZE = serial.EIGHTBITS
UART_PARITY = serial.PARITY_NONE
UART_STOPBITS = serial.STOPBITS_ONE
UART_FLOW = False
UART_ENDIAN = 'little'
UART_TIMEOUT = 0.1

class RobotCodes(Enum):
    SYSTEM_INFORMATION = 0x0001
    DEVICE_STATUS = 0x0003
    SENSOR_INFORMATION = 0x0005
    ERROR_INFORMATION = 0x0007
    OPERATING_STATUS = 0x0009
    POWER_SUPPLY_VOLTAGE = 0x0109
    DRIVING_POWER = 0x010B
    ERROR_RESET = 0x0201
    ORIGIN_RESET = 0x0203
    STARTUP_SUSPENSION = 0x0301
    OPERATING_MODE = 0x0303
    STANDARD_ACCELERATION_TIME = 0x0305
    STANDARD_DECELERATION_TIME = 0x0307
    STANDARD_HORIZONTAL_SPEED = 0x0309
    STANDARD_ROTATION_SPEED = 0x030B
    MOVING_SPEED_ABSOLUTE = 0x030D
    MOVING_SPEED_RELATIVE = 0x030F
    TARGET_POSITION_ABSOLUTE = 0x0311
    TARGET_POSITION_RELATIVE = 0x0313
    ACCELERATION_TIME = 0x0315
    DECELERATION_TIME = 0x0317
    DRIVING_TORQUE = 0x0319

class RobotValues(Enum):
    ROBOT_SUSPENSION = 0x01 # 停止（励磁OFF）
    ROBOT_STARTUP = 0x02 # 起動（励磁ON）

RobotValueTypes = {
    RobotCodes.SYSTEM_INFORMATION: TriOrbBaseSystem,
    RobotCodes.DEVICE_STATUS: TriOrbBaseState,
    RobotCodes.SENSOR_INFORMATION: TriOrbBaseSensor,
    RobotCodes.ERROR_INFORMATION: TriOrbBaseError,
    RobotCodes.OPERATING_STATUS: TriOrbBaseState,
    RobotCodes.POWER_SUPPLY_VOLTAGE: np.float32,
    RobotCodes.DRIVING_POWER: np.float32,
    RobotCodes.ERROR_RESET: np.uint8,
    RobotCodes.ORIGIN_RESET: np.uint8,
    RobotCodes.STARTUP_SUSPENSION: np.uint8,
    RobotCodes.OPERATING_MODE: np.uint8,
    RobotCodes.STANDARD_ACCELERATION_TIME: np.uint32,
    RobotCodes.STANDARD_DECELERATION_TIME: np.uint32,
    RobotCodes.STANDARD_HORIZONTAL_SPEED: np.float32,
    RobotCodes.STANDARD_ROTATION_SPEED: np.float32,
    RobotCodes.MOVING_SPEED_ABSOLUTE: TriOrbDrive3Pose,
    RobotCodes.MOVING_SPEED_RELATIVE: TriOrbDrive3Pose,
    RobotCodes.TARGET_POSITION_ABSOLUTE: TriOrbDrive3Pose,
    RobotCodes.TARGET_POSITION_RELATIVE: TriOrbDrive3Pose,
    RobotCodes.ACCELERATION_TIME: TriOrbDrive3Vector,
    RobotCodes.DECELERATION_TIME: TriOrbDrive3Vector,
    RobotCodes.DRIVING_TORQUE: np.uint16,
}

class robot:
    def __init__(self, port=None):
        if port is None:
            port = self.find_port()
        if port is None:
            raise Exception("Please set UART port path/name")
        self._uart = serial.Serial(
                                    port=port,
                                    baudrate=UART_BAUDRATE,
                                    bytesize=UART_BYTESIZE,
                                    parity=UART_PARITY,
                                    stopbits=UART_STOPBITS,
                                    timeout=UART_TIMEOUT,
                                    write_timeout=UART_TIMEOUT,
                                    rtscts=UART_FLOW,
                                   )
        self._expected_response_values = []
        pass
    
    @property
    def codes(self):
        return RobotCodes
    
    @staticmethod
    def find_port():
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            return p.device
        return None


    @staticmethod
    def to_bytes(val):
        if isinstance(val, RobotCodes):
            return val.value.to_bytes(2, UART_ENDIAN)
        if isinstance(val, RobotValues):
            return val.value.to_bytes(1, UART_ENDIAN)
        if isinstance(val, int):
            return val.to_bytes(1, UART_ENDIAN)
        if isinstance(val, TriOrbDrive3Pose):
            return val.to_bytes()
        if isinstance(val, TriOrbDrive3Vector):
            return val.to_bytes()
        """
        if isinstance(val, np.float32):
            return struct.pack('<f', val)
        if isinstance(val, np.uint32):
            return val.to_bytes(4, UART_ENDIAN)
        if isinstance(val, np.uint16):
            return val.to_bytes(2, UART_ENDIAN)
        if isinstance(val, np.uint8):
            return val.to_bytes(1, UART_ENDIAN)
        """
        logger.error(type(val))
        raise Exception("Unknown type")
    
    @staticmethod
    def byteList_to_string(arr):
        return ' '.join(['0x{:02x}'.format(_b) for _b in arr])

    def tx(self, code_array=[]):
        if not isinstance(code_array, list):
            logger.warning("Please provide the code_array in list format. For instance, it should be something like 'code_array = [RobotCodes.SYSTEM_INFORMATION, [RobotCodes.STARTUP_SUSPENSION, 0x02:],]'.")
        
        self._expected_response_values = []
        _tx_bytes = [0x00]
        for code_value in code_array:
            if not isinstance(code_value, list):
                logger.debug("Since it's only a communication code, I will add value=0x00.")
                code_value = [code_value, 0x00]
            _code, _value = code_value
            _code_bytes = self.to_bytes(_code)
            _value_bytes = self.to_bytes(_value)
            _tx_bytes.extend(_code_bytes)
            _tx_bytes.extend(_value_bytes)
            self._expected_response_values.append(_code)
        _tx_bytes.extend([0x0d, 0x0a])
        _send_binary = bytes(_tx_bytes)
        self._uart.write(_send_binary)
        logger.debug(_send_binary)
        return _tx_bytes

    def rx(self):
        #self._expected_response_values = []
        data = self._uart.readline()
        return self._expected_response_values

    def wakeup(self):
        logger.debug("Wakeup")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.STARTUP_SUSPENSION, RobotValues.ROBOT_STARTUP]])))
        return self.rx()

    def sleep(self):
        logger.debug("Sleep")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.STARTUP_SUSPENSION, RobotValues.ROBOT_SUSPENSION]])))
        return self.rx()

    def join(self):
        logger.debug("join")

    def brake(self):
        logger.debug("brake")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_ABSOLUTE](0,0,0)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.MOVING_SPEED_ABSOLUTE, td3p ]])))
        return self.rx()

    def get_pos(self): ## how to get?
        logger.debug("get_pos")
        #logger.debug(self.byteList_to_string(self.tx([[RobotCodes., 0x00]])))
        return self.rx()

    def read_config(self, params=["acc", "dec", "std-vel"]):
        logger.debug("read_config")
        if isinstance(params, str):
            params = [params]
        elif not isinstance(params, list):
            logger.warning("Please provide the params in list format.")
            
        dicts = {p:0x6FFFFFFF for p in params} #irregular value for read mode
        return self.write_config(dicts)

    def write_config(self, params={"acc":1.0, "dec":1.0, "std-vel":0.5}):
        logger.debug("write_config")
        if not isinstance(params, dict):
            logger.warning("Please provide the params in dict format.")

        command = []
        for k,v in params.items():
            if k == "acc":
                command.append( [RobotCodes.STANDARD_ACCELERATION_TIME, RobotValueTypes[RobotCodes.STANDARD_ACCELERATION_TIME](v*1000)] )
            elif k == "dec":
                command.append( [RobotCodes.STANDARD_DECELERATION_TIME, RobotValueTypes[RobotCodes.STANDARD_DECELERATION_TIME](v*1000)] ) 
            elif k == "std-vel":
                command.append( [RobotCodes.STANDARD_HORIZONTAL_SPEED, RobotValueTypes[RobotCodes.STANDARD_HORIZONTAL_SPEED](v)] ) 
                command.append( [RobotCodes.STANDARD_ROTATION_SPEED,   RobotValueTypes[RobotCodes.STANDARD_ROTATION_SPEED](v)]   ) 
            else:
                print(k,"is not configure value.")
                continue
        if len(command)>0:
            logger.debug(self.byteList_to_string(self.tx( command )))
            return self.rx()
        else:
            return False

    def set_pos_absolute(self, x, y, w):
        logger.debug("set_pos_absolute")
        td3p = RobotValueTypes[RobotCodes.TARGET_POSITION_ABSOLUTE](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[ RobotCodes.TARGET_POSITION_ABSOLUTE, td3p ]])))
        return self.rx()

    def set_pos_relative(self, x, y, w):
        logger.debug("set_pos_relative")
        td3p = RobotValueTypes[RobotCodes.TARGET_POSITION_RELATIVE](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[ RobotCodes.TARGET_POSITION_RELATIVE, td3p ]])))
        return self.rx()
    
    def set_vel_absolute(self, x, y, w):
        logger.debug("set_vel_absolute")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_ABSOLUTE](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.MOVING_SPEED_ABSOLUTE, td3p ]])))
        return self.rx()

    def set_vel_relative(self, x, y, w):
        logger.debug("set_vel_relative")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_RELATIVE](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.MOVING_SPEED_RELATIVE, td3p ]])))
        return self.rx()
    



    def get_info(self):
        logger.debug("get_info")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.SYSTEM_INFORMATION, 0x00]])))
        return self.rx()

    def get_device_status(self):
        logger.debug("get_device_status")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.DEVICE_STATUS, 0x00]])))
        return self.rx()

    def get_sensor_info(self):
        logger.debug("get_sensor_info")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.SENSOR_INFORMATION, 0x00]])))
        return self.rx()

    def get_error_info(self):
        logger.debug("get_error_info")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.ERROR_INFORMATION, 0x00]])))
        return self.rx()

    def get_operating_status(self):
        logger.debug("get_operating_status")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.OPERATING_STATUS, 0x00]])))
        return self.rx()

    def get_voltage(self):
        logger.debug("get_voltage")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.POWER_SUPPLY_VOLTAGE, 0x00]])))
        return self.rx()

    def get_power(self):
        logger.debug("get_power")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.DRIVING_POWER, 0x00]])))
        return self.rx()

    def reset_error(self):
        logger.debug("reset_error")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.ERROR_RESET, 0x01]])))

    def reset_origin(self):
        logger.debug("reset_origin")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.ORIGIN_RESET, 0x01]])))


    def operating_mode(self, param=0x03):
        logger.debug("operating_mode")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.OPERATING_MODE, param]])))
        return self.rx()

    def set_acceleration_time(self, x,y,w): # each acc time should be same value?
        logger.debug("set_acceleration_time")
        td3p = RobotValueTypes[RobotCodes.ACCELERATION_TIME](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.ACCELERATION_TIME, td3p ]])))
        return self.rx()

    def set_deceleration_time(self, x,y,w): # each dec time should be same value?
        logger.debug("set_deceleration_time")
        td3p = RobotValueTypes[RobotCodes.DECELERATION_TIME](x,y,w)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.DECELERATION_TIME, td3p ]])))
        return self.rx()

    def set_torque(self, param):
        logger.debug("set_torque")
        val = RobotValueTypes[RobotCodes.DRIVING_TORQUE](param*1000)
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.DRIVING_TORQUE, val ]])))
        return self.rx()

