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
        self._expected_response_values = []

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

    def get_pos(self):
        logger.debug("get_pos")

    def read_config(self):
        logger.debug("read_config")

    def write_config(self):
        logger.debug("write_config")

    def set_pos_absolute(self, x, y, w):
        logger.debug("set_pos_absolute")

    def set_pos_relative(self, x, y, w):
        logger.debug("set_pos_relative")
    
    def set_vel_absolute(self, x, y, w):
        logger.debug("set_vel_absolute")

    def set_vel_relative(self, x, y, w):
        logger.debug("set_vel_relative")
    