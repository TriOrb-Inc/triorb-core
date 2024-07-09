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

from .alarms import get_alarm_name
from .core_types import *
import time
import serial.tools.list_ports
import serial
import struct
import numpy as np
from enum import Enum
import logging
logger = logging.getLogger(__name__)


UART_BAUDRATE = 115200
UART_BYTESIZE = serial.EIGHTBITS
UART_PARITY = serial.PARITY_NONE
UART_STOPBITS = serial.STOPBITS_ONE
UART_FLOW = False
UART_ENDIAN = 'little'
UART_TIMEOUT = 0.1

DRIVE_MOTOR_LOCAL_IDS = [1, 2, 3]
LIFTER_MOTOR_LOCAL_IDS = [4, 5, 6, 7]
ALL_MOTOR_LOCAL_IDS = DRIVE_MOTOR_LOCAL_IDS  # + LIFTER_MOTOR_LOCAL_IDS


class RobotCodes(Enum):
    SYSTEM_INFORMATION = 0x0001
    DEVICE_STATUS = 0x0003
    SENSOR_INFORMATION = 0x0005
    ERROR_INFORMATION = 0x0007
    OPERATING_STATUS = 0x0009
    POWER_SUPPLY_VOLTAGE = 0x0109
    DRIVING_POWER = 0x010B
    GET_POSE = 0x010D
    ERROR_RESET = 0x0201
    ORIGIN_RESET = 0x0203
    SET_POSE = 0x0205
    SET_USS_VALUES = 0x0207
    SET_MOTOR_PARAMS = 0x0209
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
    INITIALIZE_CONFIG = 0x031B
    POSITION_DRIVE_STD_SPEED = 0x031D
    POSITION_DRIVE_ROT_SPEED = 0x031F
    MOVING_DRIVE_LIFE_TIME = 0x0323
    AEB_MODE = 0x0325
    DRIVE_MODE = 0x0401

    KINEMATICS = 0xFF02
    KINEMATICS_TRANS = 0xFF04


class RobotValues(Enum):
    ROBOT_SUSPENSION = 0x01  # 停止（励磁OFF）
    ROBOT_STARTUP = 0x02  # 起動（励磁ON）


RobotValueTypes = {
    RobotCodes.SYSTEM_INFORMATION: TriOrbBaseSystem,
    RobotCodes.DEVICE_STATUS: TriOrbBaseDevice,
    RobotCodes.SENSOR_INFORMATION: TriOrbBaseSensor,
    RobotCodes.ERROR_INFORMATION: TriOrbBaseError,
    RobotCodes.OPERATING_STATUS: TriOrbBaseState,
    RobotCodes.POWER_SUPPLY_VOLTAGE: np.float32,
    RobotCodes.DRIVING_POWER: np.float32,
    RobotCodes.GET_POSE: TriOrbDrive3Pose,
    RobotCodes.ERROR_RESET: np.uint8,
    RobotCodes.ORIGIN_RESET: np.uint8,
    RobotCodes.SET_POSE: TriOrbDrive3Pose,
    RobotCodes.SET_USS_VALUES: TriOrbDriveUSS,
    RobotCodes.SET_MOTOR_PARAMS: TriOrbMotorParams,
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
    RobotCodes.POSITION_DRIVE_STD_SPEED: np.float32,
    RobotCodes.POSITION_DRIVE_ROT_SPEED: np.float32,
    RobotCodes.MOVING_DRIVE_LIFE_TIME: np.uint32,
    RobotCodes.AEB_MODE: np.uint8,
    RobotCodes.DRIVE_MODE: np.uint8,

    RobotCodes.KINEMATICS: TriOrbDriveMatrix,
    RobotCodes.KINEMATICS_TRANS: TriOrbDriveMatrix,
}


class robot:
    def __init__(self, port=None, node=None):
        self.node = node
        self.version = "1.0.1" # AEB, LIFETIME追加後から入れた
        if port is None:
            port = self.find_port()
        if port is None:
            raise Exception("Please set UART port path/name")
        print(port)
        self._expected_response_values = []
        self._expected_response_size = []
        self._uart = None
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


    def _print_info(self, *args, **kwargs):
        if self.node is not None:
            self.node.get_logger().info(*args, **kwargs)
        else:
            logger.info(*args, **kwargs)

    def _print_warning(self, *args, **kwargs):
        if self.node is not None:
            self.node.get_logger().warning(*args, **kwargs)
        else:
            logger.warning(*args, **kwargs)

    def _print_error(self, *args, **kwargs):
        if self.node is not None:
            self.node.get_logger().error(*args, **kwargs)
        else:
            logger.error(*args, **kwargs)

    @property
    def codes(self):
        return RobotCodes

    def find_port(self):
        while 1:
            ports = [dev for dev in serial.tools.list_ports.comports()
                     if "TriOrb CDC" in dev.description]
            for p in ports:
                return p.device
            self._print_info("waiting triorb pico..")
            time.sleep(1)
        return None
        # ports = list(serial.tools.list_ports.comports())
        # for p in ports:
        #    return p.device
        # return None

    @staticmethod
    def to_bytes(val):
        if isinstance(val, RobotCodes):
            return val.value.to_bytes(2, UART_ENDIAN)
        if isinstance(val, RobotValues):
            return val.value.to_bytes(1, UART_ENDIAN)
        if isinstance(val, TriOrbDriveUSS):
            return val.to_bytes()
        if isinstance(val, TriOrbDrive3Pose):
            return val.to_bytes()
        if isinstance(val, TriOrbDrive3Vector):
            return val.to_bytes()
        if isinstance(val, TriOrbDriveMatrix):
            return val.to_bytes()
        if isinstance(val, TriOrbBaseSystem):
            return val.to_bytes()
        if isinstance(val, TriOrbBaseDevice):
            return val.to_bytes()
        if isinstance(val, TriOrbBaseSensor):
            return val.to_bytes()
        if isinstance(val, TriOrbBaseError):
            return val.to_bytes()
        if isinstance(val, TriOrbBaseState):
            return val.to_bytes()
        if isinstance(val, int):
            return val.to_bytes(1, UART_ENDIAN)
        if isinstance(val, np.uint32):
            return int(val).to_bytes(4, UART_ENDIAN)
        if isinstance(val, np.uint16):
            return int(val).to_bytes(2, UART_ENDIAN)
        if isinstance(val, np.float32):
            return struct.pack('<f', val)
        # if isinstance(val, np.uint16):
        #    return val.to_bytes(2, UART_ENDIAN)
        # if isinstance(val, np.uint8):
        #    return val.to_bytes(1, UART_ENDIAN)
        self._print_error(type(val))
        raise Exception("Unknown type")

    @staticmethod
    def from_bytes(val, code):
        if code == RobotCodes.KINEMATICS or code == RobotCodes.KINEMATICS_TRANS:
            dtype = RobotValueTypes[code](np.zeros((3, 3), dtype=np.float32))
            dtype.from_bytes(val)
            return dtype

        dtype = RobotValueTypes[code]()
        if isinstance(dtype, RobotCodes):
            return val.value.from_bytes(2, UART_ENDIAN)
        if isinstance(dtype, RobotValues):
            return val.value.from_bytes(1, UART_ENDIAN)
        if isinstance(dtype, TriOrbDriveUSS):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbDrive3Pose):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbDrive3Vector):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbBaseSystem):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbBaseDevice):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbBaseSensor):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbBaseError):
            dtype.from_bytes(val)
            return dtype
        if isinstance(dtype, TriOrbBaseState):
            dtype.from_bytes(val, True)
            return dtype
        if isinstance(dtype, int):
            return struct.unpack("<i", val)[0]
        if isinstance(dtype, np.uint32):
            return struct.unpack('<I', val)[0]
        if isinstance(dtype, np.uint16):
            return struct.unpack('<H', val)[0]
        if isinstance(dtype, np.float32):
            return struct.unpack('<f', val)[0]
        if isinstance(dtype, np.uint8):
            return val[0]
        # if isinstance(val, np.uint16):
        #    return val.to_bytes(2, UART_ENDIAN)
        # if isinstance(val, np.uint8):
        #    return val.to_bytes(1, UART_ENDIAN)
        self._print_error(type(val))
        raise Exception("Unknown type")

    @staticmethod
    def byteList_to_string(arr):
        return ' '.join(['0x{:02x}'.format(_b) for _b in arr])

    def tx(self, code_array=[]):
        if self._uart is None:
            return []
        if not isinstance(code_array, list):
            self._print_warning(
                "Please provide the code_array in list format. For instance, it should be something like 'code_array = [RobotCodes.SYSTEM_INFORMATION, [RobotCodes.STARTUP_SUSPENSION, 0x02:],]'.")

        self._expected_response_values = []
        self._expected_response_size = []
        _tx_bytes = [0x00]
        for code_value in code_array:
            if not isinstance(code_value, list):
                logger.debug(
                    "Since it's only a communication code, I will add value=0x00.")
                code_value = [code_value, 0x00]
            _code, _value = code_value
            _code_bytes = _code if isinstance(
                _code, bytes) else self.to_bytes(_code)
            _value_bytes = _value if isinstance(
                _value, bytes) else self.to_bytes(_value)
            _tx_bytes.extend(_code_bytes)
            _tx_bytes.extend(_value_bytes)
            self._expected_response_values.append(_code)
            self._expected_response_size.append(len(_value_bytes))
        _tx_bytes.extend([0x0d, 0x0a])
        _send_binary = bytes(_tx_bytes)

        self._uart.write(_send_binary)
        logger.debug("Send: {}".format(_send_binary))
        # print(_send_binary)
        return _tx_bytes

    def rx(self):
        if self._uart is None:
            return 0
        expected_buf_length = sum(self._expected_response_size) + \
            len(self._expected_response_size)*2 + len([0x00, 0x0d, 0x0a])
        # return expected_buf_length

        buf = b""
        timeout_count = 0
        while (True):
            # buf += self._uart.readline() # 0x0aまで読み込み
            buf += self._uart.read()
            if len(buf) < 2:
                print(".", end="")
                timeout_count += 1
                if timeout_count > 20:
                    self._uart.reset_output_buffer()
                    self._uart.reset_input_buffer()
                    return 0
                continue
            if len(buf) < expected_buf_length:
                continue
            if buf[-2] == 0x0d:  # たまたま改行コードと同じ値が送られた場合を防ぐ. ただし, たまたま0x0d0aとなる値が送られてきた場合はどうしようもない
                break

        if len(buf) == 3:
            print("[ERROR] Send wrong packet")
            return buf

        with_not_readable_code = (
            expected_buf_length != len(buf))  # 期待通りの長さのコードが帰ってきているか

        logger.debug("Response: {}".format(buf))
        # print("Res: {}".format(buf) )
        values = []

        code_index = 0
        i = 0
        for _ in range(len(buf)):
            code = self._expected_response_values[code_index]
            if isinstance(code, bytes):
                c = struct.unpack("<H", code)[0]
                code = RobotCodes(c)
            clen = self._expected_response_size[code_index]
            if i > len(buf):
                break

            if code.value == int.from_bytes(buf[i:i+2], UART_ENDIAN):
                if with_not_readable_code:
                    next_bytes = int.from_bytes(buf[i+2:i+4], UART_ENDIAN)
                    if next_bytes == 0x0a0d:
                        break
                    if code_index+1 < len(self._expected_response_values):
                        if next_bytes == self._expected_response_values[code_index+1].value:
                            code_index += 1
                            i += 2
                            continue

                values.append(self.from_bytes(buf[i+2:i+2+clen], code))
                code_index += 1
                i += clen
                if len(self._expected_response_values) == code_index:
                    break
            i += 1
        # print(values)
        print("\r", end="")
        return values

    def rx_bytes(self):
        buf = b""
        while (True):
            buf += self._uart.readline()  # 0x0aまで読み込み
            if len(buf) < 2:
                print(".", end="")
                continue
            if buf[-2] == 0x0d:  # たまたま改行コードと同じ値が送られた場合を防ぐ. ただし, たまたま0x0d0aとなる値が送られてきた場合はどうしようもない
                break
        return buf

    def clear_rx(self):
        msg = "clear "
        if self._uart.in_waiting > 0:
            self._uart.reset_input_buffer()
            msg += "input"
        if self._uart.out_waiting > 0:
            self._uart.reset_output_buffer()
            msg += "output"

        # while(True):
        #    buf = self._uart.read(1)
        #    if buf==b"":
        #        break
        return msg

    def wakeup(self):
        logger.debug("Wakeup")
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.STARTUP_SUSPENSION, RobotValues.ROBOT_STARTUP]])))
        return self.rx()

    def sleep(self):
        logger.debug("Sleep")
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.STARTUP_SUSPENSION, RobotValues.ROBOT_SUSPENSION]])))
        return self.rx()

    def join(self):
        logger.debug("join")
        time.sleep(3.0)  # 早くに送信するとモーターのmoveフラグが立つ前のクエリが帰ってくるのでsleep
        while (True):  # 定期的に状態取得用のクエリを送る
            # data = self.get_operating_status(_id=DRIVE_MOTOR_LOCAL_IDS)
            data = self.get_motor_status(
                params=["state"], _id=DRIVE_MOTOR_LOCAL_IDS)
            # 全てのmoveが0になるまでループ. 本来ならin_posを使いたいが, 時間経過でoffになるので信頼性が低い
            if (data[0].in_pos and data[1].in_pos and data[2].in_pos):
                break
            if data[0].success:
                if not (data[0].move or data[1].move or data[2].move):
                    break
            time.sleep(1.0)
        print("move end")

    def brake(self):
        logger.debug("brake")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_RELATIVE](0, 0, 0)
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.MOVING_SPEED_RELATIVE, td3p]])))
        return self.rx()

    def get_pos(self):  # how to get? not implemented
        logger.debug("get_pos")
        val = RobotValueTypes[RobotCodes.GET_POSE]()
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.GET_POSE, val]])))
        return self.rx()

    def read_config(self, params=["acc", "dec", "std-vel", "torque"]):
        logger.debug("read_config")
        if isinstance(params, str):
            params = [params]
        elif not isinstance(params, list):
            self._print_warning("Please provide the params in list format.")

        # irregular value for read mode
        dicts = {p: 0x7FFFFFFF for p in params}
        values = self.write_config(dicts)

        for i in range(len(params)):
            if params[i] == "std-vel":  # 水平速度と回転速度両方が帰ってくるので除く
                values.pop(i)
            print(params[i], ":", values[i])
        return values

    # 指定していない値が勝手に変わるのはまずいので初期値無し
    # def write_config(self, params={"acc":1000, "dec":1000, "std-vel":0.5, "torque":1000}):
    def write_config(self, params):
        logger.debug("write_config")
        if not isinstance(params, dict):
            self._print_warning("Please provide the params in dict format.")

        command = []
        for k, v in params.items():
            if k == "acc":
                command.append([RobotCodes.STANDARD_ACCELERATION_TIME,
                               RobotValueTypes[RobotCodes.STANDARD_ACCELERATION_TIME](v)])
            elif k == "dec":
                command.append([RobotCodes.STANDARD_DECELERATION_TIME,
                               RobotValueTypes[RobotCodes.STANDARD_DECELERATION_TIME](v)])
            elif k == "std-vel":
                command.append([RobotCodes.STANDARD_HORIZONTAL_SPEED,
                               RobotValueTypes[RobotCodes.STANDARD_HORIZONTAL_SPEED](v)])
                command.append([RobotCodes.STANDARD_ROTATION_SPEED,
                               RobotValueTypes[RobotCodes.STANDARD_ROTATION_SPEED](v)])
            elif k == "torque":
                command.append([RobotCodes.DRIVING_TORQUE,
                               RobotValueTypes[RobotCodes.DRIVING_TORQUE](v)])
            elif k == "std-rot":
                command.append([RobotCodes.STANDARD_ROTATION_SPEED,
                               RobotValueTypes[RobotCodes.STANDARD_ROTATION_SPEED](v)])

            elif k == "kin-matrix":
                command.append(
                    [RobotCodes.KINEMATICS,   RobotValueTypes[RobotCodes.KINEMATICS](v)])
            elif k == "kin-trans":
                command.append([RobotCodes.KINEMATICS_TRANS,
                               RobotValueTypes[RobotCodes.KINEMATICS_TRANS](v)])
            else:
                print(k, "is not configure value.")
        if len(command) > 0:
            logger.debug(self.byteList_to_string(self.tx(command)))
            return self.rx()
        else:
            return False

    # read mode not implemented
    def set_pos_absolute(self, x, y, w, acc=None, dec=None, vel_xy=None, vel_w=None):   
        logger.debug("set_pos_absolute")
        td3p = RobotValueTypes[RobotCodes.TARGET_POSITION_ABSOLUTE](x, y, w)
        query = [[RobotCodes.TARGET_POSITION_ABSOLUTE, td3p]]
        if acc is not None:
            acc = RobotValueTypes[RobotCodes.ACCELERATION_TIME](acc, acc, acc)
            query.append([RobotCodes.ACCELERATION_TIME, acc])
        if dec is not None:
            dec = RobotValueTypes[RobotCodes.DECELERATION_TIME](dec, dec, dec)
            query.append([RobotCodes.DECELERATION_TIME, dec])
        if vel_xy is not None:
            vel = RobotValueTypes[RobotCodes.POSITION_DRIVE_STD_SPEED](vel_xy)
            query.append([RobotCodes.POSITION_DRIVE_STD_SPEED, vel])
        if vel_w is not None:
            vel = RobotValueTypes[RobotCodes.POSITION_DRIVE_ROT_SPEED](vel_w)
            query.append([RobotCodes.POSITION_DRIVE_ROT_SPEED, vel])

        logger.debug(self.byteList_to_string(self.tx(query)))
        return self.rx()

    # read mode not implemented
    def set_pos_relative(self, x, y, w, acc=None, dec=None, vel_xy=None, vel_w=None):
        logger.debug("set_pos_relative")
        td3p = RobotValueTypes[RobotCodes.TARGET_POSITION_RELATIVE](x, y, w)
        query = [[RobotCodes.TARGET_POSITION_RELATIVE, td3p]]
        if acc is not None:
            acc = RobotValueTypes[RobotCodes.ACCELERATION_TIME](acc, acc, acc)
            query.append([RobotCodes.ACCELERATION_TIME, acc])
        if dec is not None:
            dec = RobotValueTypes[RobotCodes.DECELERATION_TIME](dec, dec, dec)
            query.append([RobotCodes.DECELERATION_TIME, dec])
        if vel_xy is not None:
            vel = RobotValueTypes[RobotCodes.POSITION_DRIVE_STD_SPEED](vel_xy)
            query.append([RobotCodes.POSITION_DRIVE_STD_SPEED, vel])
        if vel_w is not None:
            vel = RobotValueTypes[RobotCodes.POSITION_DRIVE_ROT_SPEED](vel_w)
            query.append([RobotCodes.POSITION_DRIVE_ROT_SPEED, vel])

        logger.debug(self.byteList_to_string(self.tx(query)))
        return self.rx()

    def set_vel_absolute(self, vx, vy, vw, acc=None, dec=None, life_time=None):  # read mode not implemented
        logger.debug("set_vel_absolute")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_ABSOLUTE](vx, vy, vw)
        query = [[RobotCodes.MOVING_SPEED_ABSOLUTE, td3p]]
        if acc is not None:
            acc = RobotValueTypes[RobotCodes.ACCELERATION_TIME](acc, acc, acc)
            query.append([RobotCodes.ACCELERATION_TIME, acc])
        if dec is not None:
            dec = RobotValueTypes[RobotCodes.DECELERATION_TIME](dec, dec, dec)
            query.append([RobotCodes.DECELERATION_TIME, dec])
        if life_time is not None:
            life = RobotValueTypes[RobotCodes.MOVING_DRIVE_LIFE_TIME](life_time)
            query.append([RobotCodes.MOVING_DRIVE_LIFE_TIME, life])
        logger.debug(self.byteList_to_string(self.tx(query)))
        return self.rx()

    def set_vel_relative(self, vx, vy, vw, acc=None, dec=None, life_time=None, drive_mode=None):
        logger.debug("set_vel_relative")
        td3p = RobotValueTypes[RobotCodes.MOVING_SPEED_RELATIVE](vx, vy, vw)
        query = [[RobotCodes.MOVING_SPEED_RELATIVE, td3p]]
        if acc is not None:
            acc = RobotValueTypes[RobotCodes.ACCELERATION_TIME](acc, acc, acc)
            query.append([RobotCodes.ACCELERATION_TIME, acc])
        if dec is not None:
            dec = RobotValueTypes[RobotCodes.DECELERATION_TIME](dec, dec, dec)
            query.append([RobotCodes.DECELERATION_TIME, dec])
        if life_time is not None:
            life = RobotValueTypes[RobotCodes.MOVING_DRIVE_LIFE_TIME](life_time)
            query.append([RobotCodes.MOVING_DRIVE_LIFE_TIME, life])
        if drive_mode is not None:
            mode = RobotValueTypes[RobotCodes.DRIVE_MODE](drive_mode)
            query.append([RobotCodes.DRIVE_MODE, mode])
        logger.debug(self.byteList_to_string(self.tx(query)))
        return self.rx()

    def get_info(self):
        logger.debug("get_info")
        val = RobotValueTypes[RobotCodes.SYSTEM_INFORMATION]()
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.SYSTEM_INFORMATION, val]])))
        values = self.rx()
        return values[0]

    def get_device_status(self):
        logger.debug("get_device_status")
        val = RobotValueTypes[RobotCodes.DEVICE_STATUS]()
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.DEVICE_STATUS, val]])))
        values = self.rx()
        return values[0]

    def get_sensor_info(self):
        logger.debug("get_sensor_info")
        val = RobotValueTypes[RobotCodes.SENSOR_INFORMATION]()
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.SENSOR_INFORMATION, val]])))
        values = self.rx()
        return values[0]

    def get_motor_status(self, params=["error", "state", "voltage", "power"], _id=ALL_MOTOR_LOCAL_IDS):

        if not isinstance(_id, list):
            _id = [_id]
        if isinstance(params, str):
            params = [params]
        code_dicts = {"error": RobotCodes.ERROR_INFORMATION,
                      "state": RobotCodes.OPERATING_STATUS,
                      "voltage": RobotCodes.POWER_SUPPLY_VOLTAGE,
                      "power": RobotCodes.DRIVING_POWER}
        if len(params) > 3: # 送信バイト数の上限を超えちゃうので分割して送信するための分岐
            query = []
            for pm in params[:3]:
                code = code_dicts[pm]
                val = RobotValueTypes[code](0)
                val = self.to_bytes(val)
                for i in _id:
                    if i in ALL_MOTOR_LOCAL_IDS:
                        # 最後の1バイトでIDを指定
                        v = val[:-1] + int(i).to_bytes(1, "little")
                        query.append([code, v])
                    else:
                        print("motor ID %d is not existing" % i)
            logger.debug(self.byteList_to_string(self.tx(query)))
            res1 = self.rx()

            query = []
            code = code_dicts[params[-1]]
            val = RobotValueTypes[code](0)
            val = self.to_bytes(val)
            for i in _id:
                if i in ALL_MOTOR_LOCAL_IDS:
                    v = val[:-1] + int(i).to_bytes(1, "little")
                    query.append([code, v])
                else:
                    print("motor ID %d is not existing" % i)
            logger.debug(self.byteList_to_string(self.tx(query)))
            res2 = self.rx()
            return res1 + res2


        else:
            query = []
            for pm in params:
                code = code_dicts[pm]
                val = RobotValueTypes[code](0)
                val = self.to_bytes(val)
                for i in _id:
                    if i in ALL_MOTOR_LOCAL_IDS:
                        # 最後の1バイトでIDを指定
                        v = val[:-1] + int(i).to_bytes(1, "little")
                        query.append([code, v])
                    else:
                        print("motor ID %d is not existing" % i)
            logger.debug(self.byteList_to_string(self.tx(query)))
            return self.rx()



    def reset_error(self):
        logger.debug("reset_error")
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.ERROR_RESET, 0x01]])))
        return self.rx()

    def reset_origin(self):
        logger.debug("reset_origin")
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.ORIGIN_RESET, 0x01]])))
        return self.rx()

    def set_odometry(self, x, y, w):
        logger.debug("set_odometry")
        td3p = RobotValueTypes[RobotCodes.SET_POSE](x, y, w)
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.SET_POSE, td3p]])))
        return self.rx()

    def operating_mode(self, param=0x03):  # no need?
        logger.debug("operating_mode")
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.OPERATING_MODE, param]])))
        return self.rx()

    def set_acceleration_time(self, param):  # read mode not implemented yet
        logger.debug("set_acceleration_time")
        td3p = RobotValueTypes[RobotCodes.ACCELERATION_TIME](
            param, param, param)
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.ACCELERATION_TIME, td3p]])))
        return self.rx()

    def set_deceleration_time(self, param):  # read mode not implemented yet
        logger.debug("set_deceleration_time")
        td3p = RobotValueTypes[RobotCodes.DECELERATION_TIME](
            param, param, param)
        logger.debug(self.byteList_to_string(
            self.tx([[RobotCodes.DECELERATION_TIME, td3p]])))
        return self.rx()

    def set_aeb(self, param):
        logger.debug("set_aeb")
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.AEB_MODE, param]])))
        return self.rx()

    def set_motor_param(self, lpf=True, filter_t=1, pos_p_gain=10, speed_p_gain=100, speed_i_gain=1580, torque_filter=1000, speed_feedforward=80, stiffness=7):
        logger.debug("set_motor_param")
        #smp = RobotValueTypes[RobotCodes.SET_MOTOR_PARAMS]( lpf, filter_t, pos_p_gain, speed_p_gain, speed_i_gain )
        smp = RobotValueTypes[RobotCodes.SET_MOTOR_PARAMS]( lpf, filter_t, pos_p_gain, speed_p_gain, speed_i_gain, torque_filter, speed_feedforward, stiffness )
        logger.debug(self.byteList_to_string(self.tx([[RobotCodes.SET_MOTOR_PARAMS, smp]])))
        return self.rx()


    # def set_torque(self, param):
    #    logger.debug("set_torque")
    #    code = RobotCodes.DRIVING_TORQUE
    #    val = RobotValueTypes[code](param)
    #    logger.debug(self.byteList_to_string(self.tx([[code, val ]])))
    #    return self.rx()[0]

    def close_serial(self):
        self._uart.close()

    def __del__(self):
        self.sleep()
        if self._uart is not None:
            self._uart.close()
