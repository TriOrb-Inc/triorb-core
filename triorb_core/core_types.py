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
from dataclasses import dataclass
import numpy as np
import struct

@dataclass
class TriOrbBaseSystem:
    age: int = 0
    weight: int = 0
    def to_bytes(self) -> bytes:
        return struct.pack('<ii', self.age, self.weight)
    def from_bytes(self, arr):
        self.age, self.weight = struct.unpack("<ii", arr)
    
@dataclass
class TriOrbBaseDevice:
    age: int = 0
    weight: int = 0
    def to_bytes(self) -> bytes:
        return struct.pack('<ii', self.age, self.weight)
    def from_bytes(self, arr):
        self.age, self.weight = struct.unpack("<ii", arr)

@dataclass
class TriOrbBaseSensor:
    age: int = 0
    weight: int = 0
    def to_bytes(self) -> bytes:
        return struct.pack('<ii', self.age, self.weight)
    def from_bytes(self, arr):
        self.age, self.weight = struct.unpack("<ii", arr)

@dataclass
class TriOrbBaseError:
    alarm: np.uint8 = 0
    motor_id: np.uint8 = 0
    def to_bytes(self) -> bytes:
        return struct.pack('<bb', self.alarm, self.motor_id)
    def from_bytes(self, arr):
        self.alarm, self.motor_id = struct.unpack("<BB", arr)
    

@dataclass
class TriOrbDriveUSS:
    v1: np.uint8 = 255
    v2: np.uint8 = 255
    v3: np.uint8 = 255
    v4: np.uint8 = 255
    v5: np.uint8 = 255
    def to_bytes(self) -> bytes:
        return struct.pack('<BBBBB', self.v1, self.v2, self.v3, self.v4, self.v5)
    def from_bytes(self, arr):
        self.v1, self.v2, self.v3, self.v4, self.v5 = struct.unpack("<BBBBB", arr)


@dataclass
class TriOrbMotorParams:
    lpf:          bool = True
    filter_t:     np.uint8 = 1
    pos_p_gain:   np.uint8 = 10
    speed_p_gain: np.uint16 = 100
    speed_i_gain: np.uint16 = 1580
    torque_filter: np.uint16 = 1000
    speed_feedforward: np.uint8 = 80
    stiffness: np.uint8 = 7
    def to_bytes(self) -> bytes:
        return struct.pack('<BBBHHHBB', self.lpf, self.filter_t, self.pos_p_gain, self.speed_p_gain, self.speed_i_gain, self.torque_filter, self.speed_feedforward, self.stiffness)
    def from_bytes(self, arr):
        self.lpf, self.filter_t, self.pos_p_gain, self.speed_p_gain, self.speed_i_gain, self.torque_filter, self.speed_feedforward, self.stiffness = struct.unpack("<?BBHHHBB", arr)
        


@dataclass
class TriOrbBaseState:
    btn_y:   bool = 0
    btn_b:   bool = 0
    btn_a:   bool = 0
    btn_x:   bool = 0
    move:    bool = 0
    in_pos:  bool = 0
    s_on:    bool = 0
    success: bool = 0

    emergency: bool = 0
    flag1: bool = 0
    flag2: bool = 0
    flag3: bool = 0
    flag4: bool = 0
    flag5: bool = 0
    flag6: bool = 0
    flag7: bool = 0

    motor_id: np.uint8 = 0
    def to_bytes(self) -> bytes:
        hb =  self.btn_y<<7 | self.btn_b<<6 | self.btn_a<<5 | self.btn_x<<4 \
             |self.move<<3 | self.in_pos<<2 | self.s_on<<1 | self.success
        lb =  self.emergency<<7

        return hb.to_bytes(length=1, byteorder=sys.byteorder) + lb.to_bytes(length=1, byteorder=sys.byteorder) + self.motor_id.to_bytes(length=1, byteorder=sys.byteorder)

#    def from_bytes(self, arr):
#        state, self.motor_id = struct.unpack("<bb", arr)
#        #print("%16b\n" % state)
#        self.volt_l = (state & 0b00000001) >> 0
#        self.volt_h = (state & 0b00000010) >> 1
#        self.watt   = (state & 0b00000100) >> 2
#        self.trq    = (state & 0b00001000) >> 3
#        self.move   = (state & 0b00010000) >> 4
#        self.in_pos = (state & 0b00100000) >> 5
#        self.s_on   = (state & 0b01000000) >> 6
#        #self.success= state!=0
#        self.success= (state & 0b10000000) >> 7
#        #print(state)
#        if self.success==0:
#            print("motor ID{} failed to read status".format(self.motor_id))

    def from_bytes(self, arr, from_BE=False):
        state, state2, self.motor_id = struct.unpack("<bbb", arr)
        #print("%16b\n" % state)
        if from_BE:
            self.btn_y  = (state & 0b00000001) > 0
            self.btn_b  = (state & 0b00000010) > 0
            self.btn_a  = (state & 0b00000100) > 0
            self.btn_x  = (state & 0b00001000) > 0
            self.move   = (state & 0b00010000) > 0
            self.in_pos = (state & 0b00100000) > 0
            self.s_on   = (state & 0b01000000) > 0
            self.success= (state & 0b10000000) > 0

            self.emergency = (state2 & 0b00000001) > 0
        else:
            self.btn_y  = (state & 0b10000000) > 0
            self.btn_b  = (state & 0b01000000) > 0
            self.btn_a  = (state & 0b00100000) > 0
            self.btn_x  = (state & 0b00010000) > 0
            self.move   = (state & 0b00001000) > 0
            self.in_pos = (state & 0b00000100) > 0
            self.s_on   = (state & 0b00000010) > 0
            self.success= (state & 0b00000001) > 0

            self.emergency = (state2 & 0b10000000) > 0

        if self.success==0:
            print("motor ID{} failed to read status".format(self.motor_id))

    

@dataclass
class TriOrbDrive3Pose:
    x: np.float32 = 0.0
    y: np.float32 = 0.0
    w: np.float32 = 0.0
    def to_bytes(self) -> bytes:
        return struct.pack('<fff', self.x, self.y, self.w)
    def from_bytes(self, arr):
        self.x, self.y, self.w = struct.unpack("<fff", arr)

@dataclass
class TriOrbDrive3Vector:
    v1: np.float32 = 0.0
    v2: np.float32 = 0.0
    v3: np.float32 = 0.0
    def to_bytes(self) -> bytes:
        return struct.pack('<fff', self.v1, self.v2, self.v3)
    def from_bytes(self, arr):
        self.v1, self.v2, self.v3 = struct.unpack("<fff", arr)


@dataclass
class TriOrbDriveMatrix:
    #mat:  np.ndarray = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype=np.float32)
    mat:  np.ndarray
    #m11: np.float32 = 1.0
    #m12: np.float32 = 0.0
    #m13: np.float32 = 0.0
    #m21: np.float32 = 0.0
    #m22: np.float32 = 1.0
    #m23: np.float32 = 0.0
    #m31: np.float32 = 0.0
    #m32: np.float32 = 0.0
    #m33: np.float32 = 1.0
    def to_bytes(self) -> bytes:
        #return struct.pack('<fffffffff', self.m11, self.m12, self.m13,
        #                                 self.m21, self.m22, self.m23,
        #                                 self.m31, self.m32, self.m33,
        return struct.pack('<fffffffff', self.mat[0,0], self.mat[0,1], self.mat[0,2],
                                         self.mat[1,0], self.mat[1,1], self.mat[1,2],
                                         self.mat[2,0], self.mat[2,1], self.mat[2,2])

    def from_bytes(self, arr):
        self.mat[0,0], self.mat[0,1], self.mat[0,2], self.mat[1,0], self.mat[1,1], self.mat[1,2], self.mat[2,0], self.mat[2,1], self.mat[2,2] = struct.unpack('<fffffffff', arr)
                                         
                                         
