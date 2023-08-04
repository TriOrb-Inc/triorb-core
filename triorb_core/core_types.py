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
        self.alarm, self.motor_id = struct.unpack("<bb", arr)
    
@dataclass
class TriOrbBaseState:
    #state: np.uint32 = 0
    volt_l:  bool = 0
    volt_h:  bool = 0
    watt:    bool = 0
    trq:     bool = 0
    move:    bool = 0
    in_pos:  bool = 0
    s_on:    bool = 0
    success: bool = 0
    motor_id: np.uint8 = 0
    def to_bytes(self) -> bytes:
        #return struct.pack('<ib', self.state, self.motor_id)
        hb =  self.volt_l<<7 + self.volt_h<<6 + self.watt<<5 + self.trq<<4 \
             +self.move<<3 + self.in_pos<<2 + self.s_on<<1 + self.success
        return hb.to_bytes() + self.motor_id.to_bytes()

    def from_bytes(self, arr):
        state, self.motor_id = struct.unpack("<bb", arr)
        #print("%16b\n" % state)
        self.volt_l = (state & 0b00000001) >> 0
        self.volt_h = (state & 0b00000010) >> 1
        self.watt   = (state & 0b00000100) >> 2
        self.trq    = (state & 0b00001000) >> 3
        self.move   = (state & 0b00010000) >> 4
        self.in_pos = (state & 0b00100000) >> 5
        self.s_on   = (state & 0b01000000) >> 6
        #self.success= state!=0
        self.success= (state & 0b10000000) >> 7
        #print(state)
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
                                         
                                         
