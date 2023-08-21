import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from triorb_core import robot

vehicle = robot("COM1")
vehicle.wakeup() # 起動

#print(vehicle.get_pos()) # 現在姿勢を取得（wakeup時の姿勢が原点）
#vehicle.set_pos_absolute(x=1.0, y=1.0, w=180.0) # ロボットの姿勢を絶対座標系で設定。水平x=1.0[m], y=1.0[m]および回転w=180.0[deg] の姿勢に移動します。
#vehicle.join() # 運転完了待ち
#print(vehicle.get_pos()) # 現在姿勢を取得
vehicle.set_pos_relative(x=1.0, y=-1.0, w=90.0) # ロボットの姿勢をローカル座標系で設定。水平x=1.0[m], y=-1.0[m]および回転w=90.0[deg] 移動します。
vehicle.join() # 運転完了待ち
#print(vehicle.get_pos()) # 現在姿勢を取得

vehicle.sleep() # 休止


import time
from triorb_core import robot

vehicle = robot("COM1")
vehicle.wakeup() # 起動

#print(vehicle.get_pos()) # 現在姿勢を取得（wakeup時の姿勢が原点）
#vehicle.set_vel_absolute(x=0.5, y=0.5, w=45.0) # ロボットの速度を絶対座標系で設定。水平x=0.5[m/s], y=0.5[m/s]および回転w=45.0[deg/s] の速度で移動します。
#time.sleep(5.0) # 1秒待つ
#vehicle.brake() # 減速停止する
#vehicle.join() # 運転完了待ち

print(vehicle.get_pos()) # 現在姿勢を取得（wakeup時の姿勢が原点）
vehicle.set_vel_relative(x=0.1, y=0.1, w=0.1) # ロボットの速度をローカル座標系で設定。水平x=0.1[m/s], y=0.1[m/s]および回転w=0.1[rad/s] の速度で移動します。
time.sleep(3.0) # 3秒待つ
vehicle.brake() # 減速停止する
vehicle.join() # 運転完了待ち

vehicle.sleep() # 休止



from triorb_core import robot

vehicle = robot("COM1")
print(vehicle.read_config()) # 現在設定を読み込み
vehicle.write_config({ # 設定を書込み
    "acc" : 1500, # 標準加速時間を1.5[s]に設定
    "std-vel" : 0.1, # 標準速度を0.1[m/s]に設定
    "std-rot" : 0.1, # 標準旋回速度を0.1[rad/s]に設定
})
print(vehicle.read_config(["acc","std-vel","std-rot"])) # 現在設定を読み込み


from triorb_core import robot

vehicle = robot("COM1")
print(vehicle.get_info()) # 現在状態を取得

