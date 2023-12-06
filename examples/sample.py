import sys
import os
import time
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# sample1
def sample1(port):
    from triorb_core import robot
    vehicle = robot(port) 
    vehicle.wakeup() # 起動 
    vehicle.sleep() # 休止 


def sample2(port):
    from triorb_core import robot 
    vehicle = robot(port) 
    print(vehicle.get_info())
    print(vehicle.get_motor_status(params=['error','state','voltage','power'], _id=[1,2,3]))    # 指定したIDのモータードライバの各種ステータスを取得 


def sample3(port):
    from triorb_core import robot 
    vehicle = robot(port) 
    vehicle.wakeup() # 起動 
    time.sleep(3)
    vehicle.write_config({ # 設定を書込み 
        "acc" : 1000,
        "std-vel" : 0.4, 
        "std-rot" : 1.5,
    }) 
    time.sleep(3)
    #vehicle.set_pos_relative(x=1.0, y=-1.0, w=90.0) # ロボットの姿勢をローカル座標系で設定。並進位置x=1.0[m], y=-1.0[m]および回転位置w=90.0[deg] の姿勢を取ります。 
    vehicle.set_pos_relative(x=0.0, y=1.0, w=0.0) # ロボットの姿勢をローカル座標系で設定。並進位置x=1.0[m], y=-1.0[m]および回転位置w=90.0[deg] の姿勢を取ります。 
    time.sleep(3)
    vehicle.join() # 動作完了待ち 
    time.sleep(3)
    vehicle.set_pos_relative(x=0.0, y=0, w=-90)
    time.sleep(3)
    vehicle.join() 
    print(vehicle.set_pos_relative(x=3600001, y=0, w=0)) # 無効値を入力した場合、前回の指示値を取得できます。


def sample4(port):
    #import logging
    #formatter = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    #logging.basicConfig(level=logging.DEBUG, format=formatter)
    from triorb_core import robot 
    vehicle = robot(port) 
    vehicle.wakeup() # 起動 
    time.sleep(3.0)
    vehicle.set_vel_relative(vx=0.1, vy=0.1, vw=0.1) # ロボットの速度をローカル座標系で設定。並進x=0.1[m/s], y=0.1[m/s]および旋回w=0.1[rad/s] の速度で移動。 
    time.sleep(5.0) # 5秒待つ 
    vehicle.brake() # 減速停止する 
    vehicle.join()  # 移動完了待ち 

    vehicle.set_vel_relative(vx=0.1, vy=0, vw=0, acc=3000, dec=2000) # 加速時間3秒、減速時間2秒に設定し、ロボットの速度をローカル座標系の並進x=0.1[m/s]に設定。 
    time.sleep(5.0) # 5秒待つ 
    vehicle.brake() # 減速停止する  
    vehicle.join()  # 移動完了待ち 
    print(vehicle.set_vel_relative(vx=3600001, vy=0, vw=0)) # 現在速度を取得

def sample5(port):
    from triorb_core import robot 
    vehicle = robot(port) 
    print(vehicle.read_config()) # 現在設定を読み込み 
    vehicle.write_config({ # 設定を書込み 
        "acc" : 1500, # 標準加減速時間を1.5[s]に設定 
        "std-vel" : 0.25, # 標準並進速度を0.25[m/s]に設定 
        "std-rot" : 0.5, # 標準旋回速度を0.5[rad/s]に設定 
        "torque" : 500,  # トルク制限値を50[%]に設定 
    }) 
    print(vehicle.read_config(["acc","std-vel","torque"])) # 現在設定を読み込み 

def sample6(port):
    import numpy as np 
    vehicle = robot(port) 

    query = [] 
    query.append([RobotCodes(0x0301), b'\x02'])        # 励磁 
    query.append([RobotCodes(0x0313), TriOrbDrive3Pose(1,-1,0)])  # 相対姿勢制御  
    query.append([b'\x05\x03', np.uint32(1000)])    # 標準加減速時間設定 
    vehicle.tx(code_array=query) # 送信 
    print(vehicle.rx()) # 受信 

    query = []  
    query.append([RobotCodes(0x0009), b'\x00\x01'])  # モータードライバのステータス取得(ID1) 
    val = TriOrbBaseState(motor_id=2) 
    query.append([b'\x09\x00', val])  # モータードライバのステータス取得(ID2) 
    vehicle.tx(code_array=query) # 送信 
    print(vehicle.rx()) # 受信 

    time.sleep(5.0) 
    vehicle.tx(code_array=[[RobotCodes(0x0301), b'\x01']]) # 励磁解除 
    vehicle.rx() 

    # 誤ったデータの送信 
    vehicle.tx(code_array=[[RobotCodes(0x0301), b'\x02'],  # 励磁(正) 
                           [RobotCodes(0x0305), b'\x03']]) # 加速度設定(誤) 
    # 返送（STARTとENDのみ） 
    print(vehicle.rx()) # -> b'\x00\r\n' 


if __name__ == "__main__":
    port = "COM32"
    #print("sample1")
    #sample1(port)
    #time.sleep(2)
    #print("sample2")
    #sample2(port)
    #time.sleep(2)
    #print("sample3")
    #sample3(port)
    #time.sleep(2)
    #print("sample4")
    #sample4(port)
    #time.sleep(2)
    print("sample5")
    sample5(port)
    #time.sleep(2)
    #from triorb_core import *
    #print("sample6")
    #sample6(port)
