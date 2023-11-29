import sys
import os
import logging
import time
import random
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from triorb_core import *

formatter = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(level=logging.DEBUG, format=formatter)

def check_robot_odom(com1):
    vehicle = robot(com1)
    vehicle.wakeup()
    
    st = time.time()
    vel_set = [0.0]
    time_table = [0.0]
    time.sleep(2)
    
    for i in range(10):

        vel = random.uniform(-0.5, 0.5) / 10
        vel_set.append(vel)
        print(vel)
        time_table.append(time.time()-st)
        vehicle.set_vel_relative(vx=vel, vy=0.0, vw=0.0)
        delay = random.uniform(0.5, 5)
        time.sleep(delay)

    vel_set.append(0.0)
    time_table.append(time.time()-st)
    vehicle.set_vel_relative(vx=0.0, vy=0.0, vw=0.0)

    data = np.array( [time_table,vel_set])
    np.savetxt("./res_lpf.csv", data.T, delimiter=",")


    plt.plot(time_table, vel_set)
    plt.show()



def check_robot_move(com1):
    vehicle = robot(com1)
    vehicle.wakeup()
    
    st = time.time()
    vel_set = [0.0]
    time_table = [0.0]
    time.sleep(2)
    
    for vel in [0.5,0.2,-0.5,0,0.3]:
        vel_set.append(vel)
        time_table.append(time.time()-st)
        vehicle.set_vel_relative(vx=vel/10.0, vy=0.0, vw=0.0, acc=500, dec=500)
        time.sleep(0.3)

    time.sleep(2.0)
    exit()
    vel_set.append(0.0)
    time_table.append(time.time()-st)
    vehicle.set_vel_relative(vx=0.0, vy=0.0, vw=0.0)

    time.sleep(2.0)

    data = np.array( [time_table,vel_set])
    np.savetxt("./res_lpf.csv", data.T, delimiter=",")


    #plt.plot(time_table, vel_set)
    #plt.show()



def check_robot_pos(com1):
    vehicle = robot(com1)
    vehicle.wakeup()
    
    st = time.time()
    pos_set = [0.0]
    time_table = [0.0]
    time.sleep(2)
    
    #for pos in [0.2,-0.2]:
    a = 0
    for pos in [0.1,-0.1,0.3]:
        pos_set.append(pos)
        time_table.append(time.time()-st)
        if a == 0:
            vehicle.set_pos_relative(x=pos, y=0.0, w=0.0, acc=500, dec=1000)
        else:
            vehicle.set_pos_relative(x=pos, y=0.0, w=0.0, acc=1000, dec=1000)
        a = 1
        time.sleep(3)

    pos_set.append(0.0)
    time_table.append(time.time()-st)
    vehicle.set_pos_relative(x=0.0, y=0.0, w=0.0)

    time.sleep(2.0)

    data = np.array( [time_table,pos_set])
    np.savetxt("./res_pos_avg.csv", data.T, delimiter=",")



def check_robot_vel_pos(com1):
    vehicle = robot(com1)
    vehicle.wakeup()
    
    st = time.time()
    pos_set = [0.0]
    time_table = [0.0]
    time.sleep(2)
    
    #for pos in [0.2,-0.2]:
    #vehicle.set_vel_relative(vx=-0.3, vy=0.0, vw=0.0, acc=500, dec=500)
    #time.sleep(3)
    vehicle.set_pos_relative(x=0.3, y=0.0, w=0.0, acc=1000, dec=500)
    a = 1
    time.sleep(2)
    vehicle.join()




if __name__ == '__main__':
    #check_robot_odom("COM32")
    #check_robot_move("COM32")
    check_robot_pos("COM32")
    #check_robot_vel_pos("COM32")

