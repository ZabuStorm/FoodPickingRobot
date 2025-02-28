#Author: Prashant Kumar
#Created:5.07.2024
#Purpose: To run a closed loop spaghetti pickup with the developed gripper

import time
# NX-ROS Series Package
from nxr_001_ftcg_client import controllerIcraClient

import timeit
start = timeit.default_timer()

import os

import importlib
FOUND_CNOID = importlib.util.find_spec("cnoid") is not None
if FOUND_CNOID: 
    from cnoid.Util import *
    from cnoid.Base import *
    from cnoid.Body import *
    from cnoid.BodyPlugin import *
    from cnoid.GraspPlugin import *
    from cnoid.BinPicking import * 



import timeit
start = timeit.default_timer()

import time
import datetime
import numpy as np
np.set_printoptions(suppress=True)
import os
from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
from bpbot.device import DynPickClient, DynPickControl

#config related commands
root_dir = "/home/hlab/bpbot"
config_path = os.path.join(root_dir, "config/config_soft_hand.yaml")
print(f"Root directory: {root_dir} ")
print(f"Config path   : {config_path} ")
cfg = BinConfig(config_path=config_path)
cfgdata = cfg.data
camera_mode = cfgdata["exp"]["camera_mode"]
exp_mode = cfgdata["exp"]["mode"]
mf_mode = cfgdata["exp"]["motionfile_mode"]
lr_arm = cfgdata["exp"]["lr_arm"]
log_mode = cfgdata["exp"]["log_mode"]

nxt = NxtRobot(host='[::]:15005')

dc = DynPickClient()
fs = DynPickControl()
sensitivity = np.array(cfgdata["force_sensor"]["sensitivity"])
F0_reset = np.zeros(6)


p1=[0.558,-0.251,0.520,-90.0,-90.0,110.0]
p2=[0.558,-0.251,0.290,-90.0,-90.0,110.0]
p3=[0.558,-0.100,0.520,-90.0,-90.0,110.0]
#p4=[0.558,-0.80,0.300,-90.0,-90.0,70.0]

#Dynamixxel definitions
controllerClient = controllerIcraClient()
while not controllerClient.getConnectionStatus():
        controllerClient.connect()


arm="right"
degree=30
degree_2=50
controllerClient.setServoSpeed(10)
deg_des=80

deg_p=250
def picksp():
    controllerClient.controlGripper("open_gripper_slow")
    time.sleep(3)

    success = plan_move(arm,[p1,p2],[3,7])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.controlGripper("close_gripper_slow")
    time.sleep(2)
    controllerClient.controlGripper("bothServos_down", deg_des,deg_des)

    success = plan_move(arm,[p1],[8])
    motion_seq = get_motion()
    clear_motion() 
    nxt.playMotion(motion_seq)
    
    
    success = plan_move(arm,[p3],[5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    #robot pulls everything up 

def drop_sp():

    print("dropping now")
    #controllerClient.controlGripper("bothServos_up", deg_des*4,deg_des*4)
    controllerClient.controlGripper("bothServos_up", deg_des*2,deg_des*2)    #used for single drop
    time.sleep(5)
    #controllerClient.controlGripper("bothServos_down", deg_des*4-50,deg_des*4-50)
    controllerClient.controlGripper("bothServos_down", deg_des,deg_des)

b=10
def drop_sp_feedback():

    print("Starting the dropping process")
    f_t=dc.get()                            #Getting the weight value right after we reach the rop of the bowl
    F_pre=f_t[2]
    print(F_pre)

    print("------ Weight_pre------")
    F_post=F_pre-8
    print("------ Weight_post------")
    print(F_post)
    
    for i in range(0,b):
        
        f_t=dc.get()                            #Getting the weight value right after we reach the rop of the bowl
        F_pre=f_t[2]
        controllerClient.controlGripper("bothServos_up", degree,degree)
        time.sleep(2)
        if(F_pre<F_post):
            print("correct weight reached. Ending process")
            break
        print("weight on cycle numer ",i+1," is :",F_pre)
        time.sleep(2)
    print("Cycle limit reached")
    controllerClient.controlGripper("bothServos_down", degree*i-40,degree*i-40)


def inc_picksp():

    #go to p1, turn one bit, go slightly deeper, turn another bit and continue

   # controllerClient.controlGripper("open_gripper_slow")
   # time.sleep(3)
    
    success = plan_move(arm,[p1,p2],[7,7])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

   # controllerClient.controlGripper("close_gripper_slow")
    #time.sleep(2)
    controllerClient.controlGripper("bothServos_down", deg_des,deg_des)


    for descent in range(1,4):
        p_d=[p2[0],p2[1],p2[2]-0.01*descent,p2[3],p2[4],p2[5]]
        success = plan_move(arm,[p_d],[2])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)
        controllerClient.controlGripper("bothServos_down", deg_des,deg_des)
    #one rotation of servo
        



"""
#------------Main body----------------

time.sleep(3)
picksp()
time.sleep(5)
drop_sp()
#drop_sp_feedback()
print("Finishing the process")
controllerClient.controlGripper("open_gripper_slow")
time.sleep(3)
controllerClient.controlGripper("close_gripper_slow")

#adjust_up()

"""
arm="right"
#success = plan_move(arm,[p1,p2,p1,p3],[5,5,5,5])
#motion_seq = get_motion()
#clear_motion()
#nxt.playMotion(motion_seq)

#inc_picksp()
picksp()

#success = plan_move(arm,[p1],[7])
#motion_seq = get_motion()
#clear_motion()
#nxt.playMotion(motion_seq)

drop_sp()

print("motion done")











