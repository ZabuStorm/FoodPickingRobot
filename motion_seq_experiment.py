#Created on 7.10.2024 by Prasahnt
#To compile all the motion sequence and combination for the food gripper robot setup




from dynamixel_sdk import *  # Dynamixel SDK
import motor_control as dm
import time
#NX-ROS Series Package
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


# ---------------------- SETUP PARAMETERS ----------------------
DEVICENAME = "/dev/ttyUSB0"  # port fir the servos
BAUDRATE = 57600

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

EXTENDED_POSITION_MODE = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DXL_ID_1 = 1
DXL_ID_2 = 2
PROTOCOL_VERSION = 2.0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Port and Baudrate Set Successfully")
else:
    print("Failed to initialize port")
    exit()

ser = serial.Serial(port="/dev/ttyUSB1", baudrate=9600, timeout=1)  #Weighing scale connection

import datetime
import numpy as np
np.set_printoptions(suppress=True)
import os
from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
from bpbot.device import DynPickClient, DynPickControl

nxt = NxtRobot(host='[::]:15005')
#dc = DynPickClient()
#fs = DynPickControl()

p1=[0.458,-0.250,0.530,-90.0,-90.0,110.0]     #Over the picking bowl
p2=[0.458,-0.250,0.240,-90.0,-90.0,110.0]     #Upto the top surface of the bowl
p3=[0.458,-0.250,0.260,-90.0,-90.0,110.0]     #Inside the bowl
p4=[0.488,-0.050,0.550,-90.0,-90.0,110.0]     #over the weighing bowl
p5=[0.488,-0.050,0.320,-90.0,-90.0,110.0]     #Closer to the weighing bowl

#p1=[0.548,-0.251,0.450,-90.0,-90.0,110.0]     #Over the picking bowl
#p2=[0.548,-0.251,0.230,-90.0,-90.0,110.0]     #Upto the top surface of the bowl
#p3=[0.548,-0.251,0.210,-90.0,-90.0,110.0]     #Inside the bowl
#p4=[0.548,-0.080,0.500,-90.0,-90.0,110.0]     #over the weighing bowl
#p5=[0.548,-0.080,0.340,-90.0,-90.0,110.0]

p11=[0.478,-0.250,0.330,-90.0,-90.0,110.0]
p12=[0.478,-0.250,0.228,-90.0,-90.0,110.0] #pickup bowl
p13=[0.458,-0.070,0.330,-90.0,-90.0,110.0]
p14=[0.458,-0.070,0.300,-90.0,-90.0,110.0]

arm="right"

#Dynamixxel definitions
controllerClient = controllerIcraClient()
while not controllerClient.getConnectionStatus():
    controllerClient.connect()




#controllerClient.setServoSpeed(30)
p_angle=220                  #Pickup angle for step pickup
p_steps=70                  #
d_angle=10                  #step angles for step drops 

z1,z2,z3,z4=0,0.01,0.02,0.03
d1,d2,d3,d4=90,20,60,150
#d1=200
degree=d1+d2
z=z4
l=1         # in cms. length of the bit/bucket
wt_og=None   #global variable for value of F sensor w/o any payload

#----------------------Motion sequences defintions------------------------#
"""
def down_rotate():
    
    controllerClient.controlGripper("close_gripper_slow")
    p_d=[p2[0],p2[1],p2[2]-z,p2[3],p2[4],p2[5]]
    #Motion to reach at the top of the bowl
    success = plan_move(arm,[p1,p2],[3,5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    #Motion inside the bowl
    success = plan_move(arm,[p_d],[4])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)


    #controllerClient.controlGripper("bothServos_down", degree,degree)
    dm.pick_up(p_angle)
    time.sleep(1)
    #Lift back up
    success = plan_move(arm,[p1],[10])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
"""

#---------------Consecutive down and rotate sequences

def down_rotate_steps(steps):
    
    global wt_og        #original weight
    
    #controllerClient.controlGripper("open_gripper_slow")
        
    #Motion to reach at the top of the bowl
    success = plan_move(arm,[p1],[3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    """
    #To obtain the weight before any pickup i.e gripper's pre-pickup weight
    for detected_load in fs.record(stop=1):
        wt_og=detected_load[2]
    print("Value of F sensor w/o any load ",wt_og)
    """

    success = plan_move(arm,[p2],[5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.controlGripper("close_gripper_slow")
    time.sleep(1)
    #controllerClient.controlGripper("bothServos_down", d1,d1)
    dm.pick_up(p_angle)

    for i in range(1,4):
        
        i=i*steps
        p_d=[p2[0],p2[1],p2[2]-0.01*i,p2[3],p2[4],p2[5]]
        success = plan_move(arm,[p_d],[1])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)

        dm.pick_up(p_angle+(i*p_steps))
        #controllerClient.controlGripper("bothServos_down", d3,d3)

    #Lift back up
    success = plan_move(arm,[p1],[4])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    time.sleep(1)
    

"""  
def down_rotate_simulatneous():
    
    controllerClient.controlGripper("close_gripper_slow")

    p_d=[p2[0],p2[1],p2[2]-z,p2[3],p2[4],p2[5]]

    #Motion to reach at the top of the bowl
    success = plan_move(arm,[p1,p2],[3,5])
    motion_seq = get_motion()
    clear_motion()
    #nxt.playMotion(motion_seq)

    #Motion inside the bowl
    success = plan_move(arm,[p_d],[5])
    motion_seq = get_motion()
    clear_motion()
    #nxt.playMotion(motion_seq,wait=False)

    controllerClient.controlGripper("bothServos_down", degree,degree)

    #Lift back up
    time.sleep(1)
    success = plan_move(arm,[p1],[10])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

"""

#moves from top of the bowl to dropping bowl and performs dropping
def transfer_drop():

    #Moves over the dropping bowl
    success = plan_move(arm,[p4],[3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    #drop_feedback()
    #time.sleep(5)
    #controllerClient.controlGripper("open_gripper_slow")
    #time.sleep(1)
    
    #controllerClient.controlGripper("bothServos_up", degree,degree)
    #dm.pick_up(0)
    #time.sleep(5)

"""
#--------Post pickup weight processing---------------
def drop_feedback():
    
    z_axis_forces = []
    
    # Start time to track the duration of 3 seconds
    start_time = time.time()
    
    # Collect data for 3 seconds
    while time.time() - start_time < 3:
        for detected_load in fs.record(stop=0):  # stop=0 to keep it running
            z_axis_forces.append(detected_load[2])  # Collect z-axis force values
            
            # Break the inner loop if 3 seconds have passed
            if time.time() - start_time >= 3:
                break

    # Calculate the mean of the z-axis force values collected in 3 seconds
    if len(z_axis_forces) > 0:
        wt_pre = np.mean(z_axis_forces)  # Use NumPy to calculate the mean
        print("Mean force over 3 seconds (z-axis):", wt_pre)
    else:
        print("No force data collected")
        return
    
               
    #for detected_load in fs.record(stop=1):
     #    wt_pre=detected_load[2]  # Accessing the third element directly during iteration

    print("current weight ",wt_pre)


    global wt_og
    picked_wt=wt_pre-wt_og
    print("Picked up weight in N is ",picked_wt)
    wt_drop=picked_wt-(mass*0.01)
    print("F sensor value to be reduced ",wt_drop)
    i=0

    global cf

    

    # Loop through the detected load values in real-time
    for detected_load in fs.record(stop=10):
        #initialise the overall picked angle here and then just go back a bit with every motion. picked rotatoin angle is fixed. 90+30+30+30. go back 10 deg from 180 until the force thing. do we do relative here?
        if detected_load[2]<(wt_pre-(wt_drop-cf)):
            print("final weight of spaghetti is ",detected_load[2])
            break
        else:
            #controllerClient.controlGripper("bothServos_up", d2,d2)
            i=i+1
            dm.pick_up((p_angle+3*p_steps)-(10*i),5)
            time.sleep(4)

        print("Force in the z-axis:", detected_load[2])

    #controllerClient.controlGripper("bothServos_down", 60,60)
    dm.pick_up(360)
    print("finalllll weight of spaghetti is ",detected_load[2])
    #set_initial=d1+(3*d3)-(d2*i)+60
    #print("angle to be reset is",set_initial)
    time.sleep(10)
    #controllerClient.controlGripper("bothServos_up", set_initial,set_initial)
"""
def drop_feedback_scale(target_wt):
    #drops the spaghetti using the weighing scale
    i=0
    # Replace with your actual port and baud rate
    #ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
    while True:
        if ser.in_waiting > 0:

            ser.reset_input_buffer()  # Clears old data before reading
            weight = ser.readline().decode('utf-8').strip()
            
            print(f"Weight: {weight} grams")  # Modify based on your scale's output format
            
            #raw_data = ser.readline().decode('utf-8').strip()
            # Remove the 'g' (or any unwanted characters)
            clean_data = weight.replace('g', '')

            #  Handle empty or invalid data before converting to float
            if not clean_data:  # Check if empty
                print("Warning: Received empty weight data, skipping...")
                continue

            try:
                value = float(clean_data)  # Convert to float
            except ValueError:
                print(f"Warning: Invalid weight data '{clean_data}', skipping...")
                continue  # Skip this iteration and read the next value


            #value=float(clean_data)
            
            if value<=target_wt:
                print("rotate motors to drop food")
                i=i+1
                print(i)
                dm.pick_up((p_angle+3*p_steps)-(10*i),10)        #dropping food by 10 degrees and checking the weight
                #dm.pick_up(20*i,10)
                
            elif value>target_wt:
                print("Target weight reached, stop the motors")
                dm.pick_up((p_angle+4*p_steps),25)
                break
    


def depth(a):
    global cf
    if a<=50:
        delta_z=1*l
        cf=0.5
    elif a>50 and mass<85:
        delta_z=1.7*l
        cf=0.4
    elif a>85:
        delta_z=1.5*l
        cf=0.2
    return delta_z



"""
#----------------- Ikura---------------------------#

def iku_pick():

    success = plan_move(arm,[p11,p12],[3,4])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    print("starting the pickup")
    time.sleep(15)
    print("pickup over")
    success = plan_move(arm,[p11,p13,p14],[3,3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    time.sleep(30)

    success = plan_move(arm,[p13,p11],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)


"""
#----------------------Main body------------------------#


#How deep should the gripper go at every step. l=length of the bit/bucket
global mass
mass=30      #grams

delta_z=depth(mass)
print("delta z is ",delta_z)

#sending the incrmental depth to the pickup code
print("starting the picking process")
down_rotate_steps(delta_z)
time.sleep(2)

print("starting the final drop")
transfer_drop()
drop_feedback_scale(mass)

#time.sleep(2)




"""
iku_pick()
print("Ikura process over")
"""
print("-----------------process complete------------------")
dm.stop_motors()