#Author: Prashant Kumar
#Created:22.07.2024
#Purpose: To make small adjustments in the bit position
import sys

# NX-ROS Series Package
from nxr_001_ftcg_client import controllerIcraClient

import timeit
start = timeit.default_timer()

import time
import datetime
import numpy as np
np.set_printoptions(suppress=True)
import os

#--------------------------------------------------------------------------------------

import time
from dynamixel_sdk import *  # Dynamixel SDK

# ---------------------- SETUP PARAMETERS ----------------------
DEVICENAME = "/dev/ttyUSB0"  # Change based on your system
BAUDRATE = 57600

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

EXTENDED_POSITION_MODE = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
"""
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64

# Define motor IDs
MOTOR_1_ID = 1
MOTOR_2_ID = 2
"""


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

def enable_torque():
    packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def disable_torque():
    packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

def set_profile_velocity(rpm):
    velocity_value = int(rpm / 0.229)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID_1, ADDR_PROFILE_VELOCITY, velocity_value)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID_2, ADDR_PROFILE_VELOCITY, velocity_value)

def set_position(dxl_id, degrees):
    position_value = int((degrees * 4095) / 360)
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position_value)


# Disable torque after motion
def stop_motors():
    disable_torque()
    print(" Motors stopped.")

#Homing the motors
def home():
    set_profile_velocity(10)
    set_position(DXL_ID_1, 0)  # Motor 1 moves counterclockwise
    set_position(DXL_ID_2, -0)
    print("--------Homing done on both motors--------")
    

def pick_up(angle=200, speed=20):
    """Moves both motors in opposite directions to simulate a pick-up motion."""
    enable_torque()
    set_profile_velocity(speed)
    set_position(DXL_ID_1, angle)   # Motor 1 moves clockwise
    set_position(DXL_ID_2, -angle)  # Motor 2 moves counterclockwise
    time.sleep(2)  # Allow motion to complete

def pick_down(angle=200, speed=20):
    """Moves both motors in opposite directions to simulate a drop motion."""
    enable_torque()
    set_profile_velocity(speed)
    set_position(DXL_ID_1, angle)   # Motor 1 moves clockwise
    set_position(DXL_ID_2, -angle)  # Motor 2 moves counterclockwise
    time.sleep(2)  # Allow motion to complete

"""
def slow_drop(steps):
    
    #To drop very tiny amounts of spaghetti .This is to check the minimum dropping ability of these grippers and hopefully notice a pattern. This is needed if the f-sensor thing doesn't work
    
    #controllerClient.controlGripper("open_gripper_slow")
        
     
    controllerClient.controlGripper("close_gripper_slow")
    time.sleep(1)
    #controllerClient.controlGripper("bothServos_down", d1,d1)
    dm.pick_up(p_angle)

    for i in range(4,1,-1):
        
        pick_up(p_angle+(4*p_steps)-(i*10))     #reduce 10 degrees from the final  pickup angle(fixed), to slowly drop spaghetti and observe the minim dropping ability
        #controllerClient.control


def enable_torque(motor_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to enable torque for Motor {motor_id}")
    else:
        print(f"Torque enabled for Motor {motor_id}")


def move_relative(angle):
    
    #Moves both motors by a relative angle.
    #Motor 1 moves clockwise, Motor 2 moves counterclockwise.
    
    # Convert degrees to Dynamixel position value
    DXL_RESOLUTION = 0.088  # 1 step = 0.088 degrees
    delta_pos = int(angle / DXL_RESOLUTION)

    # Read current positions
    present_pos_1, _, _ = packetHandler.read4ByteTxRx(portHandler, MOTOR_1_ID, ADDR_PRESENT_POSITION)
    present_pos_2, _, _ = packetHandler.read4ByteTxRx(portHandler, MOTOR_2_ID, ADDR_PRESENT_POSITION)

    # Calculate new target positions
    goal_pos_1 = present_pos_1 + delta_pos  # Motor 1 moves forward
    goal_pos_2 = present_pos_2 - delta_pos  # Motor 2 moves backward

    # Send commands to motors
    packetHandler.write4ByteTxRx(portHandler, MOTOR_1_ID, ADDR_GOAL_POSITION, goal_pos_2)
    packetHandler.write4ByteTxRx(portHandler, MOTOR_2_ID, ADDR_GOAL_POSITION, goal_pos_1)

    print(f"Moved motors by {angle} degrees. New positions: {goal_pos_1}, {goal_pos_2}")

"""    



"""
#Dynamixxel definitions
controllerClient = controllerIcraClient()
while not controllerClient.getConnectionStatus():
        controllerClient.connect()


controllerClient.setServoSpeed(50)

deg=40

def adjust_up():
    controllerClient.controlGripper("bothServos_up", deg,deg)

def adjust_down():
    controllerClient.controlGripper("bothServos_down", deg,deg)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run_task(sys.argv[1])
    else:
        print("Please specify a task.")    

for x in range(0,4):
    adjust_down()
    time.sleep(1)
    adjust_up()
    time.sleep(1)

#------------Main body----------------

a=input("up or down?")

if a=="up":
    adjust_up()

elif a=="down":
    adjust_down()

print("Belt adjusted by")
"""
import argparse
parser = argparse.ArgumentParser(description='')
parser.add_argument('--r','-r', type=str, help='action you want')
args = parser.parse_args()

       
if args.r == "home":
    home()
elif args.r == "up":
    pick_up(300,20)
elif args.r == "down":
    pick_down(220,10)
#elif args.r == "moveup":
#    move_relative(40)  # Moves both motors by 90 degrees in opposite directions



