import time
from dynamixel_sdk import *
import motor_control as sm
import os
import sys

# ---------------------- SETUP PARAMETERS ----------------------
DEVICENAME = "/dev/ttyUSB1"  # Change based on your system
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




print("Performing Pick-Up Motion")
sm.pick_up(90, 10)
time.sleep(2)
print("performing dropping")
sm.pick_up(0,20)