#Extended position control of dynamixxel motors with speed control
#Created by Prashant on 11.02.2025
#We want to create diffferent motion sets here which can be called into other programs

import time
from dynamixel_sdk import *  # Dynamixel SDK

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

# ---------------------- PREDEFINED MOTIONS ----------------------

def pick_up(angle=90, speed=10):
    """Moves both motors in opposite directions to simulate a pick-up motion."""
    enable_torque()
    set_profile_velocity(speed)
    set_position(DXL_ID_1, angle)   # Motor 1 moves clockwise
    set_position(DXL_ID_2, -angle)  # Motor 2 moves counterclockwise
    time.sleep(2)  # Allow motion to complete

def drop_down(angle=90, speed=20):
    """Moves both motors back to the original position (reverse of pick-up)."""
    enable_torque()
    set_profile_velocity(speed)
    set_position(DXL_ID_1, -angle)  # Motor 1 moves counterclockwise
    set_position(DXL_ID_2, angle)   # Motor 2 moves clockwise
    time.sleep(2)  


def rotate_both(angle1,angle2,speed=20):
    """Rotates both motors in the same direction for synchronization."""
    enable_torque()
    set_profile_velocity(speed)
    set_position(DXL_ID_1, angle1)
    set_position(DXL_ID_2, angle2)
    time.sleep(3)

# Disable torque after motion
def stop_motors():
    disable_torque()
    print(" Motors stopped.")

#Homing the motors
set_position(DXL_ID_1, 0)  # Motor 1 moves counterclockwise
set_position(DXL_ID_2, -0)
print("--------Homing done on both motors--------")
time.sleep(8)

angle=790
pick_up(angle,10)

time.sleep(6)


pick_up(10,10)
"""
for a in range(angle-10,angle-150,-20):
    print(a)
    pick_up(a,20)
    time.sleep(2)
"""
