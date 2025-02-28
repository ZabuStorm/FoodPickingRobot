#Controlling the dynamixxel motors via python rather than the ROS package
#Created by Prashant on 10.02.2025


import os
import sys
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control Table Addresses (for XL330-M288-T)
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_GOAL_VELOCITY      = 104
ADDR_PRESENT_VELOCITY   = 128
ADDR_OPERATING_MODE     = 11
EXTENDED_POSITION_MODE = 4  # Value for extended position control
# Values
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
VELOCITY_MODE           = 1
POSITION_MODE           = 3

# DYNAMIXEL IDs
DXL_ID_1 = 1  # First motor
DXL_ID_2 = 2  # Second motor

# Communication Settings
BAUDRATE = 57600  # Default for XL330
DEVICENAME = "/dev/ttyUSB1"  # Adjust this based on your system (e.g., COM3 on Windows)

# Protocol version
PROTOCOL_VERSION = 2.0

# Initialize PortHandler & PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open Port
if not portHandler.openPort():
    print("Failed to open the port!")
    sys.exit()

# Set Baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate!")
    sys.exit()

# Function to enable torque
def enable_torque(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# Function to set velocity mode
def set_velocity_mode(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
    enable_torque(dxl_id)

# Function to set position mode
def set_position_mode(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, POSITION_MODE)
    enable_torque(dxl_id)

# Function to set motor speed
def set_speed(dxl_id, speed):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, speed)

# Function to set motor position
def set_position(dxl_id, position):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)

#convert positions 0-4095 to degrees
def set_position_in_degrees(dxl_id, degrees):
    position_value = int((degrees * 4095) / 360)  # Convert degrees to position units
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position_value)

def get_position_in_degrees(dxl_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return (pos * 360) / 4095  # Convert position units to degrees

# Control speed in position mode (input speed in RPM)
def set_profile_velocity(dxl_id, rpm):
    velocity_value = int(rpm / 0.229)  # Convert RPM to raw value
    if velocity_value > 32767:
        velocity_value = 32767  # Limit max speed
    packetHandler.write4ByteTxRx(portHandler, dxl_id, 112, velocity_value)  # Set Profile Velocity


def set_position_with_speed(dxl_id, degrees, rpm):
    set_profile_velocity(dxl_id, rpm)  # Set speed
    position_value = int((degrees * 4095) / 360)  # Convert degrees to position units
    packetHandler.write4ByteTxRx(portHandler, dxl_id, 116, position_value)  # Set position





# Set Extended Position Mode
packetHandler.write1ByteTxRx(portHandler, 1, ADDR_OPERATING_MODE, EXTENDED_POSITION_MODE)
packetHandler.write1ByteTxRx(portHandler, 2, ADDR_OPERATING_MODE, EXTENDED_POSITION_MODE)

"""
# Set Profile Velocity (speed)
def set_profile_velocity(dxl_id, rpm):
    velocity_value = int(rpm / 0.229)  # Convert RPM to raw unit
    if velocity_value > 32767:
        velocity_value = 32767  # Limit max speed
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity_value)

# Set Profile Acceleration
def set_profile_acceleration(dxl_id, acceleration):
    acceleration_value = int(acceleration / 0.229)  # Convert acceleration to raw unit
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, acceleration_value)
"""

def set_position_extended(dxl_id, degrees):
    position_value = int((degrees * 4095) / 360)  # Convert degrees to position units
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position_value)

# Set speed and acceleration
#set_profile_velocity(1, 30)  # 30 RPM for Motor 1
#set_profile_acceleration(1, 10)  # Set acceleration for smoother motion




# Move Motor 1 Clockwise to 720° (2 full turns)
set_position_extended(1, 720)  

# Move Motor 1 Counterclockwise to -540° (1.5 turns counterclockwise)
set_position_extended(1, -540)


"""
# Initialize Motors
set_velocity_mode(DXL_ID_1)
set_velocity_mode(DXL_ID_2)
"""
# Set Speed Example
#set_speed(DXL_ID_1, 10)  # Set motor 1 speed
#set_speed(DXL_ID_2, -10) # Set motor 2 speed (negative for reverse direction)

#time.sleep(5)  # Run for 5 seconds

# Change to Position Mode and Move Motors
#set_position_mode(DXL_ID_1)
#set_position_mode(DXL_ID_2)

#set_position(DXL_ID_1, 1000)  # Move motor 1 to position 1000
#set_position(DXL_ID_2, 2000)  # Move motor 2 to position 2000
#set_position_in_degrees(1, 30)  # Move motor 1 to 90 degrees
#set_position_in_degrees(2, 60) # Move motor 2 to 180 degrees

#set_position_with_speed(1, 200, 10)  # Move Motor 1 to 90° at 30 RPM
#set_position_with_speed(2, -200, 10) # Move Motor 2 to 180° at 10 RPM

#set_position_in_degrees(1, 720)  # Move Motor 1 to 720° (2 full turns)
#set_position_in_degrees(2, -540) # Move Motor 2 to -540° (1.5 turns counterclockwise)

time.sleep(3)

# Read and print actual motor positions
pos1 = get_position_in_degrees(1)
pos2 = get_position_in_degrees(2)
print(f"Motor 1 Position: {pos1}°")
print(f"Motor 2 Position: {pos2}°")


time.sleep(3)

# Disable Torque and Close Port
packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("execeution completed")
