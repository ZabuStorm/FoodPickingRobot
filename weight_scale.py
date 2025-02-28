import serial
import csv
import time
target_wt=30

from dynamixel_sdk import *  # Dynamixel SDK
import motor_control as dm

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

p_angle=220                  
p_steps=60                  
d_angle=10 


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Port and Baudrate Set Successfully")
else:
    print("Failed to initialize port")
    exit()


with open("weight_log.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        
        # Write the header only if the file is empty
        if file.tell() == 0:
            writer.writerow(["Angle (degrees)", "Weight (grams)"])

i=0
# Replace with your actual port and baud rate
ser = serial.Serial(port="/dev/ttyUSB1", baudrate=9600, timeout=1)
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

            #  Proceed only if the conversion was successful
            #print(f"Processed Weight: {value} grams")



            #value=float(clean_data)
            
            if value<=target_wt:
                print("rotate motors to drop food")
                i=i+1
                print(i)
                #dm.pick_up((p_angle+3*p_steps)-(10*i),20)        #dropping food by 10 degrees and checking the weight
                dm.pick_up(20*i,10)
                
            elif value>target_wt:
                print("Target weight reached, stop the motors")
                break
        
        

print("dropping process completed")