# Simple Client Connection for NXR-001-FTCG-Controller FTC Gripper
# To use this program, be sure to run the launch file of NXR-001-FTCG first.
#
#
# * Command List (controlGripper)
# Gripper:
# - open_gripper_slow
# - close_gripper_slow
# - open_gripper
# - close_gripper
#
# Servos:
# - bothServos_up
# - bothServos_down
# - bothServos_upDown
# - bothServos_downUp
# - servo1_up
# - servo1_down
# - servo2_up
# - servo2_down
# - !Only for the servos, it also need additional variable (rotation)
# - Both servos need it. So, the example should be: 
# - controllerClient.controlGripper("bothServos_up", 30, 300)
# - Both servo will pull the timing belt up. Servo 1 will rotate 30 deg and Servo 2 will rotate 300 deg
# - Currently I set it to be able to turn for 4 times, (this one added to prevent the servos to rotate too much, if we already declared the value)
#
# * Setting Servo's Speed (setServoSpeed)
# - This function basically will adjust the speed of the servos by using percentage.

import socket
import time
import base64

import numpy as np

class controllerIcraClient(object):
    client_command_socket = None
    client_response_socket = None
    client_signal_socket = None
    
    port = [5000, 5001, 5002, 5003, 5004]
    host = '127.0.0.1'
    
    connectionStatus = False
    
    def __init__(self):
        # Initialization of this class also all of required socket port
        self.client_command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance    
        self.client_response_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance    
        self.client_signal_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance
        self.client_command2_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance        
        self.client_servoSpeed_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance     
        
    def connect(self):
        # Function to connect this class socket to the gripper controller socket
        self.client_command_socket.settimeout(5)
        self.client_response_socket.settimeout(5)
        self.client_signal_socket.settimeout(5)
        self.client_command2_socket.settimeout(5)
        self.client_servoSpeed_socket.settimeout(5)
        
        try:
            self.client_command_socket.connect((self.host, self.port[0]))
            self.client_response_socket.connect((self.host, self.port[1]))
            self.client_signal_socket.connect((self.host, self.port[2]))
            self.client_command2_socket.connect((self.host, self.port[3]))
            self.client_servoSpeed_socket.connect((self.host, self.port[4]))
        except:
            time.sleep(0.1)
            return False
        
        self.client_command_socket.settimeout(None)
        self.client_response_socket.settimeout(None)
        self.client_signal_socket.settimeout(None)
        self.client_command2_socket.settimeout(None)
        self.client_servoSpeed_socket.settimeout(None)
        
        self.connectionStatus = True
        
        return True
        
    def sendCommand(self, _typeCommand):
        # Send controlling command to the controller; this can be a command to control the gripper or the servos
        stringCommand = base64.b64encode(bytes(_typeCommand, 'ascii'))
        cmd_len = str(len(stringCommand))
        self.client_command_socket.sendall(cmd_len.encode('utf-8').ljust(64))
        self.client_command_socket.send(stringCommand)

    def sendServosCommand(self, _servosCommand):
        # Additional command if the command is to control the servos; which is the rotation degree of each servos
        servosCommand = np.array(_servosCommand)
        commandData = base64.b64encode(servosCommand)
        cmd_len = str(len(commandData))
        self.client_command2_socket.sendall(cmd_len.encode('utf-8').ljust(64))
        self.client_command2_socket.send(commandData)
            
    def sendOffSignal(self):
        # Send a signal to turn off the controller server
        if self.connectionStatus:
            signal = "server_off"
            stringSignal = base64.b64encode(bytes(signal, 'ascii'))
            sig_len = str(len(stringSignal))
            self.client_signal_socket.sendall(sig_len.encode('utf-8').ljust(64))
            self.client_signal_socket.send(stringSignal)
    
    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf
    
    def receiveResponse(self, sock):
        length = self.recvall(sock, 64)
        length1 = length.decode('utf-8')
        stringData = self.recvall(sock, int(length1))
        data = base64.b64decode(stringData).decode('utf-8')

        return data
    
    def sendSpeed(self, _speed):
        if self.connectionStatus:
            servosSpeed = np.array(_speed)
            commandData = base64.b64encode(servosSpeed)
            cmd_len = str(len(commandData))
            self.client_servoSpeed_socket.sendall(cmd_len.encode('utf-8').ljust(64))
            self.client_servoSpeed_socket.send(commandData)
    
    def controlGripper(self, _command, servo1Command = 285, servo2Command = 285):
        if self.connectionStatus:
            self.sendCommand(_command)
            if (_command) in ["bothServos_up", "bothServos_down", "bothServos_upDown", "bothServos_downUp", "servo1_up", "servo1_down", "servo2_up", "servo2_down"]:
                self.sendServosCommand([abs(servo1Command), abs(servo2Command)])
            try:
                response = self.receiveResponse(self.client_response_socket)
                response_bool = bool(int(response))
                return response_bool
            except:
                return False
        else:
            return False
        
    def setServoSpeed(self, _speed):
        if self.connectionStatus:
            self.sendSpeed(_speed)
            response = self.receiveResponse(self.client_response_socket)
            response_bool = bool(int(response))
            return response_bool
        else:
            return False
        
    def getConnectionStatus(self):
        return self.connectionStatus
