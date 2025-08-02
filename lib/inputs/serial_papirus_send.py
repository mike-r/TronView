#!/usr/bin/env python

#################################################
# Module: Serial to PaPiRus for Display 
# Zap 2025
# 
# Raspberry Pi 
# /dev/ttyACM0 on Pi 3B, and Pi 5 is a micro-USB cable plugged into a Pi-Zero OTG port
# /dev/ttyAMA0 for Pi-4

# To check serial ports use:   dmesg | grep tty
# To check I2C devices:        sudo i2cdetect -y 0

# Write Serial data to PaPiRus Pi Zero
# Format of data stream is:
# !41+ssssGhhhhhfff
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "hhhhh"    is the Total Time (Hobbs Time) in tenths of hours from Dynon EMS
# "fff"      is the Total Fuel Remaining in tents of gallons
# Send TronView Pi's IP Address with:
# !51aaa.bbb.ccc.ddd   IP Address of Automationhat Pi
#

import serial
from time import sleep
import time
import sys
import os
import socket
from ._input import Input
from lib.modules._module import Module
from lib import hud_utils
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_targets import TargetData, Target
from lib.common.dataship.dataship_gps import GPSData
from lib.common.dataship.dataship_imu import IMUData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from lib.common.dataship.dataship_air import AirData
from lib.common.dataship.dataship_analog import AnalogData
from lib.common import shared

class serial_papirus_send(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "Serial_PaPiRus_Send"  # set name
        self.update = True
        self.tx_count = 0
        self.isPlaybackMode = False
        self.tv_ipaddr_bytes = None
        self.retry_time = time.time()
        self.comms_ok = False
        self.engine_status = 's'  # Default engine status is stopped
        self.tv_data_one = None
        self.tv_data_two = None
        self.tv_data_three = None

        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()
        
        print("Welcome to TronView serial sender to a PaPiRus e-paper display on another RaPi`", sep=' ', end='\n\n\n') 

    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.initPapirus(dataship)  # Initialize the PaPiRus display settings
        if(self.PlayFile!=None and self.PlayFile!=False):
            pass
        else:
            pass

        # set the data to the first item in the list.
        if len(shared.Dataship.targetData) > 0:
            self.targetData = shared.Dataship.targetData[0]
        if len(shared.Dataship.gpsData) > 0:
            self.gpsData = shared.Dataship.gpsData[0]
        if len(shared.Dataship.imuData) > 0:
            self.imuData = shared.Dataship.imuData[0]
        if len(shared.Dataship.engineData) > 0:
            self.engineData = shared.Dataship.engineData[0]
        if len(shared.Dataship.fuelData) > 0:
            self.fuelData = shared.Dataship.fuelData[0]
        if len(shared.Dataship.airData) > 0:
            self.airData = shared.Dataship.airData[0]
        if len(shared.Dataship.analogData) > 0:
            self.analogData = shared.Dataship.analogData[0]

            # open serial connection to Pi Zero with PaPiRus display.
            self.ser = serial.Serial(
                port=self.papirus_data_port,
                baudrate=self.papirus_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=3,  # Set a timeout for reading
                write_timeout=5
            )

        # create analog data object.
        self.analogData = AnalogData()
        self.analogData.name = self.name
        self.index = len(dataship.analogData)
        self.analogData.id = self.name + "_" + str(self.index)
        dataship.analogData.append(self.analogData)

        # create a empty imu object.
        self.imuData = IMUData()
        self.imuData.name = "stratux_papirus_imu"
        self.imu_index = len(dataship.imuData)  # Start at 0
        self.imuData.id = "stratux_papirus_imu"+str(self.imu_index)
        dataship.imuData.append(self.imuData)
        self.last_read_time = time.time()


        # set the target data and gps data to the first item in the list.
        if len(shared.Dataship.targetData) > 0:
            self.targetData = shared.Dataship.targetData[0]
        if len(shared.Dataship.gpsData) > 0:
            self.gpsData = shared.Dataship.gpsData[0]
        if len(shared.Dataship.imuData) > 0:
            self.imuData = shared.Dataship.imuData[0]

        # Get the IP address of the TronView Pi
        try:
            gw = os.popen("ip -4 route show default").read().split()
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((gw[2], 0))
            tv_ipaddr = s.getsockname()[0]
            gateway = gw[2]
            host = socket.gethostname()
            print ("IP:", tv_ipaddr, " GW:", gateway, " Host:", host)
        except:
            print("Error: Unable to get IP address")
        # Send TronView Pi's IP Address to PaPiRus display Pi:
        tv_ipaddr_str = "!51" + tv_ipaddr + "\r\n"
        self.tv_ipaddr_bytes = tv_ipaddr_str.encode()
        print("Sending PaPiRus message: ", tv_ipaddr_str)     

#  Send message to PaPiRus display pi every 5 seconds until we get a response
#  to signal that comms are OK
        self.retry_time = time.time()
        
        while True:
            if time.time() - self.retry_time > 60: break
            if self.comms_ok:
                break
            else:
                self.sendIPaddrToPapirus()
            sleep(5)  # Wait for 5 seconds before retrying

    def initPapirus(self, dataship: Dataship):
        # Initialize the PaPiRus display settings
        self.papirus_data_port = hud_utils.readConfig(self.name, "port", "/dev/ttyACM0")
        self.papirus_data_baudrate = hud_utils.readConfigInt(self.name, "baudrate", 9600)
        self.registration = hud_utils.readConfig(self.name, "registration", "N12345")


        self.tv_label1 = hud_utils.readConfig(self.name, "PaPirus_Label_1", "none")
        tv_data1_name = hud_utils.readConfig(self.name, "TronView_PaPiRus_1", "none")
        self.tv_data1_exec = "self.tv_data_one = self." + tv_data1_name
        print("tv_data_one_exec: ", self.tv_data1_exec)
        exec(self.tv_data1_exec)  # Evaluate the string to get the value
        print("self.tv_data_one: ", self.tv_data_one, " ", self.tv_label1)

        self.tv_label2 = hud_utils.readConfig(self.name, "PaPirus_Label_2", "none")
        tv_data2_name = hud_utils.readConfig(self.name, "TronView_PaPiRus_2", "none")
        self.tv_data2_exec = "self.tv_data_two = self." + tv_data2_name
        print("tv_data_two_exec: ", self.tv_data2_exec)
        exec(self.tv_data2_exec)  # Evaluate the string to get the value
        print("self.tv_data_two: ", self.tv_data_two, " ", self.tv_label2)
        
        self.tv_label3 = hud_utils.readConfig(self.name, "PaPirus_Label_3", "none")
        tv_data3_name = hud_utils.readConfig(self.name, "TronView_PaPiRus_3", "none")
        self.tv_data3_exec = "self.tv_data_three = self." + tv_data3_name
        print("tv_data_three_exec: ", self.tv_data3_exec)
        exec(self.tv_data3_exec)  # Evaluate the string to get the value
        print("self.tv_data_three: ", self.tv_data_three, " ", self.tv_label3)

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            return dataship
            # Find the IP Address of the TronView Pi and send it to the PaPiRus display Pi.
            # Then read data out of the Dataship and send it to the PaPiRus display.
    
        self.updateEngineStatus(dataship)    
        while True:       
# Build text string to send to PaPiRus display pi
            if self.tx_count > 20:
                self.tx_count = 0

                exec(self.tv_data1_exec)  # Evaluate the string to get the value
                exec(self.tv_data2_exec)
                exec(self.tv_data3_exec)
                papirus1_str = self.tv_label1 + "," + str(self.tv_data_one)
                papirus2_str = self.tv_label2 + "," + str(self.tv_data_two)
                papirus3_str = self.tv_label3 + "," + str(self.tv_data_three)
                print()
                print("papirus1_str = ", papirus1_str)
                print("papirus2_str = ", papirus2_str)
                print("papirus3_str = ", papirus3_str)
                print()

    # Pad with leading zeros to 5 digits

                papirus_str = '!4#,' + self.registration + "," + papirus1_str + "," + papirus2_str + "," + papirus3_str + "," + self.engine_status + '\r\n'
                papirus_bytes = papirus_str.encode()
                print("PaPiRus Bytes = ", papirus_bytes)
                try:
                    self.ser.write(papirus_bytes)         # Send data to PaPiRus
                    if not self.comms_ok:
                        sleep(.1)
                        self.sendIPaddrToPapirus()
                except Exception as e:
                    print(e)
                    print("Unexpected error in write to PaPiRus: ", e)
            self.tx_count += 1

            if self.isPlaybackMode:  # if no bytes read and in playback mode, reset file pointer
                self.ser.seek(0)
            return dataship
        return dataship 

    def sendIPaddrToPapirus(self):
        try:
            self.ser.write(self.tv_ipaddr_bytes)         # Send data to PaPiRus
            sleep(0.5)  # Wait for 0.5 seconds before recieving reply message
        except Exception as e:
            print(e)
            print("Unexpected error in write to PaPiRus: ", e)
        papirus_bytes = self.ser.read_until(b'\r\n', None)
        if papirus_bytes == b'':
            print("No data received from PaPiRus...")
            self.comms_ok = True  # Assume comms are OK even if no data received
            print("Assuming comms are OK with PaPiRus display.")
            return
        else:
            papirus_str = papirus_bytes.decode().strip()
            print("Received IP Address from PaPiRus: ", papirus_str)
            self.comms_ok = True
             
    # close this data input 
    def closeInput(self,dataship: Dataship):
        if self.isPlaybackMode:
            self.ser.close()
        else:
            self.ser.close()
            
    def updateEngineStatus(self, dataship: Dataship):
        if not hasattr(self, 'engineData'):
            self.engineData = dataship.engineData[0]
        if not hasattr(self, 'old_engine_status_str'):
            self.old_engine_status_str = ''
        if not hasattr(self, 'old_OilPress'):
            self.old_OilPress = 0

        self.old_engine_status = self.engine_status  # Set old engine status to current status
        if self.engineData.OilPress != None:
            if self.engineData.OilPress > 15:
                self.engine_status = "r"  # running
                if dataship.debug_mode > 0: print("Engine is running, Oil Pressure: ", self.engineData.OilPress)
            else:
                self.engine_status = "s"  # stopped
            self.new_OilPress = self.engineData.OilPress
            if self.new_OilPress != self.old_OilPress:
                self.old_OilPress = self.new_OilPress
                self.update = True
