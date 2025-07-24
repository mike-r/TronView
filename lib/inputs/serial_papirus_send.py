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
import time
#from osgeo import osr
from lib.common import shared

class serial_papirus_send(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "Serial_PaPiRus_Send"  # set name
        self.update = True
        self.tx_count = 0
        self.loop_count = 0
        self.isPlaybackMode = False

        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()

    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        
        if(self.PlayFile!=None and self.PlayFile!=False):
            pass
        else:
            #self.efis_data_format = hud_utils.readConfig(self.name, "format", "none")
            self.efis_data_port = hud_utils.readConfig(self.name, "port", "/dev/ttyUSB0")
            self.efis_data_baudrate = hud_utils.readConfigInt(
                self.name, "baudrate", 9600
            )

            # open serial connection to Pi Zero with PaPiRus display.
            self.ser = serial.Serial(
                port=self.efis_data_port,
                baudrate=self.efis_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=3,
                write_timeout=0
            )

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

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            return dataship
            # Read until we find a message start character ($ or !)
        x = 0
        while x != ord('!'):
            
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
                1

# Build text string to send to PaPiRus display pi
            if self.update or self.tx_count > 10:
                if self.targetData.src_alt != None:
                    targetData_str = str(self.targetData.src_alt)
                    hobbs_str = targetData_str.zfill(5)  # Pad with leading zeros to 5 digits
                    print("hobbs_str = ", hobbs_str)
                else:
                    hobbs_str = "10234"
                smoke_str = "+0234G"      #   "+nnnnG"
                fuel_remain_str = "678"
                papirus_str = '!41' + smoke_str + hobbs_str + fuel_remain_str + '\r\n'
                if self.loop_count < 10:  print("To  Papirus:", papirus_str)
                papirus_bytes = papirus_str.encode()
                print(papirus_bytes)
                try:
                    self.ser.write(papirus_bytes)         # Send data to PaPiRus
                except Exception as e:
                    print(e)
                    print("Unexpected error in write to PaPiRus: ", e)
                #self.update = False
            self.tx_count = 20
            self.loop_count = self.loop_count + 1

            if len(t) != 0:
                x = ord(t)
            else:
                if self.isPlaybackMode:  # if no bytes read and in playback mode, reset file pointer
                    self.ser.seek(0)
                return dataship

        return dataship 

#  Version 1.0 testing
        print("serial_PaPiRus.py Version 1.0.Testing")

        print("TargetData: src_alt = ", self.targetData.src_alt)
        print("TargetData: src_gps = ", self.targetData.src_gps)





    # close this data input 
    def closeInput(self,dataship: Dataship):
        if self.isPlaybackMode:
            self.ser.close()
        else:
            self.ser.close()

    print("Did it work?")