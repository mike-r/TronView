#!/usr/bin/env python

#################################################
# Module: Serial to another Raspberry Pi for Display on e-paper
# Zap 2025
# 
# Raspberry Pi 
# /dev/ttyACM0 on Pi 3B, and Pi 5 is a micro-USB cable plugged into a Pi-Zero OTG port
# /dev/ttyAMA0 for Pi-4

# To check serial ports use:   dmesg | grep tty
# To check I2C devices:        sudo i2cdetect -y 0

# Write Serial data to Pi
# Format of data stream is:
# !41+ssssGhhhhhfff
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "hhhhh"    is the Total Time (Hobbs Time) in tenths of hours from Dynon EMS
# "fff"      is the Total Fuel Remaining in tents of gallons
# Send TronView Pi's IP Address with:
# !51aaa.bbb.ccc.ddd   IP Address of Automationhat Pi
#

import serial
import time
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

class serial_rapi_send(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "Serial_RaPiRmt_Send"  # set name
        self.update = True
        self.tx_count = 0
        self.isPlaybackMode = False
        self.tv_ipaddr_bytes = None
        self.retry_time = time.time()
        self.comms_ok = False
        self.serialCommsOK = False
        self.engine_status = 's'  # Default engine status is stopped
        self.tv_data_one = 0
        self.tv_data_two = 0
        self.tv_data_three = 0
        self.tv_data_one_old = 0
        self.tv_data_two_old = 0
        self.tv_data_three_old = 0

        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()
        
        print("Welcome to TronView serial sender to a remote Raspberry Pi for display`", sep=' ', end='\n\n\n') 

    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.initRaPiRmt(dataship)  # Initialize the remote RaPi  display settings
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

            # open serial connection to Pi with display.
            self.connectToRaPiRmt()  # Try to connect to the remote RaPi display if not already connected


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
        else:
            # Send TronView Pi's IP Address to remote RaPi and display:
            tv_ipaddr_str = "!51" + tv_ipaddr + "\r\n"
            self.tv_ipaddr_bytes = tv_ipaddr_str.encode()
            print("Sending remote RaPi message (bytes): ", self.tv_ipaddr_bytes)
            if self.serialCommsOK and not self.comms_ok: self.sendIPaddrToRaPiRmt(dataship)

    def initRaPiRmt(self, dataship: Dataship):
        # Initialize the remote RaPi display settings
        self.rapi_rmt_data_port = hud_utils.readConfig(self.name, "port", "/dev/ttyACM0")
        self.rapi_rmt_data_baudrate = hud_utils.readConfigInt(self.name, "baudrate", 9600)
        self.registration = hud_utils.readConfig(self.name, "registration", "N12345")


        self.tv_label1 = hud_utils.readConfig(self.name, "RaPiRmt_Label_1", "None")
        self.tv_data1_name = hud_utils.readConfig(self.name, "TronView_RaPiRmt_1", "None")
        self.tv_data1_exec = "self.tv_data_one = self." + self.tv_data1_name
        print("tv_data1_exec: ", self.tv_data1_exec)
        if self.tv_data1_name == "None":
            print("No data source defined for tv_data_one.  Set TronView_RaPiRmt_1 in config.cfg to dataship variable you want to display.")
        else:
            exec(self.tv_data1_exec)  # Evaluate the string to get the value
            print("self.tv_data_one: ", self.tv_data_one, " ", self.tv_label1)
            if self.tv_data_one == None:
                self.tv_data_one = 0
            self.tv_data_one_old = self.tv_data_one
            print("self.tv_data_one: ", self.tv_data_one, " ", self.tv_label1)

        self.tv_label2 = hud_utils.readConfig(self.name, "RaPiRmt_Label_2", "None")
        self.tv_data2_name = hud_utils.readConfig(self.name, "TronView_RaPiRmt_2", "None")
        self.tv_data2_exec = "self.tv_data_two = self." + self.tv_data2_name
        print("tv_data2_exec: ", self.tv_data2_exec)
        if self.tv_data2_name == "None":
            print("No data source defined for tv_data2.  Set TronView_RaPiRmt_2 in config.cfg to dataship variable you want to display.")
        else:
            exec(self.tv_data2_exec)  # Evaluate the string to get the value
            print("self.tv_data_two: ", self.tv_data_two, " ", self.tv_label2)
            if self.tv_data_two == None:
                self.tv_data_two = 0
            self.tv_data_two_old = self.tv_data_two
            print("self.tv_data_two: ", self.tv_data_two, " ", self.tv_label2)

        self.tv_label3 = hud_utils.readConfig(self.name, "RaPiRmt_Label_3", "None")
        self.tv_data3_name = hud_utils.readConfig(self.name, "TronView_RaPiRmt_3", "None")
        self.tv_data3_exec = "self.tv_data_three = self." + self.tv_data3_name
        print("tv_data3_exec: ", self.tv_data3_exec)
        if self.tv_data3_name == "None":
            print("No data source defined for tv_data3.  Set TronView_RaPiRmt_3 in config.cfg to dataship variable you want to display.")
        else:
            exec(self.tv_data3_exec)  # Evaluate the string to get the value
            print("self.tv_data_three: ", self.tv_data_three, " ", self.tv_label3)
            if self.tv_data_three == None:
                self.tv_data_three = 0
            self.tv_data_three_old = self.tv_data_three
            print("self.tv_data_three: ", self.tv_data_three, " ", self.tv_label3)

    def sendIPaddrToRaPiRmt(self, dataship: Dataship):
        try:
            waiting_bytes = self.ser.in_waiting    # int: bytes in input buffer
            out_bytes = self.ser.out_waiting       # int: bytes in output buffer (if supported)
            print(f"Input buffer: {waiting_bytes} bytes")
            if hasattr(self.ser, 'out_waiting'):
                print(f"Output buffer: {out_bytes} bytes")
            self.ser.write(self.tv_ipaddr_bytes)         # Send data to remote Pi
            print("sent IP Address to remote pi")
            #sleep(0.5)  # Wait for 0.5 seconds before recieving reply message
        except Exception as e:
            #if dataship.debug_mode>0: print("Unexpected error in write to remote Pi: ", e)
            print("Unexpected error in write to remote Pi: ", e)
        display_bytes = self.ser.read_until(b'\r\n', None)
        if display_bytes == b'':
            if dataship.debug_mode>0: print("No data received from remote Pi...")
            self.comms_ok = False  # Assume comms are not OK if no data received
            return
        else:
            display_str = display_bytes.decode().strip()
            print("Received from remote Pi: ", display_str)
            self.comms_ok = True
        return
             
    # close this data input 
    def closeInput(self,dataship: Dataship):
        if self.isPlaybackMode:
            self.ser.close()
        else:
            self.ser.close()
            
    def updateEngineStatus(self, dataship: Dataship):
        if not hasattr(self, 'engineData'):
            self.engineData = dataship.engineData[0]
#        if not hasattr(self, 'old_engine_status_str'):
#            self.old_engine_status_str = ''
        if not hasattr(self, 'old_OilPress'):
            self.old_OilPress = 0

        self.old_engine_status = self.engine_status  # Set old engine status to current status
        if self.engineData.OilPress != None:
            if self.engineData.OilPress > 15:     # If Oil Pressure is greater than 15 psi, engine is running
                self.engine_status = "r"  # running
                if dataship.debug_mode > 0: print("Engine is running, Oil Pressure: ", self.engineData.OilPress)
            else:
                self.engine_status = "s"  # stopped
            self.new_OilPress = self.engineData.OilPress
            if self.new_OilPress != self.old_OilPress:
                self.old_OilPress = self.new_OilPress
                self.update = True
                
    def connectToRaPiRmt(self):
        # Try to connect to the remote RaPi display if not already connected
        if not self.serialCommsOK:
            try:
                self.ser = serial.Serial(
                port=self.rapi_rmt_data_port,
                baudrate=self.rapi_rmt_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1,       # Set a timeout for reading
                #write_timeout=0.1  # Set a timeout for writing
            )
                self.serialCommsOK = True
                print("Connected to remote RaPi display.")
                print("Serial Port opened: ", self.rapi_rmt_data_port, " at baudrate: ", self.rapi_rmt_data_baudrate)

            except serial.SerialException as e:
                print("Error opening serial port: ", e)
                print("Is the USB cable to the remote RaPi plugged in?")
                self.serialCommsOK = False
                self.comms_ok = False
                time.sleep(2)  # Wait for 2 seconds before retrying
        return

    #############################################
    ## Method: readMessage
    def readMessage(self, dataship: Dataship):        
        if dataship.errorFoundNeedToExit:
            return dataship
            # Find the IP Address of the TronView Pi and send it to the remote Raspberry Pi display Pi.
            # Then read data out of the Dataship and send it to the remote Raspberry Pi display.
    
        self.updateEngineStatus(dataship)
        
        if not self.serialCommsOK:
            self.connectToRaPiRmt()  # Try to connect to the remote Raspberry Pi display if not already connected
            return dataship  # If serial comms are not OK, return the dataship without sending data

        if self.tv_data1_name == "None" or self.tv_data_one == None:
            self.tv_data_one = 0.00
        else:
            exec(self.tv_data1_exec)  # Evaluate the string to get the value
            if self.tv_data_one == None: self.tv_data_one = 0.00
            if round(self.tv_data_one,1) != round(self.tv_data_one_old,1):
                print("tv_data_one changed from ", self.tv_data_one_old, " to ", self.tv_data_one)
                self.update = True
                self.tv_data_one_old = self.tv_data_one

        if self.tv_data2_name == "None" or self.tv_data_two == None:
            self.tv_data_two = 0.00
        else:
            exec(self.tv_data2_exec)
            if self.tv_data_two == None: self.tv_data_two = 0.00
            if round(self.tv_data_two,1) != round(self.tv_data_two_old,1):
                print("tv_data_two changed from ", self.tv_data_two_old, " to ", self.tv_data_two)
                self.update = True
                self.tv_data_two_old = self.tv_data_two

        if self.tv_data3_name == "None" or self.tv_data_three == None:
            self.tv_data_three = 0.00
        else:
            exec(self.tv_data3_exec)
            if self.tv_data_three == None: self.tv_data_three = 0.00
            if round(self.tv_data_three,1) != round(self.tv_data_three_old,1):
                print("tv_data_three changed from ", self.tv_data_three_old, " to ", self.tv_data_three)
                self.update = True
                self.tv_data_three_old = self.tv_data_three
        
        display1_str = self.tv_label1 + "," + str(self.tv_data_one)
        display2_str = self.tv_label2 + "," + str(self.tv_data_two)
        display3_str = self.tv_label3 + "," + str(self.tv_data_three)
        if dataship.debug_mode>0: 
            print()
            print("display1_str = ", display1_str)
            print("display2_str = ", display2_str)
            print("display3_str = ", display3_str)
            print("Analog Data[0] = ", self.analogData.Data[0])
            print("Analog Data[1] = ", self.analogData.Data[1])
            print()
        
        # Create the string to send to the remote RaPi display
        display_str = '!4#,' + self.registration + "," + display1_str + "," + display2_str + "," + display3_str + "," + self.engine_status + '\r\n'
        display_bytes = display_str.encode()
        if dataship.debug_mode>0: print("Remote RaPi Bytes = ", display_bytes)
        try:
            if self.update:
                self.update = False
                waiting_bytes = self.ser.in_waiting    # int: bytes in input buffer
                out_bytes = self.ser.out_waiting       # int: bytes in output buffer (if supported)
                print(f"Input buffer: {waiting_bytes} bytes")
                if hasattr(self.ser, 'out_waiting'):
                    print(f"Output buffer: {out_bytes} bytes")
                self.ser.write(display_bytes)         # Send data to Remote RaPi
                print("write to RaPi OK")
            if not self.comms_ok:
                #sleep(.1)
                print("Comms Not OK")
                self.sendIPaddrToRaPiRmt(dataship)
        except Exception as e:
            if dataship.debug_mode>0: print("Unexpected error in write to remote Pi: ", e)
            print("Unexpected error in write to remote Pi: ", e)

        if self.isPlaybackMode:  # if no bytes read and in playback mode, reset file pointer
            self.ser.seek(0)
        return dataship


                