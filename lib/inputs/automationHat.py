#!/usr/bin/env python

#################################################
# IO to and from a Pimoroni AutomationHat which has three SPDT relays, three 12-bit ADC inputs
# three sinking outputs and lots of cool status leds.  The hat is mounted on the Pi 5 via the
# GPIO pins.  The Automation Hat is used to increase the number of analog and digital inputs 
# when the EFIS is maxed out.  
# It will be first used to read the airplane's smoke tank level.
# 
# This module will also send data via RS-232 Serial to a PaPiRus e-Paper display.
# There is a mqtt message broker and client which may be used in the future to
# send and receive data via WiFi Vs RS-232.
#
# Zap 2025
# 
# /dev/ttyUSB0 is a USB to RS-232 converter to send and/or receive data.
# /dev/ttyACM0 on Pi 3B, and Pi 5 is a micro-USB cable plugged into a Pi-Zero OTG port
# /dev/ttyAMA0 for Pi-4

# To check serial ports use:   dmesg | grep tty
# To check I2C devices:        sudo i2cdetect -y 0
# To check MQTT status:        sudo systemctl status mosquitto

# To install and setup the MQTT Broker on the Pi:
#  sudo apt update
#  sudo apt upgrade
#  sudo apt install mosquitto mosquitto-clients
#  sudo systemctl enable mosquitto
# To allow anonymous access:
#  sudo nano /etc/mosquitto/mosquitto.conf
# Add the following lines to the end of the file:
#    allow_anonymous true
#    listener 1883 0.0.0.0
# Restart Mosquitto service to apply changes:
#  sudo service mosquitto restart

# To install MQTT library in Python
#  sudo pip3 install paho-mqtt --break-system-packages

# To install AutomationHat code:
# git clone https://github.com/pimoroni/automation-hat
# cd automation-hat
# ./install.sh
#
# pip freeze > requirments.txt
#    Must install as superuser to be able to run the AutomationHat code as superuser.
# sudo python3 -m pip install -r requirments.txt  --break-system-packages
# sudo pip3 install automationhat --break-system-packages


# Requires modification to /usr/lib/python3/dist-packages/automationhat/__init__.py
# Fork of modification is from: https://github.com/kiddigital/automation-hat


# Write Serial data to PaPiRus Pi
# Format of data stream is:
# !41+ssssGhhhhhfffr
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "hhhhh"    is the Total Time (Hobbs Time) in tenths of hours from Dynon EMS
# "fff"      is the Total Fuel Remaining in tents of gallons
# "r"       is engine status, r= running, r= stopped

import serial
from time import time
from .import _input_file_utils
import sys
import os
import socket
import statistics
from ._input import Input
from lib.modules._module import Module
from lib import hud_graphics
from lib import hud_utils
from lib import smartdisplay
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_targets import TargetData, Target
from lib.common.dataship.dataship_gps import GPSData
from lib.common.dataship.dataship_imu import IMUData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from lib.common.dataship.dataship_air import AirData
from lib.common.dataship.dataship_analog import AnalogData
import pygame
import math
import time
from lib.common import shared
from urllib.request import urlopen
import paho.mqtt.client as mqtt #import the client
import automationhat
from Adafruit_IO import Client, Feed, RequestError  # import Adafruit IO REST client.



class automationHat(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "AutomationHat"  # set name
        self.show_callsign = False
        self.show_details = False
        self.targetDetails = {} # keep track of details about each target.
        self.update = True
        self.loop_count = 0
        self.isPlaybackMode = False
        self.old_src_alt = -100
        self.old_hobbs_time = 0
        self.old_OilPress = 0
        self.old_FuelRemain = 0
        self.old_FuelLevel = 0
        self.old_smokeLevel = 0.0
        self.mqtt_broker_address_cloud = "broker.mqtt.cool"
        self.mqtt_broker_address_local = "localhost"
        self.engineData_hobbs_time_str = "00000"
        self.fuelData_FuelRemain_str = "0000"
        self.fuelData_FuelLevel_str = "000"
        self.engineData_OilPress_str = "000"
        self.analogData_smoke_remain_str = "0000"
        self.engine_status_str = "s"  # Default to stopped
        self.old_engine_status_str = "s"  # Default to stopped
        self.a0 = 0
        self.start_time = time.time()
        self.loop_time = time.time()  - 5 # Start loop_time 5 seconds in the past to allow first readMessage to run immediately.
        self.papirus_str = ""
        self.mqtt_cloud = False

        # Add smoothing configuration
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.debug_mode = 0
        
        # Add tracking of previous positions
        self.target_positions = {}          # Store previous positions for smoothing
        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()
        self.selectedTarget = None
        self.selectedTargetID = None
        
    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.initMqtt(dataship)
        self.initAutomationHat(dataship)
        if(self.PlayFile!=None and self.PlayFile!=False):
            pass
        else:
            self.papirus_data_port = hud_utils.readConfig(self.name, "port", "/dev/ttyUSB0")
            self.papirus_data_baudrate = hud_utils.readConfigInt(
                self.name, "baudrate", 9600
            )

            # open serial connection.
            try:
                self.ser = serial.Serial(
                port=self.papirus_data_port,
                baudrate=self.papirus_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=3,
                write_timeout=0
                )
                self.ser.write("\r\nFrom AutomationHat\r\n".encode())
                print("Serial port opened: ", self.papirus_data_port)
            except serial.SerialException as e:
                print("Error opening serial port: ", e)

        # create analog data object.
        self.analogData = AnalogData()
        self.analogData.name = self.name
        self.index = len(dataship.analogData)
        self.analogData.id = self.name + "_" + str(self.index)
        dataship.analogData.append(self.analogData)

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

        # Set up MQTT client

    def initMqtt(self, dataship: Dataship):
        # Initialize MQTT client
        print("Initializing MQTT client...")
        try:
            self.mqtt_client_cloud = mqtt.Client()
            self.mqtt_client_cloud.on_connect = self.cloud_on_connect
            self.mqtt_client_cloud.on_message = self.on_message
            self.mqtt_client_cloud.on_disconnect = self.cloud_on_disconnect
            self.mqtt_client_cloud.connect_async(self.mqtt_broker_address_cloud, 1883, 60)
            self.mqtt_client_cloud.loop_start()
        except Exception as e:
            print("Error initializing MQTT cloud client: ", e)
            self.mqtt_cloud = False
            
        try:
            self.mqtt_client_local = mqtt.Client()
            self.mqtt_client_local.on_connect = self.local_on_connect
            self.mqtt_client_local.on_message = self.on_message
            self.mqtt_client_local.on_disconnect = self.local_on_disconnect
            self.mqtt_client_local.connect_async(self.mqtt_broker_address_local, 1883, 60)
            self.mqtt_client_local.loop_start()
        except Exception as e:
            print("Error initializing MQTT local client: ", e)


    def initAutomationHat(self, dataship: Dataship):
        # Initialize Automation Hat
        print("Initializing Automation Hat...")
        try:
            # Set up Automation Hat inputs and outputs
            if automationhat.is_automation_hat(): automationhat.light.power.write(1)
            #automationhat.digital.write(1, 0)  # Set output 1 to low
            #automationhat.digital.write(2, 0)  # Set output 2 to low
            #automationhat.digital.write(3, 0)  # Set output 3 to low
            self.a0 = automationhat.analog[0].read()      # Read from analog input 1
        # Set Automation Hat inputs HIGH.
            #automationhat.input.one.resistor(automationhat.PULL_UP)
            #automationhat.input.two.resistor(automationhat.PULL_UP)
            #automationhat.input.three.resistor(automationhat.PULL_UP)
        # Startup with all relays turned off.
            if automationhat.is_automation_hat(): 
                automationhat.relay.one.off()
                automationhat.relay.two.off()
                automationhat.relay.three.off()

            print("Automation Hat initialized successfully.")
        except Exception as e:
            print("Error initializing Automation Hat: ", e)

    #############################################

    def cloud_on_connect(self, client, userdata, flags, rc):
        print("mqtt cloud client connected with result code " + str(rc))
        # Subscribe to the topic
        self.mqtt_client_cloud.subscribe("1TM")
        print("mqtt cloud client subscribed to topic: 1TM")
        self.mqtt_cloud = True
        if automationhat.is_automation_hat(): automationhat.light.comms.write(1)
        
    def local_on_connect(self, client, userdata, flags, rc):
        print("mqtt local client connected with result code " + str(rc))
        # Subscribe to the topic
        self.mqtt_client_local.subscribe("1TM")
        print("mqtt local client subscribed to topic: 1TM")

    def cloud_on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(client, " Unexpected disconnection from cloud mosquito broker.")
        if automationhat.is_automation_hat(): automationhat.light.comms.write(0)
        self.mqtt_cloud = False

    def local_on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(client, " Unexpected disconnection from local mosquito broker.")

    def on_message(self, client, userdata, msg):
        # Handle incoming messages
        if self.debug_mode>0: print("mqtt Received message: " + str(msg.payload))
        # Process the message as needed

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            print("Error found, exiting readMessage")
            self.mqtt_client_local.disconnect()
            self.mqtt_client_cloud.disconnect()
            self.mqtt_client_local.loop_stop()
            self.mqtt_client_cloud.loop_stop()
            return dataship

        # print ("self.loop_time: ", self.loop_time, " time.time(): ", time.time())        
        if time.time() - self.loop_time < 1:   # no need to read data faster than once per every 1 seconds.
            return dataship
        self.loop_time = time.time()

        self.debug_mode = dataship.debug_mode   # Set debug mode from dataship for received mqtt messages

        # Read the analog input value and convert to gallons
        # Convert the value to gallons (0.250 - 4.0 Volts corresponds to 0-5 gallons)
        self.a0 = automationhat.analog[0].read()  # Read from analog input 1
        if(self.ApplySmoothing):
            self.smoothingA.append(self.a0)  # Append the current value to the smoothing list
            if(len(self.smoothingA)>self.SmoothingAVGMaxCount): self.smoothingA.pop(0)
            self.a0 = statistics.mean(self.smoothingA)  # Calculate the average of the last N values
        else:
            #else don't apply smoothing.
            pass
        self.analogData.Data[0] = self.a0

        if self.a0 < 0.250 or self.a0 > 4.000:  # Check for broken wire or bad sensor
            if automationhat.is_automation_hat(): automationhat.light.warn.write(1)
        else:
            if automationhat.is_automation_hat(): automationhat.light.warn.write(0)

        if self.a0 > 0.250:
            self.a0 = self.a0 - 0.250
        else:
            self.a0 = 0     # Wire or sensor probably broken, set level to zero
            
        # Convert analog voltage to gallons:
        self.smokeLevel = 5 * self.a0 / 3.75
        self.smokeLevel = round(self.smokeLevel, 1)  # Round to 1 decimal place
        if dataship.debug_mode>0: print("Smoke Oil Level: ", self.smokeLevel, " gallons")
        self.analogData_smoke_remain_str = str(int(self.smokeLevel*10)).zfill(4)    # Format as 4 digits with leading zeros
        if dataship.debug_mode>0: print("analogData_smoke_remain_str: ", self.analogData_smoke_remain_str, " gallons")


# Build text string to send to PaPiRus display pi

        self.update = False
        new_FuelLevel = 0
        new_FuelRemain = 0
        new_hobbs_time = 0
        
        if self.smokeLevel != self.old_smokeLevel:
            if dataship.debug_mode>0: print("Smoke Level changed: ", self.smokeLevel, " gallons")
            self.old_smokeLevel = self.smokeLevel
            self.update = True

        if self.engineData.hobbs_time != None:
            new_hobbs_time = self.engineData.hobbs_time *10
            if new_hobbs_time != self.old_hobbs_time:
                self.old_hobbs_time = new_hobbs_time
                self.update = True
                
        self.old_engine_status_str = self.engine_status_str  # Set old engine status to current status
        new_OilPress = 0
        if self.engineData.OilPress != None:
            if self.engineData.OilPress > 15: self.engine_status_str = "r"  # running
            else: self.engine_status_str = "s"  # stopped
            new_OilPress = self.engineData.OilPress
            if new_OilPress != self.old_OilPress:
                self.old_OilPress = new_OilPress
                self.update = True
                
        # Test code to fake engine status
        if self.smokeLevel > 5:
            self.engine_status_str = "s"  # stopped
        else:
            self.engine_status_str = "r"  # running
            
        new_FuelRemain = 0
        if self.fuelData.FuelRemain != None:
            new_FuelRemain = self.fuelData.FuelRemain * 10
            if new_FuelRemain != self.old_FuelRemain:
                self.old_FuelRemain = new_FuelRemain
                self.update = True
        new_FuelLevel = 0
        if self.fuelData.FuelLevels[0] != None:
            new_FuelLevel = self.fuelData.FuelLevels[0] * 10
            if new_FuelLevel != self.old_FuelLevel:
                self.old_FuelLevel = new_FuelLevel
                self.update = True

        if self.update:
            self.engineData_hobbs_time_str = str(int(new_hobbs_time)).zfill(5)
            self.engineData_OilPress_str = str(int(new_OilPress)).zfill(2)
            self.fuelData_FuelRemain_str = str(int(new_FuelRemain)).zfill(3)
            self.fuelData_FuelLevel_str = str(int(new_FuelLevel)).zfill(3)
            
        # Build the string to send to the display
        if time.time() - self.start_time > 10 and self.update:   # Send data every 10 seconds at the most.
            if dataship.debug_mode > 0:
                print("engineData_hobbs_time_str = ", self.engineData_hobbs_time_str)
                print("fuelData_FuelRemain_str = ", self.fuelData_FuelRemain_str)
                print("fuelData_FuelLevel_str = ", self.fuelData_FuelLevel_str)
                print("engineData_OilPress_str = ", self.engineData_OilPress_str)
            self.start_time = time.time()
            # Send the data to the PaPiRus display
            self.papirus_str = "!41+" + self.analogData_smoke_remain_str + "G" + self.engineData_hobbs_time_str + self.fuelData_FuelRemain_str + self.engine_status_str + '\r\n'
            papirus_bytes = self.papirus_str.encode()
            if dataship.debug_mode>0: print("papirus_bytes: ", papirus_bytes)
            try:
                self.ser.write(papirus_bytes)         # Send data to PaPiRus
            except Exception as e:
                print(e)
                print("Unexpected error in write to PaPiRus: ", e)

            if self.mqtt_cloud:
                try:
                    self.mqtt_client_cloud.publish("1TM", self.papirus_str)
                    if dataship.debug_mode>0: print("papirus_str to mqtt cloud: ", self.papirus_str)
                except Exception as e:
                    print(e)
                    print("Unexpected error in publish to MQTT: ", e)
            try:
                self.mqtt_client_local.publish("1TM", self.papirus_str)
                if dataship.debug_mode>0: print("papirus_str to mqtt local: ", self.papirus_str)
            except Exception as e:
                print(e)
                print("Unexpected error in publish to MQTT: ", e)
            
                
        if self.engine_status_str == "s" and self.old_engine_status_str == "r" and self.isAdafruitIOReachable():
            print("Initializing Adafruit IO...")
            self.ADAFRUIT_IO_USERNAME = _input_file_utils.readConfig("AIO", "ADAFRUIT_IO_USERNAME")
            self.ADAFRUIT_IO_KEY = _input_file_utils.readConfig("AIO", "ADAFRUIT_IO_KEY")
            self.ADAFRUIT_FEED_ONE = _input_file_utils.readConfig("AIO", "ADAFRUIT_FEED_ONE")
            self.ADAFRUIT_FEED_TWO = _input_file_utils.readConfig("AIO", "ADAFRUIT_FEED_TWO")
            self.ADAFRUIT_FEED_THREE = _input_file_utils.readConfig("AIO", "ADAFRUIT_FEED_THREE")

            print ("Feed_One: ", self.ADAFRUIT_FEED_ONE)
            print ("Feed_Two: ", self.ADAFRUIT_FEED_TWO)
            print ("Feed_Three: ", self.ADAFRUIT_FEED_THREE)
    
            # Initialize AdaFruit IO client, feeds then send the data
            self.AIO = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)  # Initialize Adafruit IO client
            print("Adafruit IO client initialized.")
            try:
                self.ADAFRUIT_FEED_ONE = self.AIO.feeds(self.ADAFRUIT_FEED_ONE)
            except RequestError: # Doesn't exist, create a new feed
                self.ADAFRUIT_FEED_ONE = Feed(name=self.ADAFRUIT_FEED_ONE)
                self.AIO.create_feed(self.ADAFRUIT_FEED_ONE)

            try:
                self.ADAFRUIT_FEED_TWO = self.AIO.feeds(self.ADAFRUIT_FEED_TWO)
            except RequestError: # Doesn't exist, create a new feed
                self.ADAFRUIT_FEED_TWO = Feed(name=self.ADAFRUIT_FEED_TWO)
                self.AIO.create_feed(self.ADAFRUIT_FEED_TWO)
                
            try:
                self.ADAFRUIT_FEED_THREE = self.AIO.feeds(self.ADAFRUIT_FEED_THREE)
            except RequestError: # Doesn't exist, create a new feed
                self.ADAFRUIT_FEED_THREE = Feed(name=self.ADAFRUIT_FEED_THREE)
                self.AIO.create_feed(self.ADAFRUIT_FEED_THREE)

            # Fuel Remaining:
            fuelRemain = self.fuelData.FuelRemain
            print("fuelRemain: ", fuelRemain)
            self.AIO.send_data(self.ADAFRUIT_FEED_ONE.key, str(fuelRemain))

            # Hobbs Time:
            hobbsTime = self.engineData.hobbs_time 
            print("hobbsTime: ", hobbsTime)
            self.AIO.send_data(self.ADAFRUIT_FEED_TWO.key, str(hobbsTime))
                    
            # Smoke Level:
            self.AIO.send_data(self.ADAFRUIT_FEED_THREE.key, self.analogData_smoke_remain_str)

        self.loop_count = self.loop_count + 1
        if dataship.debug_mode >0: print("end of readMessage, loop_count: ", self.loop_count)
        return dataship

    def isAdafruitIOReachable(self):
        url = "https://io.adafruit.com"
        return self.isUrlReachable(url)
    
    def isUrlReachable(self, url):
        try:
            response = urlopen(url)
            return response.status == 200
        except Exception as e:
            print(f"Error checking URL {url}: {e}")
            return False
     
    # close this data input 
    def closeInput(self,dataship: Dataship):
        self.ser.close()
        self.mqtt_client_cloud.disconnect()
        self.mqtt_client_local.disconnect()
        self.mqtt_client_cloud.loop_stop()
        self.mqtt_client_local.loop_stop()        

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python

