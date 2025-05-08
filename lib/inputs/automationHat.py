#!/usr/bin/env python

#################################################
# IO to and from a Pimoroni AutomationHat which has three SPDT relays, three 12-bit ADC inputs
# three sinking outputs and lots of cool status leds.  The hat is mounted on the Pi 5 via the
# GPIO pins.  The Automation Hat is used to increase the number of analog inputs when the EFIS
# is maxed out.  Smoke tank level and cowl flap control or other devices on the aircraft.
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

# To install and setup the MQTT Broker:
#  sudo apt install mosquitto mosquitto-clients
#  sudo systemctl enable mosquitto

# To install MQTT library in Python
#   sudo pip3 install paho-mqtt

# Write Serial data to PaPiRus Pi
# Format of data stream is:
# !41+ssssGhhhhhfff
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "hhhhh"    is the Total Time (Hobbs Time) in tenths of hours from Dynon EMS
# "fff"      is the Total Fuel Remaining in tents of gallons

import serial
from time import sleep
import paho.mqtt.client as mqtt #import the client
import automationhat
import sys
import os
import socket
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
        self.old_IAS = 0
        self.old_OilPress = 0
        self.old_FuelRemain = 0
        self.old_FuelLevel = 0
        self.mqtt_broker_address_cloud = "broker.mqtt.cool"
        self.Mqtt_broker_address_local = "localhost"
        self.airData_IAS_str = "00000"
        self.fuelData_FuelRemain_str = "0000"
        self.fuelData_FuelLevel_str = "000"
        self.engineData_OilPress_str = "000"

        # Add smoothing configuration
        self.enable_smoothing = True
        self.smoothing_factor = 0.15
        
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


        # set the target data and gps data to the first item in the list.
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


    def initMqtt(self, dataship: Dataship):
        # Initialize MQTT client
        print("Initializing MQTT client...")
        try:
            self.mqtt_client_cloud = mqtt.Client()
            self.mqtt_client_cloud.on_connect = self.on_connect
            self.mqtt_client_cloud.on_message = self.on_message
            self.mqtt_client_cloud.connect(self.mqtt_broker_address_cloud, 1883, 60)
            self.mqtt_client_cloud.loop_start()
        except Exception as e:
            print("Error initializing MQTT client: ", e)

    def initAutomationHat(self, dataship: Dataship):
        # Initialize Automation Hat
        print("Initializing Automation Hat...")
        try:
            # Set up Automation Hat inputs and outputs
            automationhat.light.power.write(1)
            automationhat.digital.write(1, 0)  # Set output 1 to low
            automationhat.digital.write(2, 0)  # Set output 2 to low
            automationhat.digital.write(3, 0)  # Set output 3 to low
            automationhat.analog.read(1)       # Read from analog input 1
        # Set Automation Hat inputs HIGH.
            automationhat.input.one.resistor(automationhat.PULL_UP)
            automationhat.input.two.resistor(automationhat.PULL_UP)
            automationhat.input.three.resistor(automationhat.PULL_UP)
        # Startup with all relays turned off.
            automationhat.relay.one.off()
            automationhat.relay.two.off()
            automationhat.relay.three.off()

            print("Automation Hat initialized successfully.")
        except Exception as e:
            print("Error initializing Automation Hat: ", e)

    #############################################

    def on_connect(self, client, userdata, flags, rc):
        print("mqtt client connected with result code " + str(rc))
        # Subscribe to the topic
        self.mqtt_client_cloud.subscribe("1TM")
        print("mqtt client subscribed to topic: 1TM")

    def on_message(self, client, userdata, msg):
        # Handle incoming messages
        print("mqtt Received message: " + str(msg.payload))
        # Process the message as needed

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            print("Error found, exiting readMessage")
            return dataship

        if self.loop_count < 3:
            pub = "readMessage method, loop_count: " + str(self.loop_count)
            self.mqtt_client_cloud.publish("1TM", pub)

# Build text string to send to PaPiRus display pi

        new_FuelLevel = 0
        new_FuelRemain = 0
        if self.airData.IAS != None:
            self.update = False
            if self.airData.IAS != self.old_IAS:
                self.old_IAS = self.airData.IAS
                self.update = True
        if self.engineData.OilPress != None:
            if self.engineData.OilPress != self.old_OilPress:
                self.old_OilPress = self.engineData.OilPress
                self.update = True
        if self.fuelData.FuelRemain != None:
            new_FuelRemain = self.fuelData.FuelRemain * 10
            if new_FuelRemain != self.old_FuelRemain:
                self.old_FuelRemain = new_FuelRemain
                self.update = True
        new_FuelLevel = self.fuelData.FuelLevels[0] * 10
        if new_FuelLevel != self.old_FuelLevel:
            self.old_FuelLevel = new_FuelLevel
            self.update = True
        if self.update:
            self.airData_IAS_str = str(self.airData.IAS).zfill(5)
            self.engineData_OilPress_str = str(self.engineData.OilPress).zfill(2)
            self.fuelData_FuelRemain_str = str(new_FuelRemain).zfill(3)
            self.fuelData_FuelLevel_str = str(new_FuelLevel).zfill(3)
            # Build the string to send to the display
        if dataship.debug_mode==0:
            print("airData_IAS_str = ", self.airData_IAS_str)
            print("fuelData_FuelRemain_str = ", self.fuelData_FuelRemain_str)
            print("fuelData_FuelLevel_str = ", self.fuelData_FuelLevel_str)
            print("engineData_OilPress_str = ", self.engineData_OilPress_str)
        papirus_str = '!41' + self.airData_IAS_str + self.engineData_OilPress_str + self.fuelData_FuelRemain_str + '\r\n'
        papirus_bytes = papirus_str.encode()
        print(papirus_bytes)
        try:
            self.ser.write(papirus_bytes)         # Send data to PaPiRus
        except Exception as e:
            print(e)
            print("Unexpected error in write to PaPiRus: ", e)
        try:
            self.mqqt.client_cloud.publish("1TM", papirus_str)
        except Exception as e:
            print(e)
            print("Unexpected error in publish to MQTT: ", e)
        self.loop_count = self.loop_count + 1
        print("end of readMessage, loop_count: ", self.loop_count)
        return dataship
     
    # close this data input 
    def closeInput(self,dataship: Dataship):
        self.ser.close()


# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python

