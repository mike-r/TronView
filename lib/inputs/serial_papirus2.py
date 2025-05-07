#!/usr/bin/env python

#################################################
# Module: Serial to PaPiRus for Display 
# TopZaper 2025.
# 
# Raspberry Pi 
# /dev/ttyUSB0 is a USB to RS-232 converter to receive data from
# the Dynon HDX via Serial out.
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
import sys
import os
import socket
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
#from osgeo import osr
from lib.common import shared


class serialPaPiRus(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "Serial_PaPiRus"  # set name
        self.show_callsign = False
        self.show_details = False
        self.scope_scale = 0
        self.scope_scale_miles = 10
        self.target_show_lat_lon = hud_utils.readConfigBool("TrafficScope", "target_show_lat_lon", False)
        self.draw_icon = hud_utils.readConfigBool("TrafficScope", "draw_icon", True)
        self.icon_scale = hud_utils.readConfigInt("TrafficScope", "icon_scale", 10)
        self.details_offset = hud_utils.readConfigInt("TrafficScope", "details_offset", 5)

        self.targetDetails = {} # keep track of details about each target. like the x,y position on the screen. and if they are selected.

        # Add smoothing configuration
        self.enable_smoothing = hud_utils.readConfigBool("TrafficScope", "enable_smoothing", True)
        self.smoothing_factor = 0.15
        
        # Add tracking of previous positions
        self.target_positions = {} # Store previous positions for smoothing

        # store the target data and gps data to use in the draw method.
        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()
        self.selectedTarget = None
        self.selectedTargetID = None
        self.setScaleInMiles()

        # set the target data and gps data to the first item in the list.
        if len(shared.Dataship.targetData) > 0:
            self.targetData = shared.Dataship.targetData[0]
        if len(shared.Dataship.gpsData) > 0:
            self.gpsData = shared.Dataship.gpsData[0]
        if len(shared.Dataship.imuData) > 0:
            self.imuData = shared.Dataship.imuData[0]


#  Version 1.0 testing
print("serial_PaPiRus.py Version 1.0.Testing")

print("TargetData: src_alt = ", TargetData.src_alt)
print("TargetData: src_gps = ", TargetData.src_gps)

print("Did it work?")