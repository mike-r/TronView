#!/usr/bin/env python

#################################################
# IO to and from a Pimoroni AutomationHat which has three SPDT relays, three 12-bit ADC inputs
# three sinking outputs and lots of cool status leds.  The hat is mounted on the Pi 5 via the
# GPIO pins.  The Automation Hat is used to increase the number of analog and digital inputs 
# when the EFIS is maxed out.  
# It will be first used to read the airplane's smoke tank level.
# 
# Zap 2025
# 
# To check I2C devices:        sudo i2cdetect -y 0

# To install AutomationHat code:
# git clone https://github.com/pimoroni/automation-hat
# cd automation-hat
# ./install.sh
#
# pip freeze > requirments.txt
#    Must install as superuser to be able to run the AutomationHat code as superuser.
# sudo python3 -m pip install -r requirments.txt  --break-system-packages
# sudo pip3 install automationhat --break-system-packages


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
import automationhat
import st7735
from PIL import Image, ImageDraw, ImageFont
from fonts.ttf import RobotoBlackItalic as UserFont


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
        self.old_hobbs_time = -10
        self.old_OilPress = -10.0
        self.old_FuelRemain = -10.0
        self.old_FuelLevel = -10.0
        self.old_smokeLevel = -10.0
        self.new_hobbs_time = 0.0
        self.new_FuelRemain = 0.0
        self.new_OilPress = 0.0
        self.tv_feed_one = None
        self.tv_feed_two = None
        self.tv_feed_three = None
        self.engineData_hobbs_time_str = "00000"
        self.fuelData_FuelRemain_str = "0000"
        self.fuelData_FuelLevel_str = "000"
        self.engineData_OilPress_str = "000"
        self.analogData_smoke_remain_str = "0000"
        self.engine_status_str = "s"  # Default to stopped
        self.old_engine_status_str = "s"  # Default to stopped
        self.a0 = 0                         # Analog input 0.  Read from Automation Hat.  
        self.di0 = 0                        # Digital input 0.  Set to 1 to indicate the Automation Hat is running.
        self.di1 = 0                        # Digital input 1.
        self.start_time = time.time()
        self.loop_time = time.time()  - 5 # Start loop_time 5 seconds in the past to allow first readMessage to run immediately.

        # Add smoothing configuration
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.debug_mode = 0
                
    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.initAutomationHat(dataship)
        if(self.PlayFile!=None and self.PlayFile!=False):
            pass

        # create analog data object.
        self.analogData = AnalogData()
        self.analogData.name = self.name
        self.index = len(dataship.analogData)
        self.analogData.id = self.name + "_" + str(self.index)
        dataship.analogData.append(self.analogData)

        # set the data to the first item in the list
        if len(shared.Dataship.analogData) > 0:
            self.analogData = shared.Dataship.analogData[0]

    def initAutomationHat(self, dataship: Dataship):
        # Initialize Automation Hat
        print("Initializing Automation Hat...")
        
        # Create ST7735 LCD display class if mini-hat.
        # No test to confirm but the mini-hat is based on the phat
        if automationhat.is_automation_phat():
            self.disp = st7735.ST7735(
                port=0,
                cs=st7735.BG_SPI_CS_FRONT,
                dc=9,
                backlight=25,
                rotation=270,
                spi_speed_hz=4000000
            )
            
            # Initialise display.
            self.disp.begin()

            self.colour = (255, 181, 86)
            self.font = ImageFont.truetype(UserFont, 12)

            # Values to keep everything aligned nicely.
            self.text_x = 110
            self.text_y = 34
            self.offset = 0

            self.display_is_off = False
            # Open our background image.
            self.image = Image.open("docs/imgs/blank3.bmp")
            self.draw = ImageDraw.Draw(self.image)

        try:
            # Set up Automation Hat inputs and outputs
            if automationhat.is_automation_hat():
                automationhat.light.power.write(0)
                automationhat.light.comms.write(0)
                automationhat.light.warn.write(0)
            #automationhat.digital.write(1, 0)  # Set output 1 to low
            #automationhat.digital.write(2, 0)  # Set output 2 to low
            #automationhat.digital.write(3, 0)  # Set output 3 to low
            self.a0 = automationhat.analog[0].read()      # Read from analog input 0
            self.di0 = automationhat.input[0].read()  # Read digital input 0
            self.di1 = automationhat.input[1].read()  # Read digital input 1
            print("Automation Hat initialized with analog input 0: ", self.a0)
            print("Digital input 0: ", self.di0)
            print("Digital input 1: ", self.di1)
        # Set Automation Hat inputs HIGH.
            #automationhat.input.one.resistor(automationhat.PULL_UP)
            #automationhat.input.two.resistor(automationhat.PULL_UP)
            #automationhat.input.three.resistor(automationhat.PULL_UP)
        # Startup with all relays turned off.
            automationhat.relay.one.off()
            if automationhat.is_automation_hat(): 
                automationhat.relay.two.off()
                automationhat.relay.three.off()

            print("Automation Hat initialized successfully.")
        except Exception as e:
            print("Error initializing Automation Hat: ", e)

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            print("Error found, exiting readMessage")
            return dataship

        if time.time() - self.loop_time < 3:   # no need to read data faster than once per every 3 seconds.
            return dataship
        self.loop_time = time.time()

        self.debug_mode = dataship.debug_mode   # Set debug mode from dataship for received mqtt messages

        # Read the analog input value and convert to gallons
        # Convert the value to gallons (0.250 - 4.0 Volts corresponds to 0-5 gallons)
        self.a0 = automationhat.analog[0].read()  # Read from analog input 1
        self.draw.text((self.text_x, self.text_y + self.offset), "{reading:.2f}".format(reading=self.a0), font=self.font, fill=self.colour)
        self.disp.display(self.image)
        
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
        self.analogData.Data[1] = self.smokeLevel  # Store the smoke level in the analog data object
                
        self.start_time = time.time()
        self.update = False
    
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
        pass

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python

