#!/usr/bin/env python

#################################################
# IO to and from a Pimoroni Automation Hat which comes in three versions:
# 
# The "Automation Hat:
# It has three 24V 2 ampSPDT relays.
# GPIO #14 & #15 TX/RX UART
# Inputs and Outputs that are 0-24 volt tolerant:
#     Three sinking outputs
#     Three buffered inputs
#     Three 0-24V analog inputs
# One 0-3.3V analog input
# and lots of cool status leds.  
#
# The "Automation Mini":
# It has one 24V 2 amp SPDT relay
# Inputs and Outputs that are 0-24 volt tolerant:
#     Three sinking outputs
#     Three buffered inputs
#     Three 0-24V analog inputs
# One 0.96 inch 160x80 color LCD display
#
# The "Automation pHat":  Discontinued as of 2023
# It has one 24V 2 amp SPDT relay
# Inputs and Outputs that are 0-24 volt tolerant:
#     Three sinking outputs
#     Three buffered inputs
#     Three 0-24V analog inputs
# 
# 
# The hat is mounted on the Pi 5 via the GPIO pins.
# it can be used to increase the number of analog and digital inputs 
# when the EFIS is maxed out or to provide relay outputs to control.
# If an Automation Mini is used, this code will also display the smoke oil level
# on the LCD display as a default.  Use GIMP to edit the image files in docs/imgs
# to create your own custom display.  Image /docs/imgs/mini_analog_smoke.bmp
# is used to display smoke oil level and analog voltage. Image /docs/imgs/mini_analog.bmp
# is used to display only analog voltage.
# 
# Zap 2025
# 
# To check I2C devices:        sudo i2cdetect -y 0
#
# To install AutomationHat code:
# git clone https://github.com/pimoroni/automation-hat
# cd automation-hat
# ./install.sh
#
# pip freeze > requirments.txt
#    Must install as superuser to be able to run the AutomationHat code as superuser.
# sudo python3 -m pip install -r requirments.txt  --break-system-packages
# sudo pip3 install automationhat --break-system-packages
# sudo pip3 install st7735 --break-system-packages
# sudo pip3 install pillow --break-system-packages
# sudo pip3 install fonts font-roboto --break-system-packages


from time import time
import statistics
from ._input import Input
from lib.modules._module import Module
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_analog import AnalogData
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
        self.isPlaybackMode = False
        self.tv_feed_one = None
        self.tv_feed_two = None
        self.tv_feed_three = None
        self.engineData_hobbs_time_str = "00000"
        self.fuelData_FuelRemain_str = "0000"
        self.fuelData_FuelLevel_str = "000"
        self.engineData_OilPress_str = "000"
        self.analogData_smoke_remain_str = "0000"
        self.a0 = 0                         # Analog input 0.  Read from Automation Hat.  
        self.a1 = 0                         # Analog input 1.
        self.a2 = 0                         # Analog input 2.
        self.a3 = 0                         # Analog input 3.  Only on full Automation Hat and only 0-3.3V toloerant.
        self.di0 = 0                        # Digital input 0. Set to 1 to indicate the Automation Hat is running.
        self.di1 = 0                        # Digital input 1.
        self.di2 = 0                        # Digital input 2.
        self.loop_time = 0.0
        # Add smoothing configuration
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.debug_mode = 0
        self.minihat = False
                
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
            try:
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
                self.text_y = 24
                self.offset = 0

                # Open our background image.
                self.image = Image.open("docs/imgs/mini_analog_smoke.bmp")
                self.draw = ImageDraw.Draw(self.image)
                self.minihat = True
            except Exception as e:
                print("Probably a phat Vs mini-hat. Error: ", e)            
                self.minihat = False

        try:
            # Set up Automation Hat inputs and outputs
            if automationhat.is_automation_hat():   # Only the big hat has LEDs
                automationhat.light.power.write(0)
                automationhat.light.comms.write(0)
                automationhat.light.warn.write(0)
            automationhat.output[0].write(0)  # Set output 1 to low
            automationhat.output[1].write(0)  # Set output 2 to low
            automationhat.output[2].write(0)  # Set output 3 to low
            self.a0 = automationhat.analog[0].read()      # Read from analog input 0
            self.a1 = automationhat.analog[1].read()      # Read from analog input 1
            self.a2 = automationhat.analog[2].read()      # Read from analog input 2
            if automationhat.is_automation_hat(): 
                self.a3 = automationhat.analog[3].read()      # Read from analog input 3
            self.di0 = automationhat.input[0].read()  # Read digital input 0
            self.di1 = automationhat.input[1].read()  # Read digital input 1
            self.di2 = automationhat.input[2].read()  # Read digital input 2
            print("Automation Hat initialized with analog input 0: ", self.a0)
            print("Analog input 0: ", self.a0)
            print("Analog input 1: ", self.a1)
            print("Analog input 2: ", self.a2)
            if automationhat.is_automation_hat(): print("Analog input 3: ", self.a3)
            print("Digital input 0: ", self.di0)
            print("Digital input 1: ", self.di1)
            print("Digital input 2: ", self.di2)
        # Set Automation Hat inputs HIGH. Only works with custom --init--.py
            #automationhat.input.one.resistor(automationhat.PULL_UP)
            #automationhat.input.two.resistor(automationhat.PULL_UP)
            #automationhat.input.three.resistor(automationhat.PULL_UP)
        # Startup with all relays turned off.
            automationhat.relay[0].write(0)
            if automationhat.is_automation_hat():   # Only big hat has three relays.
                automationhat.relay[1].write(0)
                automationhat.relay[2].write(0)

            print("Automation Hat initialized successfully.")
        except Exception as e:
            print("Error initializing Automation Hat: ", e)

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if dataship.errorFoundNeedToExit:
            print("Error found, exiting readMessage")
            return dataship

        if time() - self.loop_time < 0.5:   # no need to read data faster than once per every 3 seconds.
            return dataship
        self.loop_time = time()

        # Read the analog input value and convert to gallons
        # Convert the value to gallons (0.250 - 4.0 Volts corresponds to 0-5 gallons)
        self.a0 = automationhat.analog[0].read()  # Read from analog input 0
        
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
        if dataship.debug_mode>0: print("Smoke Oil Level: ", self.smokeLevel, " Gallons")
        self.analogData_smoke_remain_str = str(int(self.smokeLevel*10)).zfill(4)    # Format as 4 digits with leading zeros
        if dataship.debug_mode>0: print("analogData_smoke_remain_str: ", self.analogData_smoke_remain_str, " Gallons")
        self.analogData.Data[1] = self.smokeLevel  # Store the smoke level in the analog data object

        self.image = Image.open("docs/imgs/mini_analog_smoke.bmp")
        self.draw = ImageDraw.Draw(self.image)
        self.draw.text((self.text_x, self.text_y + self.offset), "{reading:.2f}".format(reading=self.a0), font=self.font, fill=self.colour)
        self.draw.text((self.text_x, self.text_y + self.offset + 40), "{reading:.2f}".format(reading=self.smokeLevel), font=self.font, fill=self.colour)
        self.disp.display(self.image)    
        return dataship
     
    # close this data input 
    def closeInput(self,dataship: Dataship):
        pass

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python

