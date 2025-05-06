#!/usr/bin/env python
# 
import sys
from lib.util import rpi_hardware
from lib.util import mac_hardware
from ._input import Input
from lib import hud_utils
from . import _utils
from lib.common import shared # global shared objects stored here.
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_analog import AnalogData
from time import sleep
from time import time

print("automation_hat.py Version 1.0")

isRunningOnPi = rpi_hardware.is_raspberrypi()
if isRunningOnPi == True: 
    print("Running on RaspberryPi")
    shared.Dataship.internal.Hardware = "RaspberryPi"
    shared.Dataship.internal.OS = rpi_hardware.get_full_os_name()
    shared.Dataship.internal.OSVer = rpi_hardware.get_kernel_release()

isRunningOnMac = mac_hardware.is_macosx()
if isRunningOnMac == True: 
    import platform
    import os
    print("Running on Mac OSX")
    sys.exit(0)

# Must be running on Raspberry Pi:
# To install AutomationHat code:
#   curl https://get.pimoroni.com/automationhat | bash

# Requires modification to /usr/lib/python3/dist-packages/automationhat/__init__.py
# Fork of modification is from: https://github.com/kiddigital/automation-hat

import automationhat


###  sleep(20)    # Wait for boot-up to complete

# Definition of Automation Hat IO
# Analog Input 1  Smoke Tank level (0 - 5 volts)
# Analog Input 2  Unused
# Analog Input 3  Used to enable status LEDs (>1 volt)
# Analog Input 4 (0-3.3 v only)
#
# Digital Input 1  "Flaps_Up" switch on stick
# Digital Input 2  "Flaps_Down" switch on stick
# Digital Input 3  Grounded will end program
#
# Digital Output 1
# Digital Output 2
# Digital Output 3
#
# Relay Output 3 ON runs flap motor UP
# Relay Output 2 ON runs flap motor DOWN
# Relay Output 1  Busted



class automationhat(Input):
    def __init__(self):
        self.name = "automation_hat"
        self.version = 1.0
        self.inputtype = "adc"
        self.values = []
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.smoothingB = []
        self.analogData = AnalogData()

    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        if(self.PlayFile!=None):
            self.isPlaybackMode = True
        else:
            self.isPlaybackMode = False

        #self.adc = Adafruit_ADS1x15.ADS1115()

        dataship.analog.Name = "automation_hat"
        self.Amplify = 6.144/32767
        self.values = [0,0,0,0,0,0,0,0]

        # create analog data object.
        self.analogData = AnalogData()
        self.analogData.name = self.name
        self.index = len(dataship.analogData)
        self.analogData.id = self.name + "_" + str(self.index)
        dataship.analogData.append(self.analogData)

    def closeInput(self,dataship: Dataship):
        print("automation_hat close")

# Set Automation Hat inputs HIGH.
        automationhat.input.one.resistor(automationhat.PULL_UP)
        automationhat.input.two.resistor(automationhat.PULL_UP)
        automationhat.input.three.resistor(automationhat.PULL_UP)

        automationhat.light.power.write(1)
        sleep(2)

# Check if Automation Hat status LEDs should be used.
# Use Analog Input 3, if over 1 volt, turn on LEDs.
        print ("Analog.Three: ", automationhat.analog.three.read())
        if automationhat.analog.three.read() != 10:
            automationhat.light.power.write(1)
            automationhat.relay.one.auto_light(True)
            automationhat.relay.two.auto_light(True)
            automationhat.relay.three.auto_light(True)
            automationhat.input.one.auto_light(True)
            automationhat.input.two.auto_light(True)
            automationhat.input.three.auto_light(True)
            automationhat.output.one.auto_light(True)
            automationhat.output.two.auto_light(True)
            automationhat.output.three.auto_light(True)
            automationhat.analog.one.auto_light(True)
            automationhat.analog.two.auto_light(True)
            automationhat.analog.three.auto_light(True)
        else:
    #       Disable the status LEDs on the Hat.
            automationhat.light.power.write(0)
            automationhat.light.comms.write(0)
            automationhat.light.warn.write(0)
            automationhat.relay.one.auto_light(False)
            automationhat.relay.two.auto_light(False)
            automationhat.relay.three.auto_light(False)
            automationhat.input.one.auto_light(False)
            automationhat.input.two.auto_light(False)
            automationhat.input.three.auto_light(False)
            automationhat.output.one.auto_light(False)
            automationhat.output.two.auto_light(False)
            automationhat.output.three.auto_light(False)
            automationhat.analog.one.auto_light(False)
            automationhat.analog.two.auto_light(False)
            automationhat.analog.three.auto_light(False)

# Startup with all relays turned off.
        automationhat.relay.one.off()
        automationhat.relay.two.off()
        automationhat.relay.three.off()

        mqtt_data =[1.1,0,0.1,False,1.1]  # Indicated Airspeed in Knots, Flap setting in degrees,
                                  # Smoke level in Gallons, data fresh, deadman timer
        printed = False
        old_smoke = 1
        loop_count  =  0
        max_print   = 20
        deadman_time = 0

        def on_connect(client, userdata, flags, rc):
            print("Connected to local mosquito broker with result code " + str(rc))

        def on_message(client, userdata, msg):
            print("Message from mqtt: " + str(msg.payload.decode()))
            try:
                tag,value = msg.payload.decode() .split(",")
                print("tag = ", tag, "  value = ", value)
            except ValueError:
                print("Error: ", msg.payload.decode())
            print()
            mqtt_data[3] = True  # COMs are good from Serial-Automationhat.py
            if tag == "IAS":
                print("IAS: ", value)
                ias = float(value) / 10
                mqtt_data[0] = ias
                mqtt_data[3] = True             # Fresh data
                mqtt_data[4] = time()           # Reset deadman timer
                automationhat.light.comms.write(1)  # Turn on COMMS status light
                print("mqtt_data[0]", mqtt_data[0])
            elif tag == "Flaps":
                print("Flaps at: ", value)
                flap_position = int(value)
                mqtt_data[1] = flap_position
                mqtt_data[3] = True                 # Fresh data
                mqtt_data[4] = time()               # Reset deadman timer
                automationhat.light.comms.write(1)  # Turn on COMMS status light
                print("mqtt_data[1]", mqtt_data[1])
                client.disconnect()


#def timer_callback(wait_time):
#    print("Times UP")

# timer = Timer(1, timer_callback, timer_callback_args=(1))


    print("Starting Loop")

    sleep(3)                    # force wait so COMMS will be bad unless they are good
    mqtt_data =[1.1,0,0.1,False,1.1]  # Indicated Airspeed in Knots, Flap setting in degrees,
    max_print = 20

    while True:
#    if mqtt_data[3]: print('Comms seem to be good')
        if time() - mqtt_data[4] > 2:
            mqtt_data[3] = False                            # Over 2 seconds since fresh data
            automationhat.light.comms.write(0)              # Turn off COMMS status light
        smoke_volts = automationhat.analog.one.read()
        if loop_count < max_print: print("Smoke Volts:", smoke_volts)
        if smoke_volts <= 0.25:
            smoke_gal_int = 0
        else:
            smoke_gal_int = int((smoke_volts - 0.25) * 10.19)
        if abs(old_smoke - smoke_gal_int) > 1:
            smoke_gal = smoke_gal_int / 10
            if loop_count < max_print:  print('Smoke Level:     {0:3.1f}'.format    (smoke_gal), 'Gallons')
        pub = 'Smoke,  {0:3.1f}'.format(smoke_gal)
#        client_cloud.publish("1TM", pub)
        old_smoke = smoke_gal_int


        if mqtt_data[0] > 95:                                # Max IAS in knots for flaps down over 20 degrees
            if mqtt_data[1] > 20 and mqtt_data[3]:           # Check current Flap Position and for fresh data
                count = 0
                while count < 20:
                    if automationhat.relay.two.is_on():  automationhat.relay.two.off()      # Shut off flap down motor
                    if automationhat.relay.one.is_off(): automationhat.relay.one.on()       # Turn on flap up motor
                    if mqtt_data[1] < 21:                                                   # flaps now at 20 deg or less
                        automationhat.relay.one.off()
                        break
                    count = count + 1
                    if automationhat.input.two.is_off():                                    # Flap down switch pressed
                        if automationhat.relay.one.is_on():  automationhat.relay.one.off()  # Stop raising flap
                        sleep(.4)
                        break
                    sleep(.1)
                    if mqtt_data[1] < 24:
                        automationhat.relay.one.off()                     # pause as we approach 20 deg
                        sleep(.5)
                        if mqtt_data[1] < 21: break
        if mqtt_data[0] > 87:                                           # Max IAS in knots for flaps down up to 20 degrees
            if mqtt_data[1] > 0 and mqtt_data[3]:                       # Check current Flap Position and fresh data
                count = 0
                while count < 10:
                    if automationhat.relay.two.is_on():  automationhat.relay.two.off()
                    if automationhat.relay.one.is_off(): automationhat.relay.one.on()
                    if mqtt_data[1] == 0:                                  # Flaps are up
                        automationhat.relay.one.off()
                        break
                    count = count + 1
                    if automationhat.input.two.is_off():                                 # Override with down switch
                        if automationhat.relay.one.is_on():  automationhat.relay.one.off()
                        sleep(.4)
                        break
                    sleep(.1)
                    if mqtt_data[1] <= 4:
                        automationhat.relay.one.off()                 # pause as flaps approach 0 deg
                        sleep(.5)
                        if mqtt_data[1] == 0: break
        mqtt_data[0] = 0     # Reset IAS so we only raise flaps once in case this isn't updated fast enough
#    if automationhat.input.three.is_off(): break                                   # This would kill the program
        if automationhat.input.one.is_off():                                            # Flap up switch is pressed
#        if not printed: 
            print("Flaps.UP is pressed")
            printed = True
            count = 0
            while count < 60:                                                           # Raise flaps all the way up
                if automationhat.relay.two.is_on():  automationhat.relay.two.off()      # Shut off Flap down power if on
                if automationhat.relay.one.is_off(): automationhat.relay.one.on()       # Turn on Flap up power if off
                count = count + 1
                if mqtt_data[1] == 0 and mqtt_data[3]:                                 # Flaps are up
                    automationhat.relay.one.off()
                    break
                if mqtt_data[1] <= 4 and mqtt_data[3]:
                    sleep(.4)                                                          # let run a little longer
                    automationhat.relay.one.off()                                      # Then shut off and pause
                    sleep(1)
                    count = count + 15
                    if mqtt_data[1] == 0 and mqtt_data[3]: break                # If flaps are up we are done
                if automationhat.input.two.is_off():                                    # Flap Down Switch Pressed
                    if automationhat.relay.three.is_on():  automationhat.relay.three.off()  # Turn off Flap up power if on
                    if automationhat.relay.one.is_on():    automationhat.relay.one.off()
                    sleep(.4)
                    break
                sleep(.1)
            printed = False
        elif automationhat.input.two.is_off():
            if not printed: print("Flaps.DOWN is pressed")
            printed = True
            flap_down = 20                                 # Assume lowering flaps to 20 degrees
            if mqtt_data[1] > 18: flap_down = 37           # Lower flaps all the way, already 19 or moredeg
            count = 0
            while count < 30:
                if automationhat.relay.one.is_on():  automationhat.relay.one.off()
                if automationhat.relay.two.is_off(): automationhat.relay.two.on()
                count = count + 1
                if mqtt_data[1] >= flap_down and mqtt_data[3]:
                    automationhat.relay.two.off()                                   # stop lowering flaps
                    break                                                           # Flaps down as far as they need to go
                if mqtt_data[1] >= flap_down - 4 and mqtt_data[3]:
                    sleep(.2)
                    automationhat.relay.two.off()
                    break
#                   if mqtt_data[1] >= flap_down and mqtt_data[3]: break
                if automationhat.input.one.is_off():
                    if automationhat.relay.two.is_on():  automationhat.relay.two.off()
                    sleep(.4)
                    break
                sleep(.1)
            printed = False
        else:
            if automationhat.relay.one.is_on():   automationhat.relay.one.off()
            if automationhat.relay.two.is_on():   automationhat.relay.two.off()
            if automationhat.relay.three.is_on(): automationhat.relay.three.off()
            sleep(.25)
        loop_count = loop_count + 1

