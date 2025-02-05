#!/usr/bin/env python

# ads1115 input source


from ._input import Input
from lib import hud_utils
from . import _utils
import struct
import time
import Adafruit_ADS1x15
import statistics
import traceback

class adc_ads1115(Input):
    def __init__(self):
        self.name = "ads1115"
        self.version = 1.0
        self.inputtype = "adc"
        self.values = []
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.smoothingB = []

    def initInput(self,num,aircraft):
        Input.initInput( self,num, aircraft )  # call parent init Input.
        if(self.PlayFile!=None):
            self.isPlaybackMode = True
        else:
            self.isPlaybackMode = False

        # setup comm i2c to chipset.
        self.adc = Adafruit_ADS1x15.ADS1115()
        # Choose a gain of 1 for reading voltages from 0 to 4.09V.
        # Or pick a different gain to change the range of voltages that are read:
        #  - 2/3 = +/-6.144V
        #  -   1 = +/-4.096V
        #  -   2 = +/-2.048V
        #  -   4 = +/-1.024V
        #  -   8 = +/-0.512V
        #  -  16 = +/-0.256V
        # See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
        self.GAIN = 2/3 
        aircraft.analog.Name = "ads1115"
        self.Amplify = 6.144/32767
        self.values = [0,0,0,0,0,0,0,0]


    def closeInput(self,aircraft):
        print("ads1115 close")


    #############################################
    ## Function: readMessage
    def readMessage(self, aircraft):
        if self.shouldExit == True: aircraft.errorFoundNeedToExit = True
        if aircraft.errorFoundNeedToExit: return aircraft
        if self.skipReadInput == True: return aircraft

        try:
            time.sleep(0.025)
            self.values[1] = self.adc.read_adc_difference(0, gain=self.GAIN) * self.Amplify
            time.sleep(0.025)
            self.values[0] = self.adc.read_adc_difference(3, gain=self.GAIN) * self.Amplify

            # apply smoothing avg of adc values?
            if(self.ApplySmoothing):
                self.smoothingA.append(self.values[0])
                if(len(self.smoothingA)>self.SmoothingAVGMaxCount): self.smoothingA.pop(0)
                aircraft.analog.Data[0] = statistics.mean(self.smoothingA)


                self.smoothingB.append(self.values[1])
                if(len(self.smoothingB)>self.SmoothingAVGMaxCount): self.smoothingB.pop(0)
                aircraft.analog.Data[1] = statistics.mean(self.smoothingB)
            else:
                #else don't apply smoothing.
                aircraft.analog.Data = self.values

            # TODO: have config file define what this analog input is for.

            # if analog input is for nav needles.. .then.
            # limit the output voltages to be within +/- 0.25V
            # format value to +/- 4095 for needle left/right up/down.
            aircraft.nav.GSDev = round (16380 * (max(min(aircraft.analog.Data[0], 0.25), -0.25)))
            aircraft.nav.ILSDev = round (16380 * (max(min(aircraft.analog.Data[1], 0.25), -0.25)))

        except Exception as e:
            aircraft.errorFoundNeedToExit = True
            print(e)
            print(traceback.format_exc())
        return aircraft




# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
