#!/usr/bin/env python

# ads1115 input source


from ._input import Input
from lib import hud_utils
from . import _utils
from . import _input_file_utils
import struct
import time
import Adafruit_ADS1x15
import statistics
import traceback
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_analog import AnalogData
from lib.common.dataship.dataship_nav import NavData

class adc_ads1115(Input):
    def __init__(self):
        self.name = "ads1115"
        self.version = 1.0
        self.inputtype = "adc"
        self.values = []
        self.valueNames = []
        self.ApplySmoothing = _input_file_utils.readConfig("Analog", "ApplySmoothing", 1)
        self.SmoothingAVGMaxCount = _input_file_utils.readConfig("Analog", "SmoothingAVGMaxCount", 10)
        self.smoothingA = []
        self.smoothingB = []
        self.analogData = AnalogData()

    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        if(self.PlayFile!=None):
            self.isPlaybackMode = True
        else:
            self.isPlaybackMode = False

        # setup comm i2c to chipset.
        self.adc = Adafruit_ADS1x15.ADS1115()
        # Default address is 0x48, but you can change it if needed.
        #   self.adc = Adafruit_ADS1x15.ADS1115(address=0x49)
        # Single ended mode is used to read the voltage difference between two channels.
        #   self.adc = Adafruit_ADS1x15.ADS1115(mode=Adafruit_ADS1x15.ADS1115_MODE_SINGLE)
        #   15 bit resolution
        # Double ended mode is used to read the voltage difference between two channels.
        #   self.adc = Adafruit_ADS1x15.ADS1115(mode=Adafruit_ADS1x15.ADS1115_MODE_DIFF)
        #   16 bit resolution
        # Note all the voltages must be between GND and VDD.
        # Set the gain for the ADC.
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
        dataship.analog.Name = "ads1115"
        self.Amplify = 6.144/32767
        self.values = [0,0,0,0,0,0,0,0]
        self.valueNames[0] = _input_file_utils.readConfig("Analog", "name1", "GSDev")
        self.valueNames[1] = _input_file_utils.readConfig("Analog", "name2", "ILSDev")
        self.min1 = _input_file_utils.readConfigFloat("Analog", "minimum1", 0.25)
        self.max1 = _input_file_utils.readConfigFloat("Analog", "maximum1", 4.00)
        self.min2 = _input_file_utils.readConfigFloat("Analog", "minimum2", 0.00)
        self.max2 = _input_file_utils.readConfigFloat("Analog", "maximum2", 5.00)
        self.scale1 = _input_file_utils.readConfigFloat("Analog", "scale1", 1.0)
        self.scale2 = _input_file_utils.readConfigFloat("Analog", "scale2", 1.0)

        # create analog data object.
        self.analogData = AnalogData()
        self.analogData.name = self.name
        self.index = len(dataship.analogData)
        self.analogData.id = self.name + "_" + str(self.index)
        dataship.analogData.append(self.analogData)

        # create nav data object.
        self.navData = NavData()
        self.navData.name = self.name
        self.index = len(dataship.navData)
        self.navData.id = self.name + "_" + str(self.index)
        dataship.navData.append(self.navData)

    def closeInput(self,dataship: Dataship):
        print("ads1115 close")


    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if self.shouldExit == True: dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit: return dataship
        if self.skipReadInput == True: return dataship

        try:
            time.sleep(0.025) # delay because of i2c read.
            self.values[1] = self.adc.read_adc_difference(0, gain=self.GAIN) * self.Amplify # read chanel 0-1
            time.sleep(0.025)
            self.values[0] = self.adc.read_adc_difference(3, gain=self.GAIN) * self.Amplify  # read chanel 2-3

            # apply smoothing avg of adc values?
            if(self.ApplySmoothing):
                self.smoothingA.append(self.values[0])
                if(len(self.smoothingA)>self.SmoothingAVGMaxCount): self.smoothingA.pop(0)
                self.analogData.Data[0] = statistics.mean(self.smoothingA)

                self.smoothingB.append(self.values[1])
                if(len(self.smoothingB)>self.SmoothingAVGMaxCount): self.smoothingB.pop(0)
                self.analogData.Data[1] = statistics.mean(self.smoothingB)
            else:
                #else don't apply smoothing.
                self.analogData.Data = self.values


            # if analog input is for nav needles.. .then.
            # limit the output voltages to be within +/- 0.25V
            # format value to +/- 4095 for needle left/right up/down.
            # otherwise use range and scale values from config file.
            #TODO Change "ILSDev" to "LOCDev" due to "ILS is both lateral and vertical guidance"
            
            if self.valueNames[0] == "GSDev":
                self.navData.GSDev = round (16380 * (max(min(self.analogData.Data[0], 0.25), -0.25)))
            else:
                if self.analogData[0] > self.min1:
                    self.analogData.Data[0] = self.analogData[0] - self.min1
                else:
                    self.analogData.Data[0] = 0.0
                self.analogData.Data[0] = self.scale1 * self.analogData.Data[0]
                                    
            if self.valueNames[1] == "ILSDev":
                self.navData.ILSDev = round (16380 * (max(min(self.analogData.Data[1], 0.25), -0.25)))
            else:
                if self.analogData[2] > self.min2:
                    self.analogData.Data[1] = self.analogData[1] - self.min2
                else:
                    self.analogData.Data[1] = 0.0
                self.analogData.Data[1] = self.scale2 * self.analogData.Data[1]

        except Exception as e:
            dataship.errorFoundNeedToExit = True
            print(e)
            print(traceback.format_exc())
        return dataship




# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
