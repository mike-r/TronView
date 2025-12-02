#!/usr/bin/env python

#################################################

# Install Adafruit IO library:
# sudo pip3 install adafruit-io --break-system-packages
#
#  How to setup AIO web based screens
# https://learn.adafruit.com/welcome-to-adafruit-io

#################################################


import time
from ._input import Input
from . import _input_file_utils
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
from Adafruit_IO import Client, Feed, RequestError
from lib.common import shared
from urllib.request import urlopen

class AIO(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "AIO"  # set name
        self.isPlaybackMode = False
        self.shouldExit = False

        # Add smoothing configuration
        self.ApplySmoothing = 1
        self.SmoothingAVGMaxCount = 10
        self.smoothingA = []
        self.debug_mode = 0
        
        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()

        self.ADAFRUIT_IO_USERNAME = None        # Your Adafruit IO account UserName
        self.ADAFRUIT_IO_KEY = None             #    and KEY

    # Name of AIO Feeds from config.cfg
        self.ADAFRUIT_FEED_ONE = None           
        self.ADAFRUIT_FEED_TWO = None
        self.ADAFRUIT_FEED_THREE = None
        self.ADAFRUIT_FEED_FOUR = None
        self.ADAFRUIT_FEED_FIVE = None
        self.AIO = None

    # Values pulled out of the Dataship that match AIO feeds
        self.tv_feed_one = 0.0                  
        self.tv_feed_two = 0.0
        self.tv_feed_three = 0.0
        self.tv_feed_four = 0.0
        self.tv_feed_five = 0.0

        self.tv_feed_two_old = 0.0
        self.tv_feed_one_old = 0.0
        self.tv_feed_three_old = 0.0
        self.tv_feed_four_old = 0.0
        self.tv_feed_five_old = 0.0

    # Status of the AIO feed.
        self.aio1Up = True
        self.aio2Up = True
        self.aio3Up = True
        self.aio4Up = True
        self.aio5Up = True
           
    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.dataship = dataship
        self.initAIO(dataship)
        
        if len(shared.Dataship.fuelData) > 0:
            self.fuelData = shared.Dataship.fuelData[0]
        if len(shared.Dataship.engineData) > 0:
            self.engineData = shared.Dataship.engineData[0]
        
    def closeInput(self, dataShip:Dataship):
        pass
        
    def initAIO(self, dataship: Dataship):
        print("Initializing Adafruit IO...")
        self.ADAFRUIT_IO_USERNAME = _input_file_utils.readConfig(self.name, "ADAFRUIT_IO_USERNAME")
        self.ADAFRUIT_IO_KEY = _input_file_utils.readConfig(self.name,   "ADAFRUIT_IO_KEY")
        self.ADAFRUIT_FEED_ONE = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_ONE")
        self.ADAFRUIT_FEED_TWO = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_TWO")
        self.ADAFRUIT_FEED_THREE = _input_file_utils.readConfig(self.name,"ADAFRUIT_FEED_THREE")
        self.ADAFRUIT_FEED_FOUR = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_FOUR")
        self.ADAFRUIT_FEED_FIVE = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_FIVE")

        print ("Feed_One: ",   self.ADAFRUIT_FEED_ONE)
        print ("Feed_Two: ",   self.ADAFRUIT_FEED_TWO)
        print ("Feed_Three: ", self.ADAFRUIT_FEED_THREE)
        print ("Feed_Four: ",  self.ADAFRUIT_FEED_FOUR)
        print ("Feed_Five: ",  self.ADAFRUIT_FEED_FIVE)

        if self.isAdafruitIOReachable():
            self.AIO = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)  # Initialize Adafruit IO client
            print("Adafruit IO client initialized.")
            try:
                self.ADAFRUIT_FEED_ONE = self.AIO.feeds(self.ADAFRUIT_FEED_ONE)
            except RequestError: # Doesn't exist, try to create a new feed
                try:
                    self.ADAFRUIT_FEED_ONE = Feed(name=self.ADAFRUIT_FEED_ONE)
                    self.AIO.create_feed(self.ADAFRUIT_FEED_ONE)
                except:
                    print("AIO Feed ONE probably doesn't exist")
                    self.aio1Up = False
                else:
                    self.aio1Up = True

            try:
                self.ADAFRUIT_FEED_TWO = self.AIO.feeds(self.ADAFRUIT_FEED_TWO)
            except RequestError: # Doesn't exist, try to create a new feed
                try:
                    self.ADAFRUIT_FEED_TWO = Feed(name=self.ADAFRUIT_FEED_TWO)
                    self.AIO.create_feed(self.ADAFRUIT_FEED_TWO)
                except:
                    print("AIO Feed TWO probably doesn't exist")
                    self.aio2Up = False
                else:
                    self.aio2Up = True

            try:
                self.ADAFRUIT_FEED_THREE = self.AIO.feeds(self.ADAFRUIT_FEED_THREE)
            except RequestError: # Doesn't exist, try tocreate a new feed
                try:
                    self.ADAFRUIT_FEED_THREE = Feed(name=self.ADAFRUIT_FEED_THREE)
                    self.AIO.create_feed(self.ADAFRUIT_FEED_THREE)
                except:
                    print("AIO Feed THREE probably doesn't exist")
                    self.aio3Up = False
                else:
                    self.aio3Up = True

            try:
                self.ADAFRUIT_FEED_FOUR = self.AIO.feeds(self.ADAFRUIT_FEED_FOUR)
            except RequestError: # Doesn't exist, create a new feed
                try:
                    self.ADAFRUIT_FEED_FOUR = Feed(name=self.ADAFRUIT_FEED_FOUR)
                    self.AIO.create_feed(self.ADAFRUIT_FEED_FOUR)
                except:
                    print("AIO Feed FOUR probably doesn't exist")
                    self.aio4Up = False
                else:
                    self.aio4Up = True
            
            try:
                self.ADAFRUIT_FEED_FIVE = self.AIO.feeds(self.ADAFRUIT_FEED_FIVE)
            except RequestError: # Doesn't exist, create a new feed
                try:
                    self.ADAFRUIT_FEED_FIVE = Feed(name=self.ADAFRUIT_FEED_FIVE)
                    self.AIO.create_feed(self.ADAFRUIT_FEED_FIVE)
                except:
                    print("AIO Feed FIVE probably doesn't exist")
                    self.aio5Up = False
                else:
                    self.aio5Up = True

        # Pull the name of the value to send to AIO from the config.cfg file.
        # Then fetch the value out of the Dataship.
            tv_feed_one_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_ONE")
            self.feed_one_str_exec = "self.tv_feed_one = self." + tv_feed_one_str
            exec(self.feed_one_str_exec)  # Evaluate the string to get the value
            print("tv_feed_one: ", self.tv_feed_one)
            
            tv_feed_two_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_TWO")
            self.feed_two_str_exec = "self.tv_feed_two = self." + tv_feed_two_str
            exec(self.feed_two_str_exec)  # Evaluate the string to get the value
            print("tv_feed_two: ", self.tv_feed_two)

            tv_feed_three_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_THREE")
            self.feed_three_str_exec = "self.tv_feed_three = self." + tv_feed_three_str
            exec(self.feed_three_str_exec)  # Evaluate the string to get the value
            print("tv_feed_three: ", self.tv_feed_three)

            tv_feed_four_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_FOUR")
            self.feed_four_str_exec = "self.tv_feed_four = self." + tv_feed_four_str
            exec(self.feed_four_str_exec)  # Evaluate the string to get the value
            print("tv_feed_four: ", self.tv_feed_four)


            tv_feed_five_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_FIVE")
            self.feed_five_str_exec = "self.tv_feed_five = self." + tv_feed_five_str
            exec(self.feed_five_str_exec)  # Evaluate the string to get the value
            print("tv_feed_five: ", self.tv_feed_five)

    #############################################
    ## Function: readMessage. Or in this case pull values from the Dataship and send
    ## them to an AIO feed.
    def readMessage(self, dataship: Dataship):
        if self.shouldExit == True: dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit: return dataship
        if self.skipReadInput == True: return dataship

        # AIO Feed One from TronView Value One:
        exec(self.feed_one_str_exec)  # Execute the string to get the value of self.tv_feed_one
        if self.tv_feed_one is None:
            print("TV Value One data not available ...yet.")
        else:
            if dataship.debug_mode>1: print("tv_feed_one: ", self.tv_feed_one)
            if self.tv_feed_one_old != self.tv_feed_one:    # Check for a change in the value
                self.tv_feed_one_old  = self.tv_feed_one
                if dataship.debug_mode >=0: print("tv_feed_one: ", self.tv_feed_one)
            # Only send if fuel remaining is a real number, otherwise AIO will error.
                if self.tv_feed_one != None and self.isAdafruitIOReachable() and self.aio1Up:  
                    self.AIO.send_data(self.ADAFRUIT_FEED_ONE.key, str(self.tv_feed_one))

        # AIO Feed Two from TronView Value Two:
        exec(self.feed_two_str_exec)  # Execute the string to get the value of self.tv_feed_two
        if self.tv_feed_two is None:
            print("TV Value Two data not available ...yet.")
        else:
            if dataship.debug_mode>1: print("tv_feed_two: ", self.tv_feed_two)
            if self.tv_feed_two_old != self.tv_feed_two:
                self.tv_feed_two_old  = self.tv_feed_two
                if dataship.debug_mode >=0: print("tv_feed_two: ", self.tv_feed_two)
                if self.tv_feed_two != None and self.isAdafruitIOReachable() and self.aio2Up:
                    self.AIO.send_data(self.ADAFRUIT_FEED_TWO.key, str(self.tv_feed_two))

        # AIO Feed Three from TronView Value Three:
        exec(self.feed_three_str_exec)  # Execute the string to get the value of self.tv_feed_three
        if self.tv_feed_three is None:
            print("TV Value Three data not available ...yet.")
        else:
            if dataship.debug_mode>1: print("tv_feed_three: ", self.tv_feed_three)
            if self.tv_feed_three_old != self.tv_feed_three:
                self.tv_feed_three_old  = self.tv_feed_three
                if dataship.debug_mode >=0: print("tv_feed_three ", self.tv_feed_three)
                if self.tv_feed_three != None and self.isAdafruitIOReachable() and self.aio3Up:
                    self.AIO.send_data(self.ADAFRUIT_FEED_THREE.key, str(self.tv_feed_three))

        # AIO Feed Four from TronView Value Four:
        exec(self.feed_four_str_exec)  # Execute the string to get the value of self.tv_feed_Four
        if self.tv_feed_four is None:
            print("TV Value Four data not available ...yet.")
        else:
            if dataship.debug_mode>1: print("tv_feed_four: ", self.tv_feed_four)
            if self.tv_feed_four_old != self.tv_feed_four:
                self.tv_feed_four_old  = self.tv_feed_four
                if dataship.debug_mode >=0: print("tv_feed_four ", self.tv_feed_four)
                if self.tv_feed_four != None and self.isAdafruitIOReachable() and self.aio4Up:
                    self.AIO.send_data(self.ADAFRUIT_FEED_FOUR.key, str(self.tv_feed_four))

        # AIO Feed Five from TronView Value Five:
        exec(self.feed_five_str_exec)  # Execute the string to get the value of self.tv_feed_Five
        if self.tv_feed_five is None:
            print("TV Value five data not available ...yet.")
        else:
            if dataship.debug_mode>1: print("tv_feed_five: ", self.tv_feed_five)
            if self.tv_feed_five_old != self.tv_feed_five:
                self.tv_feed_five_old  = self.tv_feed_five
                if dataship.debug_mode >=0: print("tv_feed_five ", self.tv_feed_five)
                if self.tv_feed_five != None and self.isAdafruitIOReachable() and self.aio5Up:
                    self.AIO.send_data(self.ADAFRUIT_FEED_FIVE.key, str(self.tv_feed_five))

        return dataship
    
    def isUrlReachable(self, url):
        try:
            response = urlopen(url)
            return response.status == 200
        except Exception as e:
            print(f"Error checking URL {url}: {e}")
            return False
        
    def isAdafruitIOReachable(self):
        url = "https://io.adafruit.com"
        return self.isUrlReachable(url)
    
