#!/usr/bin/env python

#################################################

# Install Adafruit IO library:
# sudo pip3 install adafruit-io --break-system-packages


from time import time
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
from Adafruit_IO import Client, Feed, RequestError  # import Adafruit IO REST client.
from lib.common import shared
from urllib.request import urlopen

class AIO(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "AIO"  # set name        self.update = True
        self.loop_count = 0
        self.engine_status_str = "s"  # Default to stopped
        self.start_time = time.time()
        self.loop_time = time.time()  - 5 # Start loop_time 5 seconds in the past to allow first readMessage to run immediately.
        self.shouldExit = False

        self.ADAFRUIT_IO_USERNAME = None
        self.ADAFRUIT_IO_KEY = None
        self.ADAFRUIT_FEED_ONE = None
        self.ADAFRUIT_FEED_TWO = None
        self.ADAFRUIT_FEED_THREE = None
        self.ADAFRUIT_FEED_FOUR = None
        self.ADAFRUIT_FEED_FIVE = None
        self.AIO = None
           
    def initInput(self,num,dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.dataship = dataship
        self.initAIO(dataship)
        
        self.targetData = TargetData()
        self.gpsData = GPSData()
        self.imuData = IMUData()
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.airData = AirData()
        
        # Initialize shared data so any can be used for AIO.
        if len(shared.Dataship.fuelData) > 0:
            self.fuelData = shared.Dataship.fuelData[0]
        if len(shared.Dataship.engineData) > 0:
            self.engineData = shared.Dataship.engineData[0]
        if len(shared.Dataship.targetData) > 0:
            self.targetData = shared.Dataship.targetData[0]
        if len(shared.Dataship.imuData) > 0:
            self.imuData = shared.Dataship.imuData[0]
        if len(shared.Dataship.analogData) > 0:
            self.analogData = shared.Dataship.analogData[0]
    def closeInput(self, dataShip:Dataship):
        pass
        
    def initAIO(self, dataship: Dataship):
        print("Initializing Adafruit IO...")
        self.ADAFRUIT_IO_USERNAME = _input_file_utils.readConfig(self.name, "ADAFRUIT_IO_USERNAME")
        self.ADAFRUIT_IO_KEY = _input_file_utils.readConfig(self.name, "ADAFRUIT_IO_KEY")
        self.ADAFRUIT_FEED_ONE = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_ONE")
        self.ADAFRUIT_FEED_TWO = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_TWO")
        self.ADAFRUIT_FEED_THREE = _input_file_utils.readConfig(self.name, "ADAFRUIT_FEED_THREE")

        print ("Feed_One: ", self.ADAFRUIT_FEED_ONE)
        print ("Feed_Two: ", self.ADAFRUIT_FEED_TWO)
        print ("Feed_Three: ", self.ADAFRUIT_FEED_THREE)
    
        if self.isAdafruitIOReachable():
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
                
            # AIO Feed One is the fuel remaining in gallons.
            # Read the value from the config file and set it to self.fuelRemain
            
            tv_feed_one_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_ONE")
            feed_one_str_exec = "self.tv_feed_one = self." + tv_feed_one_str
            exec(feed_one_str_exec)  # Evaluate the string to get the value
            print("tv_feed_one: ", self.tv_feed_one)
            
            # AIO Feed Two is the Hobbs time in tenths of hours.
            # Read the value from the config file and set it to self.hobbsTime
            
            tv_feed_two_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_TWO")
            feed_two_str_exec = "self.tv_feed_two = self." + tv_feed_two_str
            exec(feed_two_str_exec)  # Evaluate the string to get the value
            print("tv_feed_two: ", self.tv_feed_two)

            # AIO Feed Three is the smoke level.
            # Read the value from the config file and set it to self.smokeLevel

            tv_feed_three_str = _input_file_utils.readConfig("AIO", "TronView_AIO_FEED_THREE")
            feed_three_str_exec = "self.tv_feed_three = self." + tv_feed_three_str
            exec(feed_three_str_exec)  # Evaluate the string to get the value
            print("tv_feed_three: ", self.tv_feed_three)
        else:
            pass
 

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if self.shouldExit == True: dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit: return dataship
        
        if self.isAdafruitIOReachable():
            self.AIO.send_data(self.ADAFRUIT_FEED_ONE.key, str(self.tv_feed_one))
            self.AIO.send_data(self.ADAFRUIT_FEED_TWO.key, str(self.tv_feed_two))
            self.AIO.send_data(self.ADAFRUIT_FEED_THREE.key, str(self.tv_feed_three))
        else:
            pass
        time.sleep(10)
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
    
