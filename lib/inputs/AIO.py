#!/usr/bin/env python

#################################################

# Install Adafruit IO library:
# sudo pip3 install adafruit-io --break-system-packages


from time import time
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


class AIO(Module):
    # called only when object is first created.
    def __init__(self):
        Module.__init__(self)
        self.name = "AIO"  # set name
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
        self.a0 = 0
        self.start_time = time.time()
        self.loop_time = time.time()  - 5 # Start loop_time 5 seconds in the past to allow first readMessage to run immediately.
        self.papirus_str = ""
        self.mqtt_cloud = False
        self.shouldExit = False

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
        
    def closeInput(self, dataShip:Dataship):
        pass
        
    def initAIO(self, dataship: Dataship):
        print("Initializing Adafruit IO...")
        self.ADAFRUIT_IO_USERNAME = _input_file_utils.readConfigBool(self.name, "ADAFRUIT_IO_USERNAME")
        self.ADAFRUIT_IO_KEY = _input_file_utils.readConfigBool(self.name, "ADAFRUIT_IO_KEY")
        self.ADAFRUIT_FEED_ONE = _input_file_utils.readConfigBool(self.name, "ADAFRUIT_FEED_ONE")
        self.ADAFRUIT_FEED_TWO = _input_file_utils.readConfigBool(self.name, "ADAFRUIT_FEED_TWO")
        self.ADAFRUIT_FEED_THREE = _input_file_utils.readConfigBool(self.name, "ADAFRUIT_FEED_THREE")

        self.AIO = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)  # Initialize Adafruit IO client
        try:
            self.ADAFRUIT_FEED_ONE = self.AIO.feeds(self.ADAFRUIT_FEED_ONE)  # Get the feed for smoke level
        except RequestError: # Doesn't exist, create a new feed
            self.ADAFRUIT_FEED_ONE = Feed(name=self.ADAFRUIT_FEED_ONE)  # Create a new feed object
            self.AIO.create_feed(self.ADAFRUIT_FEED_ONE)  # Create the feed if it doesn't exist

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if self.shouldExit == True: dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit: return dataship
        if self.skipReadInput == True: return dataship
        
        #self.smokeLevel = 3.14159
        #smLv ='%.1f'%(self.smokeLevel)
        #self.AIO.send_data(self.smokeLevel_feed.key, str(smLv))  # Send smoke level to Adafruit IO

        # Fuel Remaining:
        fuelRemain = self.dataship.getFuelRemain()
        if fuelRemain > 0:
            self.AIO.send_data(self.ADAFRUIT_FEED_ONE.key, str(fuelRemain))
            self.shouldExit = True  # Exit after sending data to avoid infinite loop in this example
