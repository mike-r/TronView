#!/usr/bin/env python

# Network WiFi or ethernet udp input source
# Skyview ADS-B data in GDL-90 format

# 03/22/2025 Zap  Based on Topher 2019 stratux_wifi.py and serial_skyview.py


import struct
import time
import socket
import re
import traceback
import time
from geographiclib.geodesic import Geodesic
import datetime
from lib import hud_utils
from ..common.dataship.dataship import Dataship
from ..common.dataship.dataship_imu import IMUData
from lib.common.dataship.dataship_nav import NavData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from ..common.dataship.dataship_gps import GPSData
from ..common.dataship.dataship_air import AirData
from ..common.dataship.dataship_targets import TargetData, Target
from ._input import Input
from ..common import shared
from . import _input_file_utils


class network_skyview_adsb(Input):
    def __init__(self):
        self.name = "dynon_skyview_adsb"
        self.id = "dynon_skyview_adsb"
        self.version = 1.0
        self.inputtype = "network"
        self.PlayFile = None
        self.gps_index = 0
        self.gpsData = None
        self.targetData_index = 0
        self.targetData = None
        self.dataship = None

    def initInput(self, num, dataship: Dataship):
        Input.initInput( self,num, dataship )  # call parent init Input.
        self.output_logBinary = True
        self.dataship = dataship
        
        if(self.PlayFile!=None and self.PlayFile!=False):
            # if in playback mode then load example data file.
            # get file to read from config.  else default to..
            if self.PlayFile==True:
                defaultTo = "dynon_skyview_data3.bin"
                self.PlayFile = "../data/skyview/"+defaultTo
            self.ser,self.input_logFileName = Input.openLogFile(self,self.PlayFile,"rb")
            self.isPlaybackMode = True
        else:
            default_skyviewAdsbUdpPort = "4000"
            self.udpport = _input_file_utils.readConfigInt(self.name, "skyviewAdsbUdpPort", default_skyviewAdsbUdpPort)

            # open udp connection.
            self.ser = socket.socket(socket.AF_INET, #Internet
                                    socket.SOCK_DGRAM) #UDP
                             
            #Bind to any available address on port *portNum*
            print("using UDP port:"+str(self.udpport))
            self.ser.bind(("",self.udpport))
            
            #Prevent the socket from blocking until it receives all the data it wants
            #Note: Instead of blocking, it will throw a socket.error exception if it
            #doesn't get any data
            #self.ser.settimeout(.1)
            self.ser.setblocking(0)

        # create a empty gps object.
        self.gpsData = GPSData()
        self.gpsData.name = self.name
        self.gps_index = len(dataship.gpsData)  # Start at 0
        self.gpsData.id = "skyview_gps_adsb"
        print("new skyview_adsb gps "+str(self.gps_index)+": "+str(self.gpsData))
        dataship.gpsData.append(self.gpsData)

        # create a empty targets object.
        self.targetData = TargetData()
        self.targetData.id = "skyview_targets"
        self.targetData.source = "skyview_adsb"
        self.targetData.name = self.name
        self.targetData_index = len(dataship.targetData)  # Start at 0
        print("new skyview_adsb targets "+str(self.targetData_index)+": "+str(self.targetData))
        dataship.targetData.append(self.targetData)

    def closeInput(self, dataShip:Dataship):
        if self.isPlaybackMode:
            print("closing file: ", self.PlayFile)
            self.ser.close()
        else:
            self.ser.close()

    #############################################
    ## Function: getNextChunck
    def getNextChunck(self,dataship: Dataship):
        if self.isPlaybackMode:
            x = 0
            while x != b'~': # read until ~
                t = self.ser.read(1)
                if len(t) != 0:
                    if t == b'~':       # May be a GDL-90 formated Traffic Message
                        t1 = self.ser.read(1)   # 0, 7, 10, 11, 18, 20, 211 are known GDL-90 message types from Skyview
                        if len(t1) !=0: x = ord(t1)
                        if x == 0 or x == 7 or x == 10 or x == 11 or x == 18 or x == 20 or x == 211:
                            if dataship.debug_mode>1: print("\n~ .GDL-90 formated message type: "+str(x), end = ".")
                            x = 0
                            data = bytearray(b'~')
                            data.extend(t1)
                            while x != 126: # read until ending "~"
                                t = self.ser.read(1)
                                if len(t) != 0:
                                    x = ord(t)
                                    data.extend(t)
                                    if x == 126:
                                        if dataship.debug_mode>1: print("last ~ len=: "+str(len(data)))
                            return data
                        else: # Not a GDL-90 message so pass
                            pass
                else:
                    self.ser.seek(0)
                    if dataship.debug_mode>1: print("Skyview ADS-B file reset")
        else:
            try:
                #Attempt to receive up to 1024 bytes of data
                #if dataship.debug_mode>0: print("Trying to read 1024 bytes")
                recv_data = self.ser.recvfrom(1024)
                data = bytearray(recv_data[0])
                if dataship.debug_mode>1 and data[0]==126: print("Skyview GDL-90 Data received")
                return data
            except socket.timeout:
                #print("Socket timeout")
                pass
            except socket.error:
                #If no data is received, you get here, but it's not an error
                #Ignore and continue
                #print("Socket error")
                pass
        return bytes(0)

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if self.shouldExit == True: dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit: return dataship
        if self.skipReadInput == True: return dataship
        msg = self.getNextChunck(dataship)
        if len(msg) == 0: return dataship
        if dataship.debug_mode>1:
            print("---------------network_skyview_adsb--------------------------------\nNEW Chunk len:"+str(len(msg)))
            print("msg[0:4]:", msg[0:4])
            if len(msg) >= 4 and msg[0] == 126: # GDL-90 message '~'
                if dataship.debug_mode>1: print(" "+str(msg))
        if msg[0] == ord('~'): # GDL-90 message '~'
            if dataship.debug_mode>1: print("Parsing GDL-90 message")
            for line in msg.split(b'~~'):
                theLen = len(line)
                if(theLen>3):
                    if(line[0]!=ord('~')): # if no ~ then add one...
                        newline = b''.join([b'~',line])
                        theLen += 1
                    else:
                        newline = line
                    if(newline[theLen-1]!=ord('~')): # add ~ on the end if not there.
                        newline = b''.join([newline,b'~'])
                    dataship = self.processSingleGDL90Message(newline,dataship)

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,newline)
            return dataship

    def processSingleGDL90Message(self, msg, dataship: Dataship):
        if dataship.debug_mode>1: print("Processing Single Message")
        try:
            if(len(msg)<1):
                pass       
            elif (msg[0] == ord('~')): # GDL 90 message
                if(msg[1]==0): # GDL heart beat. 
                    if dataship.debug_mode>0: print("\nGDL 90 HeartBeat message")
                    if dataship.debug_mode>1: print(msg.hex())
                    if(len(msg)==11):
                        statusByte2 = msg[3]
                        timeStamp = _unsigned16(msg[4:], littleEndian=True)
                        if (statusByte2 & 0b10000000) != 0:
                            timeStamp += (1 << 16)
                        self.gpsData.GPSTime_string = str(datetime.timedelta(seconds=int(timeStamp)))   # get time stamp for gdl hearbeat.
                        timeObj = datetime.datetime.strptime(self.gpsData.GPSTime_string, "%H:%M:%S")
                        self.gpsData.GPSDate_string = datetime.datetime.now().strftime("%m/%d/%y")
                        self.gpsData.GPSTime = timeObj.time

                elif(msg[1]==10): # GDL ownership (Latitude, Longitude, Altitude, Speed, Heading)
                    '''
                    The GDL 90 will always output an Ownship Report message once per second. The message
                    uses the same format as the Traffic Report, with the Message ID set to the value 10

                    The Ownship Report is output by the GDL 90 regardless of whether a valid GPS position fix is
                    available. If the ownship GPS position fix is invalid, the Latitude, Longitude, and NIC fields in
                    the Ownship Report all have the ZERO value.
                    Ownship geometric altitude is provided in a separate message (SW Mod C).

                    Traffic Report data = st aa aa aa ll ll ll nn nn nn dd dm ia hh hv vv tt ee cc cc cc cc cc cc cc cc px
                    Field Definition:
                    s Traffic Alert Status. s = 1 indicates that a Traffic Alert is active for this target.
                    t Address Type: Describes the type of address conveyed in the Participant Address field:
                    aa aa aa Participant Address (24 bits).
                    ll ll ll Latitude: 24-bit signed binary fraction. Resolution = 180 / 223 degrees.
                    nn nn nn Longitude: 24-bit signed binary fraction. Resolution = 180 / 223 degrees.
                    ddd Altitude: 12-bit offset integer. Resolution = 25 feet. Altitude (ft) = ("ddd" * 25) - 1,000
                    m Miscellaneous indicators: (see text)
                    i Navigation Integrity Category (NIC):
                    a Navigation Accuracy Category for Position (NACp):
                    hhh Horizontal velocity. Resolution = 1 kt.
                    vvv Vertical Velocity: Signed integer in units of 64 fpm.
                    tt Track/Heading: 8-bit angular weighted binary. Resolution = 360/256 degrees. 0 = North, 128 = South. See Miscellaneous field for Track/Heading indication.
                    ee Emitter Category
                    cc cc cc cc
                    cc cc cc cc Call Sign: 8 ASCII characters, '0' through '9' and 'A' through 'Z'.
                    p Emergency/Priority Code:
                    x Spare (reserved for future use)
                    
                    '''
                    if dataship.debug_mode>0: print("\nGDL 90 Owership message")
                    if(len(msg)==32):

                        # save gps data coming from traffic source..
                        latLongIncrement = 180.0 / (2**23) # == 0.0000001490116119384765625
                        src_lat = _signed24(msg[6:]) * latLongIncrement
                        src_lon = _signed24(msg[9:]) * latLongIncrement
                        alt = _thunkByte(msg[12], 0xff, 4) + _thunkByte(msg[13], 0xf0, -4) # alt in feet MSL
                        src_alt = (alt * 25) - 1000 # convert to feet MSL (from GDL90 format

                        self.gpsData.set_gps_location(src_lat, src_lon, src_alt)

                        # set source lat/lon/alt. this is what is used to calculate distance to target.
                        self.targetData.src_lat = src_lat
                        self.targetData.src_lon = src_lon
                        self.targetData.src_alt = src_alt

                        horzVelo = _thunkByte(msg[15], 0xff, 4) + _thunkByte(msg[16], 0xf0, -4)
                        if horzVelo == 0xfff:  # no info available
                            self.gpsData.GndSpeed = None
                        else:
                            self.gpsData.GndSpeed = int(horzVelo) # ground speed in knots

                        if(msg[18] != 255):
                            self.gpsData.GndTrack = int(msg[18] * 1.40625)  # track/heading, 0-358.6 degrees
                        else:
                            self.gpsData.GndTrack = None

                        # get NIC
                        self.gpsData.Accuracy = _thunkByte(msg[14], 0xf0, -4)

                        self.gpsData.msg_count += 1

                        if dataship.debug_mode>0:
                            if dataship.debug_mode>0:
                                print(f"GPS Data: {self.gpsData.GPSTime_string} {self.gpsData.Lat} {self.gpsData.Lon} {self.gpsData.GndSpeed} {self.gpsData.GndTrack}")

                elif(msg[1]==11): # GDL 90 formated OwnershipGeometricAltitude
                    if dataship.debug_mode>0: print("\nGDL 90 Ownership GeometricA Altitude message")
                    # get alt
                    self.gpsData.AltPressure = _signed16(msg[2:]) * 5
                    if dataship.debug_mode>0: print(f"GPS Altitude: {self.gpsData.AltPressure}ft")

                elif(msg[1]==20): # Traffic report
                    '''
                    The Traffic Report data consists of 27 bytes of binary data as shown in Figure 2. Each field that
                    makes up the report is a multiple of 4 bits. Each lower case character represents a 4-bit value.
                    Each pair of lower-case characters represents a single byte value.
                    For example, Byte 2 of this message is the first byte of the Traffic Report data, and contains the
                    value "0xst", where "s" represents the Traffic Alert Status and occupies Byte 2 bits 7..4, and 't'
                    represents the Address Type and occupies Byte 2 bits 3..0. Similarly, Byte 28 contains the value
                    "0xpx".
                    '''
                    if dataship.debug_mode>1: print("\nGDL 90 formated Traffic message id:"+str(msg[1])+" len:"+str(len(msg)))
                    if len(msg)==32:  # 32 bytes is the standard GDL-90 traffic message length
                        if dataship.debug_mode>1: print(msg.hex())

                        callsign = re.sub(r'[^A-Za-z0-9]+', '', msg[20:28].rstrip().decode('ascii', errors='ignore') ) # clean the N number.
                        targetStatus = _thunkByte(msg[2], 0x0b11110000, -4) # status
                        targetType = _thunkByte(msg[2], 0b00001111) # type

                        target = Target(callsign)
                        target.aStat = targetStatus
                        target.type = targetType
                        target.address =  (msg[3] << 16) + (msg[4] << 8) + msg[5] # address
                        # get lat/lon
                        latLongIncrement = 180.0 / (2**23)
                        target.lat = _signed24(msg[6:]) * latLongIncrement
                        target.lon = _signed24(msg[9:]) * latLongIncrement
                        # alt of target.
                        alt = _thunkByte(msg[12], 0xff, 4) + _thunkByte(msg[13], 0xf0, -4)
                        target.alt = (alt * 25) - 1000 # alt in feet MSL (from GDL90 format)

                        target.misc = _thunkByte(msg[13], 0x0f) # misc
                        target.NIC = _thunkByte(msg[14], 0xf0, -4) # NIC
                        target.NACp = _thunkByte(msg[14], 0x0f) # NACp

                        #speed
                        horzVelo = _thunkByte(msg[15], 0xff, 4) + _thunkByte(msg[16], 0xf0, -4)
                        if horzVelo == 0xfff:  # no hvelocity info available
                            horzVelo = 0
                        target.speed = round(horzVelo * 1.15078,1) # convert to mph
                        # heading
                        trackIncrement = 360.0 / 256
                        target.track = int(msg[18] * trackIncrement)  # track/heading, 0-358.6 degrees
                        # vert speed. 12-bit signed value of 64 fpm increments
                        vertVelo = _thunkByte(msg[16], 0x0f, 8) + _thunkByte(msg[17])
                        if vertVelo == 0x800:   # not avail
                            vertVelo = 0
                        elif (vertVelo >= 0x1ff and vertVelo <= 0x7ff) or (vertVelo >= 0x801 and vertVelo <= 0xe01):  # not used, invalid
                            vertVelo = 0
                        elif vertVelo > 2047:  # two's complement, negative values
                            vertVelo -= 4096
                        target.vspeed = (vertVelo * 64) ;# vertical velocity

                        target.cat = int(msg[19]) # emitter category (type/size of aircraft)

                        self.targetData.addTarget(target) # add/update target to traffic list.
                        if dataship.debug_mode>1 :
                            print(f"GDL 90 Target: {target.callsign} {target.type} {target.address} {target.lat} {target.lon} {target.alt} {target.speed} {target.track} {target.vspeed}")
                        self.targetData.msg_count += 1
                    else:
                        self.targetData.msg_bad += 1
                        if dataship.debug_mode>1: 
                            print("\n\n   BAD TARGET MESSAGE")
                            print(msg.hex())
                            print(str(msg))
                            print("   len(msg):", len(msg))
                            print("")

                elif(msg[1]==7): # GDL 90 Uplink Data
                    if dataship.debug_mode>1: print("\nGDL 90 formated Uplink Data id:"+str(msg[1])+" len:"+str(len(msg))+"\n")
                elif(msg[1]==18): # Skyview ADS-B Unknown message type of 55 bytes
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg))+"\n")
                    if dataship.debug_mode>1: print(str(msg))
                elif(msg[1]==211): # Skyview ADS-B Unknown message type of 53 bytes
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                    if dataship.debug_mode>1: 
                        print(str(msg))
                        print("\n")
                elif(msg[1]==76):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                elif(msg[1]==101):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                elif(msg[1]==83):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                elif(msg[1]==204):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                elif(msg[1]==30):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg)))
                elif(msg[1]==31):
                    if dataship.debug_mode>1: print("\nSkyview ADS-B Unknown Message Type:"+str(msg[1])+" len:"+str(len(msg))) 
                else: # unknown message id
                    if dataship.debug_mode>0:
                        print("\n   skyview message unkown id:"+str(msg[1])+" "+str(msg[2])+" "+str(msg[3])+" len:"+str(len(msg))+"\n")
                        if dataship.debug_mode>1: print(str(msg))
            return dataship
        
        except ValueError as e :
            print("skyview value error exception")
            dataship.errorFoundNeedToExit = True
            print(e)
            print(traceback.format_exc())
        except struct.error:
            #error with read in length.. ignore for now?
            print("Error with read in length")
            pass
        except Exception as e:
            dataship.errorFoundNeedToExit = True
            print(e)
            print(traceback.format_exc())
        return dataship

def _unsigned24(data, littleEndian=False):
    """return a 24-bit unsigned integer with selectable Endian"""
    #if(len(data) >= 3): raise Exception("_unsigned24 len(data) >= 3")
    if(len(data)<3): return 0
    if littleEndian:
        b0 = data[2]
        b1 = data[1]
        b2 = data[0]
    else:
        b0 = data[0]
        b1 = data[1]
        b2 = data[2]
    
    val = (b0 << 16) + (b1 << 8) + b2
    return val

def _signed24(data, littleEndian=False):
    """return a 24-bit signed integer with selectable Endian"""
    val = _unsigned24(data, littleEndian)
    if val > 8388607:
        val -= 16777216
    return val

def _unsigned16(data, littleEndian=False):
    """return a 16-bit unsigned integer with selectable Endian"""
    #if(len(data) >= 2): raise Exception("_unsigned16 len(data) >= 2")
    if(len(data)<2): return 0
    if littleEndian:
        b0 = data[1]
        b1 = data[0]
    else:
        b0 = data[0]
        b1 = data[1]
    
    val = (b0 << 8) + b1
    return val

def _signed16(data, littleEndian=False):
    """return a 16-bit signed integer with selectable Endian"""
    val = _unsigned16(data, littleEndian)
    if val > 32767:
        val -= 65536
    return val

def _thunkByte(c, mask=0xff, shift=0):
    """extract an integer from a byte applying a mask and a bit shift
    @c character byte
    @mask the AND mask to get the desired bits
    @shift negative to shift right, positive to shift left, zero for no shift
    """
    val = c & mask
    if shift < 0:
        val = val >> abs(shift)
    elif shift > 0:
        val = val << shift
    return val

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
