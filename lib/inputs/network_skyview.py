#!/usr/bin/env python

# Network WiFi or ethernet udp input source
# Skyview ADHRA, NAV, AP, and Engine Data

# 03/23/2025 Zap  Based on Topher 2019 stratux_wifi.py and serial_skyview.py


import struct
import time
import socket
import traceback
import time
from lib import hud_utils
from ..common.dataship.dataship import Dataship
from ..common.dataship.dataship_imu import IMUData
from lib.common.dataship.dataship_nav import NavData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from ..common.dataship.dataship_gps import GPSData
from ..common.dataship.dataship_air import AirData
from ..common.dataship.dataship_targets import TargetData, Target
from ._input import Input
from . import _input_file_utils


class network_skyview(Input):
    def __init__(self):
        self.name = "dynon_skyview"
        self.id = "dynon_skyview"
        self.version = 1.0
        self.inputtype = "network"
        self.PlayFile = None
        self.imu_index = 0
        self.imuData = None
        self.gps_index = 0
        self.gpsData = None
        self.airdata_index = 0
        self.airData = None
        self.targetData_index = 0
        self.targetData = None

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
            default_skyviewDataUdpPort = "49155"
            self.udpport = _input_file_utils.readConfigInt(self.name, "skyviewDataUdpPort", default_skyviewDataUdpPort)

            # open udp connection.
            self.ser = socket.socket(socket.AF_INET, #Internet
                                    socket.SOCK_DGRAM) #UDP
                             
            #Bind to any available address on port *portNum*
            print("using UDP port:"+str(self.udpport)+" for Skyview data")
            self.ser.bind(("",self.udpport))
            
            #Prevent the socket from blocking until it receives all the data it wants
            #Note: Instead of blocking, it will throw a socket.error exception if it
            #doesn't get any data
            #self.ser.settimeout(.1)
            self.ser.setblocking(0)

        # if this input is not the first input then don't default to read the ahrs input.
        if(num==0): 
            defaultUseAHRS = True
        else: 
            defaultUseAHRS = False
        self.use_ahrs = _input_file_utils.readConfigBool(self.name, "use_ahrs", defaultUseAHRS)
        if(self.use_ahrs==False):
            print("Skipping AHRS data from Skyview input")

        # create a empty imu object.
        self.imuData = IMUData()
        self.imuData.id = "skyview_imu"
        self.imuData.name = self.name
        self.imu_index = len(dataship.imuData)  # Start at 0
        dataship.imuData.append(self.imuData)
        self.last_read_time = time.time()
        if dataship.debug_mode>0:
            print("new skyview imu "+str(self.imu_index)+": "+str(self.imuData))       

        # create a empty nav object.
        self.navData = NavData()
        self.navData.name = "skyview_nav"
        self.nav_index = len(dataship.navData)  # Start at 0
        self.navData.id = "skyview_nav"+str(len(dataship.navData))
        dataship.navData.append(self.navData)

        # create a empty engine object.
        self.engineData = EngineData()
        self.engineData.name = "skyview_engine"
        self.engineData.index = len(dataship.engineData)  # Start at 0
        self.engineData.id = "skyview_engine"+str(len(dataship.engineData))
        dataship.engineData.append(self.engineData)

        # create a empty fuel object.
        self.fuelData = FuelData()
        self.fuelData.name = "skyview_fuel"
        self.fuelData.index = len(dataship.fuelData)  # Start at 0
        self.fuelData.id = "skyview_fuel"+str(len(dataship.fuelData))
        dataship.fuelData.append(self.fuelData)

        # create a empty gps object.
        self.gpsData = GPSData()
        self.gpsData.name = "skyview_gps"
        self.gps_index = len(dataship.gpsData)  # Start at 0
        self.gpsData.id = "skyview_gps"+str(len(dataship.gpsData))
        dataship.gpsData.append(self.gpsData)

        # create a empty air object.
        self.airData = AirData()
        self.airData.name = "skyview_air"
        self.airData.index = len(dataship.airData)  # Start at 0
        self.airData.id = "skyview_air"+str(len(dataship.airData))
        dataship.airData.append(self.airData)

        # create a empty targets object.
        self.targetData = TargetData()
        self.targetData.id = "skyview_targets"
        self.targetData.source = "skyview"
        self.targetData.name = self.name
        self.targetData_index = len(dataship.targetData)  # Start at 0
        dataship.targetData.append(self.targetData)
        if dataship.debug_mode>0:
            print("new skyview targets "+str(self.targetData_index)+": "+str(self.targetData))

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
            while x != b'!': # read until  !
                t = self.ser.read(1)
                x = t
                if len(t) != 0:
                    if t == b'!':     # May be a Dynon Skyview Message
                        if dataship.debug_mode>0: print("\n! ", end ="." )
                        t =  self.ser.read(1)
                        if t == b'1': # Skyview ADHAES message with 74 bytes
                            data = bytearray(b'!1')
                            data.extend(self.ser.read(72))
                            if dataship.debug_mode>0:
                                print("Skyview ADHAES message with 74 bytes")
                                if dataship.debug_mode>1: print(str(data))
                            return data
                        elif t == b'2': # Skyview NAV/AP message with 93 bytes
                            data = bytearray(b'!2')
                            data.extend(self.ser.read(91))
                            if dataship.debug_mode>0:
                                print("Skyview NAV/AP message with 93 bytes")
                                if dataship.debug_mode>1: print(str(data))
                            return data
                        elif t == b'3': # Skyview Engine data message with 225 bytes
                            data = bytearray(b'!3')
                            data.extend(self.ser.read(223))
                            if dataship.debug_mode>0:
                                print("Skyview Engine data message with 225 bytes")
                                if dataship.debug_mode>1: print(str(data))
                            return data
                        else: # Not a Skyview message so pass
                            pass
                else:
                    self.ser.seek(0)
                    if dataship.debug_mode>0: print("Skyview file reset")
        else:
            try:
                #Attempt to receive up to 1024 bytes of data
                #if dataship.debug_mode>0: print("Trying to read 1024 bytes")
                rec_data = self.ser.recvfrom(1024)
                data = bytearray(rec_data[0])
                if dataship.debug_mode>0: print("Skyview Data received, first byte: "+str(data[0]))
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
        if dataship.debug_mode>0:
            print("---------------network_skyview-------------------------------------\nNEW Chunk len:"+str(len(msg)))
            print("msg[0:4]:", msg[0:4])
        if msg[0] == ord('!') and len(msg) == 392:  # set of 3 Skyview messages
            if msg[1] == ord('1'):
                msg1 = msg[0:74]
                if dataship.debug_mode>0:
                    print("Decode Skyview Type 1; ADHAES message")
                    if dataship.debug_mode>1: print(msg1.hex())
                dataship = self.processSingleSkyviewMessage(msg1,dataship)
            if msg[75] == ord('2'):
                if dataship.debug_mode>0: print("Decode Skyview Type 2; NAV/AP message")
                msg2 = msg[74:167]
                dataship = self.processSingleSkyviewMessage(msg2,dataship)
            if msg[168] == ord('3'):
                if dataship.debug_mode>0: print("Decode Skyview Type 3; Engine Data message")
                msg3 = msg[167:393]
                dataship = self.processSingleSkyviewMessage(msg3,dataship)
            return dataship
        elif msg[0] == ord('!'): # probably a single Skyview message
            #print(len(msg))
            if msg[1]==ord('1') and len(msg)==74:
                msg1 = msg[0:74]
                if dataship.debug_mode>0:
                    print("Decode Skyview Type 1; ADHAES message")
                    if dataship.debug_mode>1: print(msg1.hex())
            elif msg[1]==ord('2') and len(msg)==93:
                msg2 = msg[0:93]
                if dataship.debug_mode>0:
                    print("Decode Skyview Type 2; NAV/AP message")
                    if dataship.debug_mode>1: print(msg2.hex())
            elif msg[1]==ord('3') and len(msg)==225:
                msg3 = msg[0:225]
                if dataship.debug_mode>0:
                    print("Decode Skyview Type 3; Engine Data message")
                    if dataship.debug_mode>1: print(msg3.hex())
            dataship = self.processSingleSkyviewMessage(msg,dataship)
            return dataship
    
    def processSingleSkyviewMessage(self, msg, dataship: Dataship):
        dataType, dataVer = struct.unpack(">BB", msg[1:3])
        if dataship.debug_mode>0:
            print("processSingleSkyviewMessage; length of data: ",len(msg))
            print("dataType: "+str(dataType)+" dataVer: "+str(dataVer))
            if dataship.debug_mode>1: print(msg)

        if isinstance(dataType,str):
            dataType = dataType.encode() # if read from file then convert to bytes
            dataVer = dataVer.encode()
        try:
            if True:
                if dataType == ord('1'):  # AHRS message
                    if(isinstance(msg,str)):
                        msg = msg.encode() # if read from file then convert to bytes
                        if dataship.debug_mode==2: print("ADHARS !1", msg)
                    HH, MM, SS, FF, pitch, roll, HeadingMAG, IAS, PresAlt, TurnRate, LatAccel, VertAccel, AOA, VertSpd, OAT, TAS, Baro, DA, WD, WS, Checksum, CRLF = struct.unpack(
                         # Format string breakdown:
                         # 8s - System time (8 bytes)
                         # 4s - Pitch (4 bytes)
                         # 5s - Roll  (5 bytes)
                         # 3s - Heading  (3 bytes)
                         # 4s - IAS (4 bytes)
                         # 6s - Pres Alt (6 bytes)
                         # 4s - Turn Rate (4 bytes)
                         # 3s - Lat Accel (3 bytes)
                         # 3s - Vert Accel (3 bytes)
                         # 2s - AOA (2 bytes)
                         # 4s - Vertical Speed (4 bytes)
                         # 3s - OAT (3 bytes)
                         # 4s - TAS (4 bytes)
                         # 3s - Baro Setting (3 bytes)
                         # 6s - Density Altitude (6 bytes)
                         # 3s - Wind Direction (3 bytes)
                         # 2s - Wind Speed (2 bytes)
                         # 2s - Checksum (2 bytes)
                         # 2s - CRLF (2 bytes)                            
                        ">2s2s2s2s4s5s3s4s6s4s3s3s2s4s3s4s3s6s3s2s2s2s", msg[3:75]
                    ) 
                    self.gpsData.GPSTime_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    self.time_stamp_string = dataship.sys_time_string
                    
                    #print("time: "+dataship.sys_time_string)
                    self.imuData.pitch = Input.cleanInt(self,pitch) / 10
                    self.imuData.roll = Input.cleanInt(self,roll) / 10
                    self.imuData.mag_head = Input.cleanInt(self,HeadingMAG)

                    # Update IMU data
                    self.imuData.yaw = self.imuData.mag_head
                    if dataship.debug_mode > 0:
                        current_time = time.time() # calculate hz.
                        self.imuData.hz = round(1 / (current_time - self.last_read_time), 1)
                        self.last_read_time = current_time

                    self.airData.IAS = Input.cleanInt(self,IAS) * 0.1
                    self.airData.Alt_pres = Input.cleanInt(self,PresAlt)
                    self.airData.OAT = (Input.cleanInt(self,OAT) * 1.8) + 32 # c to f
                    self.airData.TAS = Input.cleanInt(self,TAS) * 0.1
                    if AOA == b'XX':
                        self.airData.AOA = 0
                    else:
                        self.airData.AOA = Input.cleanInt(self,AOA)
                    self.airData.Baro = (Input.cleanInt(self,Baro) + 2750.0) / 100
                    self.airData.Baro_diff = self.airData.Baro - 29.921
                    self.airData.Alt_da = Input.cleanInt(self,DA)
                    self.airData.Alt = int( Input.cleanInt(self,PresAlt) + (self.airData.Baro_diff / 0.00108) )  # 0.00108 of inches of mercury change per foot.
                    self.imuData.turn_rate = Input.cleanInt(self,TurnRate) * 0.1
                    self.airData.VSI = Input.cleanInt(self,VertSpd) * 10
                    self.imuData.vert_G = Input.cleanInt(self,VertAccel) * 0.1
                    try:
                        self.airData.Wind_dir = Input.cleanInt(self,WD)
                        self.airData.Wind_speed = Input.cleanInt(self,WS)
                        self.airData.Wind_dir_corr = (self.imuData.mag_head - self.airData.Wind_dir) % 360 #normalize the wind direction to the airplane heading
                        # # compute Gnd Speed when Gnd Speed is unknown (not provided in data)
                        # dataship.gndspeed = math.sqrt(math.pow(dataship.tas,2) + math.pow(dataship.wind_speed,2) + (2 * dataship.tas * dataship.wind_speed * math.cos(math.radians(180 - (dataship.wind_dir - dataship.mag_head)))))
                        # dataship.gndtrack = dataship.mag_head 
                    except ValueError as ex:
                        # if error trying to parse wind then must not have that info.
                        self.airData.Wind_dir = None
                        self.airData.Wind_speed = None
                        self.airData.Wind_dir_corr = None #normalize the wind direction to the airplane heading

                    self.airData.msg_count += 1
                    self.imuData.msg_count += 1

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)

                elif dataType == ord('2'): #Skyview System message (NAV,AP, etc)
                    self.navData.msg_count += 1
                    if isinstance(msg, str): msg = msg.encode()  # if read from file then convert to bytes
                    HH,MM,SS,FF,HBug,AltBug, ASIBug,VSBug,Course,CDISrcType,CDISourePort,CDIScale,CDIDeflection,GS,APEng,APRollMode,Not1,APPitch,Not2,APRollF,APRollP,APRollSlip,APPitchF, APPitchP,APPitchSlip,APYawF,APYawP,APYawSlip,TransponderStatus,TransponderReply,TransponderIdent,TransponderCode,DynonUnused,Checksum,CRLF= struct.unpack(
                         # Format string breakdown:
                         # 8s - System time (8 bytes)
                         # 3s - Heading bug (3 bytes)
                         # 5s - Altitude bug (5 bytes)
                         # 4s - Airspeed bug (4 bytes)
                         # 4s - Vertical speed bug (4 bytes)
                         # 3s - Course (3 bytes)
                         # c  - CDI source type (1 byte)
                         # c  - CDI source port (1 byte)
                         # 2s - CDI scale (2 bytes)
                         # 3s - CDI deflection (3 bytes)
                         # 3s - Glide slope % (3 bytes)
                         # c  - Autopilot engaged (1 byte)
                         # c  - AP roll mode (1 byte)
                         # c  - Not used (1 byte)
                         # c  - AP pitch mode (1 byte)
                         # c  - Not used (1 byte)
                         # 3s - AP roll force (3 bytes)
                         # 5s - AP roll position (5 bytes)
                         # c  - AP roll slip (1 byte)
                         # 3s - AP pitch force (3 byte)
                         # 5s - AP pitch position (5 bytes)
                         # c  - AP pitch slip (1 byte)
                         # 3s - AP yaw force (3 bytes)
                         # 5s - AP yaw position (5 bytes)
                         # c  - AP yaw slip (1 byte)
                         # c  - Transponder status (1 byte)
                         # c  - Transponder reply (1 byte)
                         # c  - Transponder Ident (1 byte)
                         # 4s - Transponder code (4 bytes)
                         # 10s- Not used (10 bytes)
                         # 2s - Checksum (2 bytes)
                         # 2s - CRLF (2 bytes)
                        ">2s2s2s2s3s5s4s4s3scc2s3s3sccccc3s5sc3s5sc3s5scccc4s10s2s2s", msg[3:94]
                    )
                    self.gpsData.GPSTime_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    self.time_stamp_string = self.gpsData.GPSTime_string

                    if HBug != b'XXX': self.navData.HeadBug = Input.cleanInt(self, HBug)
                    if AltBug != b'XXXXX': self.navData.AltBug = Input.cleanInt(self,AltBug) * 10
                    if ASIBug != b'XXXX': self.navData.ASIBug = Input.cleanInt(self,ASIBug) / 10
                    if VSBug != b'XXXX': self.navData.VSBug = Input.cleanInt(self,VSBug) / 10
                    if CDIDeflection != b'XXX': self.navData.ILSDev = Input.cleanInt(self,CDIDeflection)
                    if GS != b'XXX': self.navData.GSDev = Input.cleanInt(self,GS)
                    self.navData.HSISource = Input.cleanInt(self,CDISourePort)
                    if CDISrcType == b'0':
                        navSourceType = 'GPS'
                    elif CDISrcType == b'1':
                        navSourceType = 'NAV'
                    elif CDISrcType == b'2':
                        navSourceType = 'LOC'
                    self.navData.SourceDesc = navSourceType + str(Input.cleanInt(self,CDISourePort))
                    if CDIScale != b'XX': self.navData.GLSHoriz = Input.cleanInt(self,CDIScale) / 10
                    if APEng == b'0': self.navData.APeng = 0
                    if APEng == b'1' or APEng == b'2' or APEng == b'3' or APEng == b'4' or APEng == b'5' or APEng == b'6' or APEng == b'7': self.navData.APeng = 1
                    self.navData.AP_RollForce = Input.cleanInt(self,APRollF)
                    if APRollP != b'XXXXX': self.navData.AP_RollPos = Input.cleanInt(self,APRollP)
                    self.navData.AP_RollSlip = Input.cleanInt(self,APRollSlip)
                    self.navData.AP_PitchForce = Input.cleanInt(self,APPitchF)
                    if APPitchP != b'XXXXX': self.navData.AP_PitchPos = Input.cleanInt(self,APPitchP)
                    self.navData.AP_PitchSlip = Input.cleanInt(self,APPitchSlip)
                    self.navData.AP_YawForce = Input.cleanInt(self,APYawF)
                    if APYawP != b'XXXXX': self.navData.AP_YawPos = Input.cleanInt(self,APYawP)
                    self.navData.AP_YawSlip = Input.cleanInt(self,APYawSlip)
                    if TransponderStatus == b'0':
                        self.navData.XPDR_Status = 'SBY'
                    elif TransponderStatus == b'1':
                        self.navData.XPDR_Status = 'GND'
                    elif TransponderStatus == b'2':
                        self.navData.XPDR_Status = 'ON'
                    elif TransponderStatus == b'3':
                        self.navData.XPDR_Status = 'ALT'
                    if TransponderReply != b'X': self.navData.XPDR_Reply = Input.cleanInt(self,TransponderReply)
                    if TransponderIdent != b'X': self.navData.XPDR_Ident = Input.cleanInt(self,TransponderIdent)
                    if TransponderCode != b'XXXX': self.navData.XPDR_Code = Input.cleanInt(self,TransponderCode)
                    
                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)

                elif dataType == ord('3'): #Skyview EMS Engine data message
                    self.engineData.msg_count += 1
                    self.fuelData.msg_count += 1
                    if isinstance(msg,str):msg = msg.encode() # if read from file then convert to bytes
                    HH,MM,SS,FF,OilPress,OilTemp,RPM_L,RPM_R,MAP,FF1,FF2,FP,FL_L,FL_R,Frem,V1,V2,AMPs,Hobbs,Tach,TC1,TC2,TC3,TC4,TC5,TC6,TC7,TC8,TC9,TC10,TC11,TC12,TC13,TC14,GP1,GP2,GP3,GP4,GP5,GP6,GP7,GP8,GP9,GP10,GP11,GP12,GP13,Contacts,Pwr,EGTstate,Checksum,CRLF=struct.unpack(
                         # 8s - System time (8 bytes)
                         # 3s - Oil pressure (3 bytes)
                         # 4s - Oil temperature (4 bytes)
                         # 4s - Left RPM (4 bytes)
                         # 4s - Right RPM (4 bytes)
                         # 3s - Manifold pressure (3 bytes)
                         # 3s - Fuel flow 1 (3 bytes)
                         # 3s - Fuel flow 2 (3 bytes)
                         # 3s - Fuel pressure (3 bytes)
                         # 3s - Left fuel quantity (3 bytes)
                         # 3s - Right fuel quantity (3 bytes)
                         # 3s - Fuel remaining (3 bytes)
                         # 3s - Voltage 1 (3 bytes)
                         # 3s - Voltage 2 (3 bytes)
                         # 4s - Amperage (4 bytes)
                         # 5s - Hobbs time (5 bytes)
                         # 5s - Tach time (5 bytes)
                         # 14 Thermocouples, each 4 bytes:
                         # 4s - Thermocouple 1 (4 bytes)
                         # 4s - Thermocouple 2 (4 bytes)
                         # 4s - Thermocouple 3 (4 bytes)
                         # 4s - Thermocouple 4 (4 bytes)
                         # 4s - Thermocouple 5 (4 bytes)
                         # 4s - Thermocouple 6 (4 bytes)
                         # 4s - Thermocouple 7 (4 bytes)
                         # 4s - Thermocouple 8 (4 bytes)
                         # 4s - Thermocouple 9 (4 bytes)
                         # 4s - Thermocouple 10 (4 bytes)
                         # 4s - Thermocouple 11 (4 bytes)
                         # 4s - Thermocouple 12 (4 bytes)
                         # 4s - Thermocouple 13 (4 bytes)
                         # 4s - Thermocouple 14 (4 bytes)
                         # Second part - General purpose inputs and status:
                         # 6s- General purpose input 1 (6 bytes)
                         # 6s- General purpose input 2 (6 bytes)
                         # 6s- General purpose input 3 (6 bytes)
                         # 6s- General purpose input 4 (6 bytes)
                         # 6s- General purpose input 5 (6 bytes)
                         # 6s- General purpose input 6 (6 bytes)
                         # 6s- General purpose input 7 (6 bytes)
                         # 6s- General purpose input 8 (6 bytes)
                         # 6s- General purpose input 9 (6 bytes)
                         # 6s- General purpose input 10 (6 bytes)
                         # 6s- General purpose input 11 (6 bytes)
                         # 6s- General purpose input 12 (6 bytes)
                         # 6s- General purpose input 13 (6 bytes)
                         # 16c- Contact inputs status (Not Used) (16 bytes)
                         # 3s - Power percentage (3 bytes)
                         # 1s - EGT leaning state (1 byte)
                         # 2s - Checksum (2 bytes)
                         # 2s - CRLF (2 bytes)
                         ">2s2s2s2s3s4s4s4s3s3s3s3s3s3s3s3s3s4s5s5s4s4s4s4s4s4s4s4s4s4s4s4s4s4s6s6s6s6s6s6s6s6s6s6s6s6s6s16s3s1s2s2s", msg[3:226]
                    )

                    #dataship.sys_time_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    #self.time_stamp_string = dataship.sys_time_string

                    if OilPress != b'XXX': self.engineData.OilPress = Input.cleanInt(self,OilPress)
                    if OilTemp != b'XXXX': self.engineData.OilTemp = Input.cleanInt(self,OilTemp)
                    self.engineData.RPM = max(Input.cleanInt(self,RPM_L), Input.cleanInt(self,RPM_R))
                    if MAP != b'XXX': self.engineData.ManPress = Input.cleanInt(self,MAP) / 10
                    self.engineData.FuelFlow = Input.cleanInt(self,FF1) / 10
                    self.engineData.FuelFlow2 = Input.cleanInt(self,FF2) / 10
                    if FP != b'XXX': self.engineData.FuelPress = Input.cleanInt(self,FP) / 10
                    fuel_level_left  = Input.cleanInt(self, FL_L) / 10
                    fuel_level_right = Input.cleanInt(self, FL_R) / 10
                    self.fuelData.FuelLevels = [fuel_level_left, fuel_level_right, 0, 0]
                    if Frem != b'XXXX': self.fuelData.FuelRemain = Input.cleanInt(self,Frem) / 10
                    self.engineData.volts1 = Input.cleanInt(self,V1) / 10
                    self.engineData.volts2 = Input.cleanInt(self,V2) / 10
                    if AMPs != b'XXXX': self.engineData.amps = Input.cleanInt(self,AMPs) / 10
                    self.engineData.hobbs_time = Input.cleanInt(self,Hobbs) / 10
                    self.engineData.tach_time = Input.cleanInt(self,Tach) / 10

                    if TC12 != b'XXXX': self.engineData.EGT[0] = round(((Input.cleanInt(self, TC12)) * 1.8) + 32)  # convert from C to F
                    if TC10 != b'XXXX': self.engineData.EGT[1] = round(((Input.cleanInt(self, TC10)) * 1.8) + 32)  # convert from C to F
                    if TC8 != b'XXXX': self.engineData.EGT[2] = round(((Input.cleanInt(self, TC8)) * 1.8) + 32)  # convert from C to F
                    if TC6 != b'XXXX': self.engineData.EGT[3] = round(((Input.cleanInt(self, TC6)) * 1.8) + 32)  # convert from C to F
                    if TC4 != b'XXXX': self.engineData.EGT[4] = round(((Input.cleanInt(self, TC4)) * 1.8) + 32)  # convert from C to F
                    if TC2 != b'XXXX': self.engineData.EGT[5] = round(((Input.cleanInt(self, TC2)) * 1.8) + 32)  # convert from C to F

                    if TC11 != b'XXXX': self.engineData.CHT[0] = round(((Input.cleanInt(self, TC11)) * 1.8) + 32)  # convert from C to F
                    if TC9 != b'XXXX': self.engineData.CHT[1] = round(((Input.cleanInt(self, TC9)) * 1.8) + 32)  # convert from C to F
                    if TC7 != b'XXXX': self.engineData.CHT[2] = round(((Input.cleanInt(self, TC7)) * 1.8) + 32)  # convert from C to F
                    if TC5 != b'XXXX': self.engineData.CHT[3] = round(((Input.cleanInt(self, TC5)) * 1.8) + 32)  # convert from C to F
                    if TC3 != b'XXXX': self.engineData.EGT[4] = round(((Input.cleanInt(self, TC3)) * 1.8) + 32)  # convert from C to F
                    if TC1 != b'XXXX': self.engineData.EGT[5] = round(((Input.cleanInt(self, TC1)) * 1.8) + 32)  # convert from C to F

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)
                else:
                    pass
                    #self.msg_unknown += 1 # unknown message found.
        except ValueError:
            self.msg_bad += 1
            print("bad: "+str(msg))
            pass
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            dataship.errorFoundNeedToExit = True

        if self.isPlaybackMode:  #if play back mode then add a delay.  Else reading a file is way to fast.
            time.sleep(.05)
        return dataship

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
