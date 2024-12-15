#!/usr/bin/env python

# Serial input source
# Skyview
# 1/23/2019  Topher
# 11/6/2024  Added IMU data
# 11/30/2024 Added NAV, Autopilot and EMS data   Rehberg

from ._input import Input
from lib import hud_utils
import math, sys
import serial
import struct
from lib import hud_text
import time
from lib.common.dataship.dataship import IMU
import traceback


class serial_skyview(Input):
    def __init__(self):
        self.name = "skyview"
        self.version = 1.0
        self.inputtype = "serial"
        self.EOL = 10

    def initInput(self,num,aircraft):
        Input.initInput( self,num, aircraft )  # call parent init Input.
        
        if(self.PlayFile!=None and self.PlayFile!=False):
            # load playback file.
            if self.PlayFile==True:
                defaultTo = "dynon_skyview_data1.txt"
                self.PlayFile = hud_utils.readConfig(self.name, "playback_file", defaultTo)
            self.ser,self.input_logFileName = Input.openLogFile(self,self.PlayFile,"r")
            self.isPlaybackMode = True
        else:
            self.efis_data_format = hud_utils.readConfig("DataInput", "format", "none")
            self.efis_data_port = hud_utils.readConfig("DataInput", "port", "/dev/ttyS0")
            self.efis_data_baudrate = hud_utils.readConfigInt(
                "DataInput", "baudrate", 115200
            )

            # open serial connection.
            self.ser = serial.Serial(
                port=self.efis_data_port,
                baudrate=self.efis_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )

        # create a empty imu object.
        self.imuData = IMU()
        self.imuData.id = "skyview_imu"
        self.imuData.name = self.name
        self.imu_index = len(aircraft.imus)  # Start at 0
        aircraft.imus[self.imu_index] = self.imuData
        self.last_read_time = time.time()

    # close this data input
    def closeInput(self,aircraft):
        if self.isPlaybackMode:
            self.ser.close()
        else:
            self.ser.close()

    #############################################
    ## Function: readMessage
    def readMessage(self, aircraft):
        if aircraft.errorFoundNeedToExit:
            return aircraft;
        try:
            x = 0
            while x != 33:  # 33(!) is start of dynon skyview.
                t = self.ser.read(1)
                if len(t) != 0:
                    x = ord(t)
                else:
                    if self.isPlaybackMode:  # if no bytes read and in playback mode.  then reset the file pointer to the start of the file.
                        self.ser.seek(0)
                    return aircraft
            dataType = self.ser.read(1)
            dataVer  = self.ser.read(1)

            if isinstance(dataType,str):
                dataType = dataType.encode() # if read from file then convert to bytes
                dataVer = dataVer.encode()

            if True:
                #msg = (msg[:73]) if len(msg) > 73 else msg
                #aircraft.msg_last = msg
                if dataType == b'1':  # AHRS message
                    msg = self.ser.read(71)
                    if isinstance(msg, str): msg = msg.encode() # if read from file then convert to bytes
                    HH, MM, SS, FF, pitch, roll, HeadingMAG, IAS, PresAlt, TurnRate, LatAccel, VertAccel,AOA, VertSpd, OAT, TAS, Baro, DA, WD, WS, Checksum, CRLF = struct.unpack(
                        "2s2s2s2s4s5s3s4s6s4s3s3s2s4s3s4s3s6s3s2s2s2s", msg
                    )
                    #print("AHRS Message !1:", msg)
                    aircraft.sys_time_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    self.time_stamp_string = aircraft.sys_time_string
                    self.time_stamp_min = int(MM)
                    self.time_stamp_sec = int(SS)
                    
                    #print("time: "+aircraft.sys_time_string)
                    #print("pitch:"+str(pitch))
                    aircraft.pitch = Input.cleanInt(self,pitch) / 10
                    #print("roll:"+str(roll))
                    aircraft.roll = Input.cleanInt(self,roll) / 10
                    #print("HeadingMAG:"+str(HeadingMAG))
                    aircraft.mag_head = Input.cleanInt(self,HeadingMAG)

                    # Update IMU data
                    self.imuData.heading = aircraft.mag_head
                    if aircraft.debug_mode > 0:
                        current_time = time.time() # calculate hz.
                        self.imuData.hz = round(1 / (current_time - self.last_read_time), 1)
                        self.last_read_time = current_time
                    # Update the IMU in the aircraft's imu list
                    self.imuData.updatePos(aircraft.pitch, aircraft.roll, aircraft.mag_head)
                    aircraft.imus[self.imu_index] = self.imuData

                    #print("IAS:"+str(IAS))
                    aircraft.ias = Input.cleanInt(self,IAS) * 0.1
                    #print("PALT:"+str(PresAlt))
                    aircraft.PALT = Input.cleanInt(self,PresAlt)
                    #print("TurnRate:"+str(TurnRate))
                    #print("OAT:"+str(OAT))
                    aircraft.oat = (Input.cleanInt(self,OAT) * 1.8) + 32 # c to f
                    #print("TAS:"+str(TAS))
                    aircraft.tas = Input.cleanInt(self,TAS) * 0.1
                    #print("AOA:"+str(AOA))
                    if AOA == "XX":
                        aircraft.aoa = 0
                    else:
                        aircraft.aoa = Input.cleanInt(self,AOA)
                    #print("baro:"+str(Baro))
                    aircraft.baro = (Input.cleanInt(self,Baro) + 2750.0) / 100
                    aircraft.baro_diff = aircraft.baro - 29.921
                    aircraft.DA = Input.cleanInt(self,DA)
                    aircraft.alt = int( Input.cleanInt(self,PresAlt) + (aircraft.baro_diff / 0.00108) )  # 0.00108 of inches of mercury change per foot.
                    aircraft.BALT = aircraft.alt
                    aircraft.turn_rate = Input.cleanInt(self,TurnRate) * 0.1
                    aircraft.vsi = Input.cleanInt(self,VertSpd) * 10
                    aircraft.vert_G = Input.cleanInt(self,VertAccel) * 0.1
                    try:
                        aircraft.wind_dir = Input.cleanInt(self,WD)
                        aircraft.wind_speed = Input.cleanInt(self,WS)
                        aircraft.norm_wind_dir = (aircraft.mag_head - aircraft.wind_dir) % 360 #normalize the wind direction to the airplane heading
                        # compute Gnd Speed when Gnd Speed is unknown (not provided in data)
                        aircraft.gndspeed = math.sqrt(math.pow(aircraft.tas,2) + math.pow(aircraft.wind_speed,2) + (2 * aircraft.tas * aircraft.wind_speed * math.cos(math.radians(180 - (aircraft.wind_dir - aircraft.mag_head)))))
                        aircraft.gndtrack = aircraft.mag_head 
                    except ValueError as ex:
                        # if error trying to parse wind then must not have that info.
                        aircraft.wind_dir = 0
                        aircraft.wind_speed = 0
                        aircraft.norm_wind_dir = 0 #normalize the wind direction to the airplane heading
                        aircraft.gndspeed = 0
                    aircraft.msg_count += 1

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)

                elif dataType == b'2': #Dynon System message (nav,AP, etc)
                    aircraft.nav.msg_count += 1
                    msg = self.ser.read(90)
                    if isinstance(msg, str): msg = msg.encode()  # if read from file then convert to bytes
                    HH,MM,SS,FF,HBug,AltBug, AirBug,VSBug,Course,CDISrcType,CDISourePort,CDIScale,CDIDeflection,GS,APEng,APRollMode,Not1,APPitch,Not2,APRollF,APRollP,APRollSlip,APPitchF, APPitchP,APPitchSlip,APYawF,APYawP,APYawSlip,TransponderStatus,TransponderReply,TransponderIdent,TransponderCode,DynonUnused,Checksum,CRLF= struct.unpack(
                        "2s2s2s2s3s5s4s4s3scc2s3s3sccccc3s5sc3s5sc3s5scccc4s10s2s2s", msg
                    )
                    #print("NAV & System Message !2:", msg)
                    aircraft.sys_time_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    self.time_stamp_string = aircraft.sys_time_string
                    self.time_stamp_min = int(MM)
                    self.time_stamp_sec = int(SS)

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)

                elif dataType == b'3': #Dynon EMS Engine data message
                    aircraft.engine.msg_count += 1
                    msg = self.ser.read(222)
                    if isinstance(msg,str):msg = msg.encode() # if read from file then convert to bytes
                    HH,MM,SS,FF,OilPress,OilTemp, RPM_L,RPM_R,MAP,FF1,FF2,FP,FL_L,FL_R,Frem,V1,V2,AMPs,Hobbs,Tach,TC1,TC2,TC3,TC4,TC5,TC6,TC7,TC8,TC9,TC10,TC11,TC12,TC13,TC14,GP1,GP2,GP3,GP4,GP5,GP6,GP7,GP8,GP9,GP10,GP11,GP12,GP13,Contacts,Pwr,EGTstate,Checksum,CRLF= struct.unpack(
                        "2s2s2s2s3s4s4s4s3s3s3s3s3s3s3s3s3s4s5s5s4s4s4s4s4s4s4s4s4s4s4s4s4s4s6s6s6s6s6s6s6s6s6s6s6s6s6s16s3s1s2s2s", msg
                    )
                    #print("EMS Message !3:", msg)
                    aircraft.sys_time_string = "%d:%d:%d"%(int(HH),int(MM),int(SS))
                    self.time_stamp_string = aircraft.sys_time_string
                    self.time_stamp_min = int(MM)
                    self.time_stamp_sec = int(SS)

                    aircraft.engine.OilPress = Input.cleanInt(self,OilPress)
                    #print("Oil Pressure:"+str(OilPress)+"  :",aircraft.engine.OilPress, " PSI")

                    aircraft.engine.OilTemp = Input.cleanInt(self,OilTemp)
                    #print("Oil Temp:"+str(OilTemp))

                    aircraft.engine.RPM = Input.cleanInt(self,RPM_L)
                    #print("RPM:"+str(RPM_L)+"  :", aircraft.engine.RPM, " RPM")

                    aircraft.engine.ManPress = Input.cleanInt(self,MAP) * 0.1
                    #print("MAP:"+str(MAP)+"   :", aircraft.engine.ManPress, " inHG")

                    aircraft.engine.FuelFlow = Input.cleanInt(self,FF1) / 10
                    #print("Fuel Flow:"+str(FF1)+"  :", aircraft.engine.FuelFlow, " GPH")

                    aircraft.engine.FuelPress = Input.cleanInt(self,FP) / 10
                    #print("Fuel Pressure:"+str(FP))

                    fuel_level_left  = Input.cleanInt(self, FL_L) / 10
                    fuel_level_right = Input.cleanInt(self, FL_R) / 10
                    aircraft.engine.FuelLevels = [fuel_level_left, fuel_level_right, 0, 0]
                    #print("Fuel Level Left: "+str(FL_L)+"  :", aircraft.engine.FuelLevels[0], " Gallons Left Tank")
                    #print("Fuel Level Right:"+str(FL_R)+"  :", aircraft.engine.FuelLevels[1], " Gallons Right Tank")

                    egt_1 = Input.cleanInt(self, TC12)
                    aircraft.engine.EGT[0] = round((egt_1 * 1.8) + 32)  # convert from C to F
                    cht_1 = Input.cleanInt(self, TC11)
                    aircraft.engine.CHT[1] = round((cht_1 * 1.8) + 32)  # convert from C to F

                    egt_2 = Input.cleanInt(self, TC10)
                    aircraft.engine.EGT[1] = round((egt_2 * 1.8) + 32)  # convert from C to F
                    cht_2 = Input.cleanInt(self, TC9)
                    aircraft.engine.CHT[1] = round((cht_2 * 1.8) + 32)  # convert from C to F

                    egt_3 = Input.cleanInt(self, TC8)
                    aircraft.engine.EGT[2] = round((egt_3 * 1.8) + 32)  # convert from C to F

                    egt_4 = Input.cleanInt(self, TC6)
                    aircraft.engine.EGT[3] = round((egt_4 * 1.8) + 32)  # convert from C to F

                    #egt_5 = Input.cleanInt(self, TC4)
                    #aircraft.engine.EGT[4] = round((egt_5 * 1.8) + 32)  # convert from C to F

                    #egt_6 = Input.cleanInt(self, TC2)
                    #aircraft.engine.EGT[5] = round((egt_6 * 1.8) + 32)  # convert from C to F

                    cht_1 = Input.cleanInt(self, TC11)
                    aircraft.engine.CHT[0] = round((cht_1 * 1.8) + 32)  # convert from C to F

                    cht_2 = Input.cleanInt(self, TC9)
                    aircraft.engine.CHT[1] = round((cht_2 * 1.8) + 32)  # convert from C to F

                    cht_3 = Input.cleanInt(self, TC7)
                    aircraft.engine.CHT[2] = round((cht_3 * 1.8) + 32)  # convert from C to F

                    cht_4 = Input.cleanInt(self, TC5)
                    aircraft.engine.CHT[3] = round((cht_4 * 1.8) + 32)  # convert from C to F

                    #cht_5 = Input.cleanInt(self, TC3)
                    #cht_6 = Input.cleanInt(self, TC1)

                    if self.output_logFile != None:
                        Input.addToLog(self,self.output_logFile,bytes([33,int(dataType),int(dataVer)]))
                        Input.addToLog(self,self.output_logFile,msg)
                else:
                    aircraft.msg_unknown += 1 # unknown message found.
            else:
                aircraft.msg_bad += 1 # count this as a bad message
        except ValueError:
            aircraft.msg_bad += 1
            #print("bad:"+str(msg))
            pass
        except struct.error:
            aircraft.msg_bad += 1
            pass
        except serial.serialutil.SerialException:
            print("skyview serial exception")
            traceback.print_exc()
            aircraft.errorFoundNeedToExit = True

        if self.isPlaybackMode:  #if play back mode then add a delay.  Else reading a file is way to fast.
            time.sleep(.05)
        else:
            #pass
            self.ser.flushInput()  # flush the serial after every message else we see delays

        return aircraft


# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
