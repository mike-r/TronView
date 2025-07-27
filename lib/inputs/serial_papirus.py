#!/usr/bin/env python

#  Version 1.0a testing
print("serial_PaPiRus.py Version 1.0a.Testing")

# 
# This module will send data via RS-232 Serial to a PaPiRus e-Paper display.

#  This module is used to read data from the Dynon HDX and send it to a PaPiRus display.

# TODO Add error handling for opening serial ports
# FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyUSB0'

# TODO Add "Hello World" serial transmission to PaPiRus Pi to signal Comms OK
# TODO Add IP address and Gateway address and send to PaPiRus display

# Raspberry Pi 
# /dev/ttyUSB0 is a USB to RS-232 converter to receive data from

# the Dynon HDX via Serial out.
# /dev/ttyACM0 on Pi 3B, and Pi 5 is a micro-USB cable plugged into a Pi-Zero OTG port
# /dev/ttyAMA0 for Pi-4

# To check serial ports use:   dmesg | grep tty
# To check I2C devices:        sudo i2cdetect -y 0
# To check MQTT status:        sudo systemctl status mosquitto

# To install and setup the MQTT Broker:
#  sudo apt install mosquitto mosquitto-clients
#  sudo systemctl enable mosquitto

# To install MQTT library in Python
#   sudo pip3 install paho-mqtt

# Write Serial data to PaPiRus Pi
# Format of data stream is:
# !41+ssssGhhhhhfff
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "hhhhh"    is the Total Time (Hobbs Time) in tenths of hours from Dynon EMS
# "fff"      is the Total Fuel Remaining in tents of gallons

import serial
from time import sleep
import sys
import os
import socket
from ._input import Input
from lib import hud_utils
from . import _utils
import struct
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_analog import AnalogData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from lib.common.dataship.dataship_gps import GPSData
from lib.common.dataship.dataship_air import AirData

class serial_papirus(Input):
    def __init__(self):
        self.name = "papirus"
        self.version = 1.0
        self.inputtype = "serial"
        self.EOL = 10
        self.analogData = AnalogData()
        self.engineData = EngineData()
        self.fuelData = FuelData()
        self.gpsData = GPSData()
        self.airData = AirData()
        self.msg_unknown = 0
        self.msg_bad = 0
        self.nmea_buffer = ""  # Add buffer for NMEA messages

#########################################

# Define serial link to PaPaRus display Pi via OTG cable to PiZero
    papirus_serial = serial.Serial(
        baudrate=9600,
        port=  '/dev/ttyACM0',
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        writeTimeout=5
    )

    hobbs           = 0   # Initialize hobbs meter as 0 (don't I wish...)
    smoke           = 0   # Initialize smoke oil tank as empty
    flap_position   = 0   # Initialize flaps as up
    old_IAS         = 1.1
    old_fuel_remain = 1.1
    old_smoke       = 1.1
    old_hobbs       = 1
    old_flap_pos    = 1
    update          = False
    tx_count        = 0
    loop_count     = 0     # print only max_print loops of debug messages
    max_print       = 10

    print("Welcome to N221TM's Raspberry Pi", sep=' ', end='\n\n\n')
    print("Starting loop to read data from Dynon HDX:")
    print()

    try:
        gw = os.popen("ip -4 route show default").read().split()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((gw[2], 0))
        ipaddr = s.getsockname()[0]
        gateway = gw[2]
        host = socket.gethostname()
        print ("IP:", ipaddr, " GW:", gateway, " Host:", host)
    except:
        print("Error: Unable to get IP address")

#  Send message to PaPiRus display pi every 5 secondsto signal that comms are OK
#  and to test the serial link

    papirus_str = "!51" + ipaddr + "\r\n"
    papirus_bytes = papirus_str.encode()
    print("Sending PaPiRus message: ", papirus_str)

    while True:
        try:
            papirus_serial.write(papirus_bytes)         # Send data to PaPiRus
        except Exception as e:
            print(e)
            print("Unexpected error in write to PaPiRus: ", e)
        papirus_bytes = papirus_serial.read_until(b'\r\n', None)
        if papirus_bytes == b'':
            print("No data received from PaPiRus, retrying...")
            continue
        else:
            papirus_str = papirus_bytes.decode().strip()
            print("Received from PaPiRus:", papirus_str)
            break
        sleep(1)

    sys.exit(0)  # Exit the program after sending the dummy message
    #  ##############################################################
    #  ##############################################################


    while True:
        print('Input test string from Dynon:')                  # For Testing take input from console
        dynon_str = input()

        print(dynon_str)
        print()
        if dynon_str[57:62].isnumeric():
            hobbs = float(dynon_str[57:62]) /10
            if old_hobbs != hobbs:
                if loop_count < max_print:  print('Hobbs:         {0:6.1f}' .format(hobbs), 'Hours')
                pub= "Hobbs," +  dynon_str[57:62]
                client_cloud.publish("1TM", pub)
                old_hobbs = hobbs
                update = True
        else: print('Trash for Hobbs:', dynon_str[57:62])
        if loop_count < max_print:  print()

# Build text string to send to PaPiRus display pi
        if update or tx_count > 10:
            smoke_str = dynon_str[141:147]      #   "+nnnnG"
            hobbs_str = dynon_str[57:62]
            uel_remain_str = dynon_str[44:47]
            papirus_str = '!41' + smoke_str + hobbs_str + fuel_remain_str + '\r\n'
            if loop_count < max_print:  print("To  Papirus:", papirus_str)
            papirus_bytes = papirus_str.encode()
            print(papirus_bytes)
            try:
                papirus_serial.write(papirus_bytes)         # Send data to PaPiRus
            except Exception as e:
                print(e)
                print("Unexpected error in write to PaPiRus: ", e)
            update = False
        tx_count = 0
        loop_count = loop_count + 1
    sleep(1)



