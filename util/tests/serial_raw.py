#!/usr/bin/env python


import time
import serial
import struct
import sys
import os, getopt, subprocess, platform

version = "0.1"

ser = None
counter = 0


def list_serial_ports(printthem):

    # List all the Serial COM Ports on Raspberry Pi
    proc = subprocess.Popen(['ls /dev/tty[A-Za-z]*'], shell=True, stdout=subprocess.PIPE)
    com_ports = proc.communicate()[0]
    com_ports_list = str(com_ports).split("\\n") # find serial ports
    rtn = []
    for com_port in com_ports_list:
        if 'ttyS' in com_port:
            if(printthem==True): print("found serial port: "+com_port)
            rtn.append(com_port)
        if 'ttyUSB' in com_port:
            if(printthem==True): print("found USB serial port: "+com_port)
            rtn.append(com_port)
    return rtn


# print at location
def print_xy(x, y, text):
    sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
    sys.stdout.flush()


def readMessage():
    global ser
    try:
        t = ser.read(1)
        if(len(t)>0):
            if logging: raw_log.write(t)
            if int(t[0]) < 32:  # if its binary then convert it to string first.
                x = str(t)
            else:
                x = t
            # print(x, end=" ")
            readcount += 1
            if readcount > 15:
                print("15 lines read from serial port")
                readcount = 0

    except serial.serialutil.SerialException:
        print("exception")

if sys.platform.startswith('win'):
    os.system('cls')  # on windows
else:
    # os.system("clear")  # on Linux / os X
    pass
argv = sys.argv[1:]
showBin = 0
port = "/dev/ttyS0"  # default serial port
backup_port = "/dev/ttyUSB0"
baudrate = "115200"   # default baud rate
logging = False

try:
    opts, args = getopt.getopt(argv, "hb:i:s:l", ["bin="])
except getopt.GetoptError:
    print("raw_serial.py -b")
    sys.exit(2)
for opt, arg in opts:
    if opt == "-h":
        print("raw_serial.py [-i <serial port>] -l")
        print(" -h help")
        print(" -b or --bin select binary file to log to")
        print(" -l (list serial ports found)")
        print(" -i select input serial port")
        print(" -s select input serial port baud rate")
        sys.exit()
    if opt == "-i":
        port=arg
    if opt == "-l":
        list_serial_ports(True)
        sys.exit()
    if opt == "-s":
        baudrate=arg
    if opt == "-b" or opt == "--bin":
        logging = True
        filename = arg
        print("Logging to: "+filename)
        try:
            raw_log = open(filename, "ab")
        except:
            raw_log = open("raw_log.bin", "ab")
            print("Unable to open file: "+filename)
            print("Using default file: raw_log.bin")
try:
    ser=serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1,
    )

except:
    print(f"Unable to open primary serial port: {port}")
    print(f"Trying backup port: {backup_port}")
    try:
        ser = serial.Serial(
            port=backup_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )
        print(f"Successfully opened port: {backup_port}")
        port = backup_port
    except: 
        print(f"Unable to open port: {backup_port}")
        print("Try passing in port to command line with -i <port>")
        print("Here is a list of ports found:")
        list_serial_ports(True)
        sys.exit()

print(f"Opened port: {port} @{baudrate} baud (cntrl-c to quit)")
while 1:
    readMessage()
raw_log.close()


