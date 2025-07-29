#!/usr/bin/env python
# Run on Pi-Zero with 2" ePaper display hat called "PaPiRus"

# /home/pi/1TM/serial-papirus.py

#  Version 0.3i testing
print("serial-papirus_display.py Version 0.3i.Testing")


# Power Raspberry Pi Zero via Micro-USB in USB port.
# Modify /boot/cmdline.txt
#    Add "modules-load=dw2" after "rootwait"
# Modify /boot/confix.txt
#    Add "dtoverlay=dwc2" at the end of the file
# Run Raspi-Config to enable serial ports and disable login console over serial
# The above will create a Serial instance on the USB cable via the "/dev/ttyGS0" driver
# The host USB will see a serial-USB device (/dev/ttyACM0) on a Raspberry Pi.
# Retrieve host IP address and gateway address and write to PaPiRus display

#   To setup micro-USB:
#    sudo systemctl enable getty@ttyGS0.service
#    sudo systemctl is-active getty@ttyGS0.service

# Program will read from OTG serial port connected to Automationhat Pi
# Format of data stream is:
# !41+ssssGhhhhhfffr
# "+ssssG"   is the amount of smoke oil remaining in tenths of gallons
# "yyyyy"    is the Total Time (Hobbs Time) in tenths of hours from EMS
# "fff"      is the Total Fuel Remaining in tents of gallons
# "r"        is engine runnuing status, 'r' for running, 's' for stopped
# Example: !41+0005G00001f100r
#          !51aaa.bbb.ccc.ddd   IP Address of Automationhat Pi

# Run this line and PaPiRus will be setup and installed
#   curl -sSL https://pisupp.ly/papiruscode | sudo bash
#   Select "Python3"
#   Set screen size to 2.0

# To change screen size run:
#   sudo papirus-set [1.44 | 1.9 | 2.0 | 2.6 | 2.7 ]   -or-
#   sudo papirus-config

# To run at boot must have entry in /etc/rc.local
# sudo python3 /home/zap/Speedster/serial-papirus.py &

import socket
import os
import sys
from time import sleep
import serial
import time
from papirus import PapirusTextPos

tronview_comms_ok = False
tronview_ipaddr = "Waiting 60s"  # Default value if TronView not connected


print("Waiting 10 seconds for PaPiRus display and USB OTG to be ready")
sleep(10)

#  2" PaPiRus Display size is:  200 X 96 pixels

try:
    textPu = PapirusTextPos(True)       # Initialize with update=True for partial updates
    text = PapirusTextPos(False)        # Initialize with update=False for full screen updates
except:
    print("Error: Unable to initialize PapirusTextPos.  Display not attached?\r\n   Program will exit")
    exit()

try:
    text.Clear()
    time.sleep(1.0)
    gw = os.popen("ip -4 route show default").read().split()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((gw[2], 0))
    ePaper_ipaddr = s.getsockname()[0]
    gateway = gw[2]
    host = socket.gethostname()
    print ("IP:", ePaper_ipaddr, " GW:", gateway, " Host:", host)
except:
    print("Error: Unable to get IP address")
    
text.AddText("PaPiRus Display:", 35,  5, 15, Id="Line-1-Addr")
text.AddText(ePaper_ipaddr,       0, 20, 25, Id="Line-2-Addr")
text.AddText("TronView:",        60, 50, 15, Id="Line-3-Addr")
text.AddText(tronview_ipaddr,     0, 65, 25, Id="Line-4-Addr")
time.sleep(1.0)
text.WriteAll()

# Define serial link to TronView via USB OTG cable
try:
    tronview_serial = serial.Serial(
        port='/dev/ttyGS0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=5
    )
except serial.SerialException:
    print("Error: Unable to open serial port")
    exit()

if tronview_serial.is_open:
    wait_time = time.time()
    while True:
        if time.time() - wait_time > 60: break
        tronview_bytes = tronview_serial.read_until(b'\r\n', None)
        if len(tronview_bytes) < 10:
            print("Received: ", len(tronview_bytes), " bytes from TronView, waiting and retry...")
            sleep(0.5)
            continue

        print("Received: ", len(tronview_bytes), " bytes from TronView")
        tronview_str = tronview_bytes.decode()
        tronview_str.strip()
        print("String from TronView: ", tronview_str)
        print()
        if tronview_str[1] == "5":
            tronview_ipaddr = tronview_str[3:18]
            tronview_str = "Received TronView IP: " + tronview_ipaddr
            print("Sending back to TronView:", tronview_str)
            papirus_bytes = ePaper_ipaddr.encode()
            papirus_bytes += b'\r\n'
            tronview_serial.write(papirus_bytes)
            tronview_comms_ok = True
            break
        sleep(0.5)

#                           coll, row, height
#text.AddText("N873PW",       25, 0, 39, Id="Line-1-Addr")
#text.AddText(ePaper_ipaddr,   0, 39, 25, Id="Line-2-Addr")
#text.AddText(tronview_ipaddr, 0, 65, 25, Id="Line-3-Addr")

text.UpdateText("Line-4-Addr", tronview_ipaddr)
time.sleep(1.0)
text.WriteAll()
print("Displayed updated TronView IP Address on PaPiRus")

logfile = open("/home/pi/1TM/serial-papirus.log", "r+")
data=logfile.readlines()[-1]
dataList = data.split(",")
print("data:", data)
print ("dataList:", dataList)

if len(dataList) > 0:
    #logfile.close()
    try:
        last_hobbs = float(dataList[0])
        last_fuel = float(dataList[1])
        last_smoke = float(dataList[2])

        print("Last Fuel:", last_fuel, "Last Smoke:", last_smoke, "Last Hobbs:", last_hobbs)
    except ValueError:
        print("Error: Unable to parse previous values from log file")
        last_fuel =  100.1
        last_smoke = 100.1
        last_hobbs = 100.1
else:
    print("No previous values found in log file, using defaults")
    last_fuel =  100.1
    last_smoke = 100.1
    last_hobbs = 100.1

hobbs      = 200.1
smoke_gal  = 200.1
fuel       = 200.1
update     = False
loop_count = 0
engine_status = 's'       # 'r' for running, 's' for stopped
engine_status_prev = 's'  # Previous engine status for comparison

time.sleep(10.0)
text.Clear()
time.sleep(1.0)

textPu.AddText("N873PW",             25,  0, 39, Id="Line-1")
textPu.AddText(f"{last_fuel} Fuel",   0, 37, 30, Id="Line-2")
textPu.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
time.sleep(1.0)

tronview_serial.flushInput()  # Clear any existing data in the serial buffer
tronview_serial.flushOutput() # Clear any existing data in the serial buffer

while True:
    print("engine_status: ", engine_status)
    engine_status_prev = engine_status
    tronview_bytes = tronview_serial.read_until(b'\r\n', None)
    if len(tronview_bytes) < 10:
        print("Received: ", len(tronview_bytes), " bytes from TronView, retrying...")
        continue
    print("TronView Bytes: ", tronview_bytes)
    tronview_str = tronview_bytes.decode()
    tronview_str.strip()

    if tronview_str[1] == "5" and not tronview_comms_ok:
        tronview_ipaddr = tronview_str[3:18]
        tronview_str = "Received TronView IP: " + tronview_ipaddr
        print("Sending PaPiRus IP Address back to TronView:", tronview_str)
        papirus_bytes = ePaper_ipaddr.encode()
        papirus_bytes += b'\r\n'
        tronview_serial.write(papirus_bytes)
        tronview_comms_ok = True
        text.Clear()
        time.sleep(1.0)
        text.AddText("PaPiRus Display:", 35,  5, 15, Id="Line-1-Addr")
        text.AddText(ePaper_ipaddr,       0, 20, 25, Id="Line-2-Addr")
        text.AddText("TronView:",        60, 50, 15, Id="Line-3-Addr")
        text.AddText(tronview_ipaddr,     0, 65, 25, Id="Line-4-Addr")

        text.WriteAll()
        time.sleep(15.0)
        text.Clear()
        time.sleep(1.0)
        
        textPu.AddText("N873PW",             25,  0, 39, Id="Line-1")
        textPu.AddText(f"{last_fuel} Fuel",   0, 37, 30, Id="Line-2")
        textPu.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
        continue

    if tronview_str[1] == "4":
        smoke_str = tronview_str[3:9]
        hobbs_str = tronview_str[9:14]
        fuel_remain_str = tronview_str[14:17]
        engine_status = tronview_str[17:18]

        print('Smoke Level: ',   smoke_str, ' Gallons')
        print('Total Time: ',    hobbs_str, ' Hours')
        print('Fuel Remaining:', fuel_remain_str, ' Gallons', end='\r\n\n')
        print('engine_status:',  engine_status)

        try:
            smoke_gal = float(tronview_bytes[4:8]) / 10
            print('Smoke Level:', '{0:3.1f}' .format(smoke_gal), 'Gallons')
            smoke_change = abs(smoke_gal - last_smoke)
            if smoke_change > 0.2:
                gallonsF = "{:.1f}".format(smoke_gal)
                gallonsF = gallonsF + "  Smoke"
                if smoke_gal < 0.25: gallonsF = "--EMPTY--"
                textPu.UpdateText("Line-3", gallonsF)
                print("GallonsF:", gallonsF)
                last_smoke = smoke_gal
                update = True
        except ValueError:
            print()

        try:
            fuel = float(tronview_bytes[14:17]) / 10
            print ('Fuel Level:', '{0:3.1f}' .format(fuel), 'Gallons')
            fuel_change = abs(fuel - last_fuel)
            if fuel_change > 0.5:  # Update if fuel changes by more than 0.5 gallons
                fuelF = "{:.1f}".format(fuel)
                fuelF = fuelF + " Fuel"
                textPu.UpdateText("Line-2", fuelF)
                print("fuelF:", fuelF)
                last_fuel = fuel
                update = True
        except ValueError:
            print("Error parsing fuel level from TronView data")
            
        try:
            hobbs = float(tronview_bytes[9:14]) / 10
            print('Hobbs: ', '{0:6.1f}'.format(hobbs), ' Hours')
            hobbs_change = abs(hobbs - last_hobbs)
            if hobbs_change > 0:
                hobbsF = "{:.1f}".format(hobbs)
                if hobbs < 1000:  hobbsF = hobbsF + " TT"
                print("hobbsF:", hobbsF)
                last_hobbs = hobbs
                update = True
        except ValueError:
            print("Error parsing Hobbs time from TronView data")
        
    if engine_status == "s" and engine_status_prev == "r":      # Engine stopped and was running
        textPu.UpdateText("Line-1", hobbsF)
        print("Engine stopped, updating Line-1 with Hobbs")
        logfile.write(f"{last_hobbs},{last_fuel},{last_smoke}\n")
        logfile.close() 
        sys.exit(0)
    
    if update or loop_count > 50:
        update = False
        loop_count = 0
    loop_count += 1
    print()
