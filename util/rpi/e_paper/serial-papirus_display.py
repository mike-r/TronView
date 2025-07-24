#!/usr/bin/env python
# Run on Pi-Zero with 2" ePaper display hat called "PaPiRus"

# /home/pi/1TM/serial-papirus.py

#  Version 2.3
print("serial-papirus.py Version 2.3")


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

print("Waiting 1 second for PaPiRus display and USB OTG to be ready")
sleep(1)

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

# Define serial link to Automationhat via USB OTG cable
try:
    automationhat_serial = serial.Serial(
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
    
aHat_ip_str = "waiting for aHat"
if automationhat_serial.is_open:
    wait_time = time.time()
    while True:
        if time.time() - wait_time > 60: break
        automationhat_bytes = automationhat_serial.read_until(b'\r\n', None)
        if len(automationhat_bytes) < 20:
            print("Received: ", len(automationhat_bytes), " bytes from Automationhat, retrying...")
            continue
        
        print("Received: ", len(automationhat_bytes), " bytes from Automationhat")
        automationhat_str = automationhat_bytes.decode()
        automationhat_str.strip()
        print(automationhat_str)
        print()
        if automationhat_bytes(1) == "5":
            aHat_ip_str = automationhat_str[3:18]
            break
        sleep(0.5)


#                         coll, row, height

text.AddText("N221TM",       25, 0, 39, Id="Line-1-Addr")
text.AddText(ePaper_ipaddr,  0, 39, 25, Id="Line-2-Addr")
text.AddText(aHat_ip_str,    0, 65, 25, Id="Line-3-Addr")

text.WriteAll()
time.sleep(5.0)




logfile = open("/home/zap/Speedster/serial-papirus.log", "r+")
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

# automationhat_serial.flushInput()                      # Flush input
# automationhat_bytes = automationhat_serial.read_until(b'\r\n', None)

time.sleep(1.0)
textPu.AddText("N221TM",    25,  0, 39, Id="Line-1")
textPu.AddText(f"{last_fuel} Fuel",  0, 37, 30, Id="Line-2")
textPu.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
time.sleep(1.0)

while True:
    print("engine_status: ", engine_status)
    engine_status_prev = engine_status
    automationhat_bytes = automationhat_serial.read_until(b'\r\n', None)
#    automationhat_bytes = automationhat_serial.readline()
    if len(automationhat_bytes) < 20:
        print("Received: ", len(automationhat_bytes), " bytes from Automationhat, retrying...")
        continue
    print(automationhat_bytes)
    automationhat_str = automationhat_bytes.decode()
    automationhat_str.strip()
    print(automationhat_str)
    print()
    if automationhat_bytes(1) != "4": continue

    smoke_str = automationhat_str[3:9]
    hobbs_str = automationhat_str[9:14]
    fuel_remain_str = automationhat_str[14:17]
    engine_status = automationhat_str[17:18]

    print('Smoke Level: ',   smoke_str, ' Gallons')
    print('Total Time: ',    hobbs_str, ' Hours')
    print('Fuel Remaining:', fuel_remain_str, ' Gallons', end='\r\n\n')
    print('engine_status:',  engine_status)

    try:
        smoke_gal = float(automationhat_bytes[4:8]) / 10
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
        fuel = float(automationhat_bytes[14:17]) / 10
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
        print()

    try:
        hobbs = float(automationhat_bytes[9:14]) / 10
        print('Hobbs: ', '{0:6.1f}'.format(hobbs), ' Hours')
        hobbs_change = abs(hobbs - last_hobbs)
        if hobbs_change > 0:
            hobbsF = "{:.1f}".format(hobbs)
            if hobbs < 1000:  hobbsF = hobbsF + " TT"
            print("hobbsF:", hobbsF)
            last_hobbs = hobbs
            update = True
    except ValueError:
        print()
        
    if engine_status == "s" and engine_status_prev == "r":      # Engine stopped and was running
        textPu.UpdateText("Line-1", hobbsF)
        print("Engine stopped, updating Line-1 with Hobbs")
        logfile.write(f"{last_hobbs},{last_fuel},{last_smoke}\n")
        logfile.close() 
        sys.exit(0)
    
    if update or loop_count > 50:
        #text.WriteAll()
        update = False
        loop_count = 0
    loop_count += 1
    print()
