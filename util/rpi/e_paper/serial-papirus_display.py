#!/usr/bin/env python
# Run on Pi-Zero with 2" ePaper display hat called "PaPiRus"

# /home/pi/1TM/serial-papirus.py

#  Version 0.4 testing
print("serial-papirus_display.py Version 0.4.Testing")


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
import RPi.GPIO as GPIO

tronview_comms_ok = False
tronview_ipaddr = "Waiting 60s"  # Default value if TronView not connected
registration = "Speedy"  # Default registration number
last_registration = registration  # Last registration number
engine_status = 's'  # Default engine status, 's' for stopped
engine_status_prev = 's'  # Previous engine status for comparison
tvName1 = "TronView1"
tvName2 = "TronView2"
tvName3 = "TronView3"
tvValue1 = 0
tvValue2 = 0
tvValue3 = 0

GPIO.setmode(GPIO.BCM)
# Setup GPIO pins for PaPiRus buttons
SW1 = 21
SW2 = 16
SW3 = 20
SW4 = 19
SW5 = 26
GPIO.setup(SW1, GPIO.IN)
GPIO.setup(SW2, GPIO.IN)
GPIO.setup(SW3, GPIO.IN)
GPIO.setup(SW4, GPIO.IN)
GPIO.setup(SW5, GPIO.IN)

def displayAddreses():
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

    text.AddText(registration,         20,  0, 39, Id="Line-1")
    text.AddText(f"{last_fuel} Fuel",   0, 37, 30, Id="Line-2")
    text.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
    time.sleep(1.0)
    text.WriteAll()

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
tronview_serial.flushInput()  # Clear any existing data in the serial buffer
if tronview_serial.is_open:
    wait_time = time.time()
    while True:
        if GPIO.input(SW1) == False:
            print("SW1 pressed but do nothing")
            #time.sleep(0.1) # Debounce delay
        if time.time() - wait_time > 6: break
        tronview_bytes = tronview_serial.read_until(b'\r\n', None)
        if len(tronview_bytes) < 10:
            print("Received: ", len(tronview_bytes), " bytes from TronView, waiting and retry...")
            sleep(0.5)
            continue

        print("Received: ", len(tronview_bytes), " bytes from TronView")
        tronview_str = tronview_bytes.decode()
        tronview_str.strip()
        tronview_str.split(",")
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
        registration = dataList[0]
        last_registration = registration
        last_hobbs = float(dataList[1])
        last_fuel = float(dataList[2])
        last_smoke = float(dataList[3])

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


time.sleep(10.0)
text.Clear()
#time.sleep(1.0)

text.AddText(registration,         20,  0, 39, Id="Line-1")
text.AddText(f"{last_fuel} Fuel",   0, 37, 30, Id="Line-2")
text.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
time.sleep(1.0)
text.WriteAll()

tronview_serial.flushInput()  # Clear any existing data in the serial buffer
tronview_serial.flushOutput() # Clear any existing data in the serial buffer

while True:
    tronview_bytes = tronview_serial.read_until(b'\r\n', None)
    if GPIO.input(SW1) == False:
        print("SW1 pressed - Displaying IP addresses")
        displayAddreses()
        time.sleep(0.1) # Debounce delay
    if len(tronview_bytes) < 10:
        print("Received: ", len(tronview_bytes), " bytes from TronView, retrying...")
        continue
    
    tronview_str = tronview_bytes.decode()

    if tronview_str[1] == "5" and not tronview_comms_ok:
        tronview_ipaddr = tronview_str[3:18]
        tronview_str = "Received TronView IP: " + tronview_ipaddr
        print("Sending PaPiRus IP Address back to TronView:", tronview_str)
        papirus_bytes = ePaper_ipaddr.encode()
        papirus_bytes += b'\r\n'
        tronview_serial.write(papirus_bytes)
        displayAddreses()
        tronview_serial.flushInput()  # Clear any existing data in the serial buffer
        continue

    if tronview_str[1] == "4":
        tronview_str1 = tronview_str.strip()
        tronview_str2 = tronview_str1.split(",")
        for i in range(len(tronview_str2)):
            print("Value : ", tronview_str2[i], " at index:", i)
            
        engine_status_prev = engine_status  # Save previous engine status
        registration = tronview_str2[1]
        tvName1 = tronview_str2[2]  # Fuel remaining
        tvValue1 = tronview_str2[3]  # Fuel value
        tvName2 = tronview_str2[4]  # Hobbs time
        tvValue2 = tronview_str2[5]  # Hobbs value
        tvName3 = tronview_str2[6]  # Smoke level
        tvValue3 = tronview_str2[7]  # Smoke value
        engine_status = tronview_str2[8]  # Engine status
        print()
        print('Registration: ', registration)
        print('tv1: ', tvName1, tvValue1)
        print('tv2: ', tvName2, tvValue2)
        print('tv3: ', tvName3, tvValue3)
        print('engine_status:', engine_status)
        print()

        try:
            if registration != last_registration:
                last_registration = registration
                #textPu.UpdateText("Line-1", registration)
                text.UpdateText("Line-1", registration)
                print("Updated Line-1 with Registration:", registration)
                update = True
        except Exception as e:
            print("Error updating Line-1 with Registration:", e)
        
        try:
            smoke_gal = float(tvValue3)
            print('Smoke Level:', '{0:3.1f}' .format(smoke_gal), 'Gallons')
            smoke_change = abs(smoke_gal - last_smoke)
            if smoke_change > 0.109:  # Update if smoke changes by more than 0.1 gallons
                gallonsF = "{:.1f}".format(smoke_gal)
                gallonsF = gallonsF + "  Smoke"
                if smoke_gal < 0.25: gallonsF = "--EMPTY--"
                #textPu.UpdateText("Line-3", gallonsF)
                text.UpdateText("Line-3", gallonsF)
                print("Updated Line-3 withGallonsF:", gallonsF)
                last_smoke = smoke_gal
                update = True
        except Exception as e:
            print("Error updating Line-3 with Smoke Level:", e)

        try:
            fuel = float(tvValue1)
            print (tvName1, '{0:3.1f}' .format(fuel), 'Gallons')
            fuel_change = abs(fuel - last_fuel)
            if fuel_change > 0.09:  # Update if fuel changes by more than 0.09 gallons
                fuelF = "{:.1f}".format(fuel)
                fuelF = fuelF + " Fuel"
                #textPu.UpdateText("Line-2", fuelF)
                text.UpdateText("Line-2", fuelF)
                print("Updated Line-2 with fuelF:", fuelF)
                last_fuel = fuel
                update = True
        except Exception as e:
            print("Error updating Line-2 with fuelF:", e)

        try:
            hobbs = float(tvValue2)
            hobbsF = "{:.1f}".format(hobbs)
            print(tvName2, '{0:6.1f}'.format(hobbs), ' Hours')
            print('hobbsF: ', hobbsF, ' Hours')
            hobbs_change = abs(hobbs - last_hobbs)
            if hobbs_change > 0:
                if hobbs < 1000:  hobbsF = hobbsF + " TT"
                print("hobbsF:", hobbsF)
                last_hobbs = hobbs
        except Exception as e:
            print("Error updating Line-1 with Hobbs:", e)

    #if fuel < 15.5: engine_status = "s"  # Debug to test engine status change
    if engine_status == "s" and engine_status_prev == "r":      # Engine stopped and was running
        text.UpdateText("Line-1", hobbsF)
        print("Engine stopped, updating Line-1 with Hobbs")
        logfile.write(f"{registration},{last_hobbs},{last_fuel},{last_smoke}\n")
        logfile.close()
        time.sleep(1.0)
        text.WriteAll()
        print("PaPiRus display updated with Hobbs time")
        time.sleep(10.0)
        sys.exit(0)
    
    if update or loop_count > 50:
        update = False
        loop_count = 0
        time.sleep(1.0)
        print('Updating PaPiRus display with new values')
        text.WriteAll()
        time.sleep(1.0)
        tronview_serial.flushInput()  # Clear any existing data in the serial buffer
    loop_count += 1
    print()
    

