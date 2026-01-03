#!/usr/bin/env python
# Run on Pi-Zero with 2" ePaper display hat called "PaPiRus"

# /home/pi/1TM/serial-papirus.py

print("serial-papirus_display.py Version 1.0")


# Power Raspberry Pi Zero via Micro-USB in USB port.
# Modify /boot/cmdline.txt (or /boot/firmware/cmdline.txt for 64 bit OS)
#    Add "modules-load=dw2" after "rootwait"
# Modify /boot/config.txt (or /boot/firmware/config.txt for 64 bit OS)
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

# Run this line and PaPiRus setup form /PaPiRus directory

#   Set screen size to 2.0

# To change screen size run:
#   sudo papirus-set [1.44 | 1.9 | 2.0 | 2.6 | 2.7 ]   -or-
#   sudo papirus-config

# To run at boot must have entry in /etc/rc.local
# sudo python3 /home/zap/Speedster/serial-papirus.py &

import socket
import os
import sys
import serial
import time
from papirus import PapirusTextPos
import RPi.GPIO as GPIO

gotIpAddress = False
tronview_ipaddr = "Wait for OTG"  # Default value if TronView not connected
ePaper_ipaddr = "No WiFi yet"
registration = "Speedy"  # Default registration number
last_registration = registration  # Last registration number
last_hobbs = 0.0  # Last Hobbs time
last_fuel = 0.0  # Last fuel level
last_smoke = 0.0  # Last smoke level
hobbs = 0.0  # Hobbs time
smoke_gal = 0.0  # Smoke level in gallons
fuel = 0.0  # Fuel level in gallons
engine_status = 's'  # Default engine status, 's' for stopped
engine_status_prev = 's'  # Previous engine status for comparison
tvName1 = "TronView1"
tvName2 = "TronView2"
tvName3 = "TronView3"
tvValue1 = 0.0
tvValue2 = 0.0
tvValue3 = 0.0
sw2_pressed = False

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

def buttonEventHandlerSw1(channel):
    global gotIpAddress
    print("SW1 (", channel, ") pressed - Getting and Displaying IP addresses")
    if not gotIpAddress: getIpAddress()
    displayAddreses()
    time.sleep(5) # Display for 5 seconds
    displayRegFuelSmoke()
GPIO.add_event_detect(SW1, GPIO.FALLING, buttonEventHandlerSw1, 100)

def buttonEventHandlerSw2(channel):
    global sw2_pressed
    print("Button SW2 (", channel, ") pressed - Simulate Engine Shutdown")
    sw2_pressed = True
GPIO.add_event_detect(SW2, GPIO.FALLING, buttonEventHandlerSw2, 100)

def buttonEventHandlerSw3(channel):
    print("Button SW3 (", channel, ") pressed - Do Nothing")
GPIO.add_event_detect(SW3, GPIO.FALLING, buttonEventHandlerSw3, 100)

def buttonEventHandlerSw4(channel):
    print("Button SW4 (", channel, ") pressed - Do Nothing")
GPIO.add_event_detect(SW4, GPIO.FALLING, buttonEventHandlerSw4, 100)

def buttonEventHandlerSw5(channel):
    print("Button SW5 (", channel, ") pressed - Do Nothing")
GPIO.add_event_detect(SW5, GPIO.FALLING, buttonEventHandlerSw5, 100)

def displayAddreses():
    global gotIpAddress
    text.Clear()
    time.sleep(1.0)
    print("in displayAddress, gotIpAddress: ",gotIpAddress)
    text.AddText("PaPiRus Display:", 35,  5, 15, Id="Line-1-Addr")
    text.AddText(ePaper_ipaddr,       0, 20, 25, Id="Line-2-Addr")
    text.AddText("TronView:",        60, 50, 15, Id="Line-3-Addr")
    text.AddText(tronview_ipaddr,     0, 65, 25, Id="Line-4-Addr")

    text.WriteAll()
    time.sleep(1.0)

def displayRegFuelSmoke():
    text.Clear()
    time.sleep(1.0)
    text.AddText(registration,         20,  0, 39, Id="Line-1")
    text.AddText(f"{last_fuel} Fuel",   0, 37, 30, Id="Line-2")
    text.AddText(f"{last_smoke} Smoke", 0, 66, 30, Id="Line-3")
    text.WriteAll()
    time.sleep(1.0)

def getIpAddress():
    global ePaper_ipaddr
    global gotIpAddress
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
        gotIpAddress = True
        return()
    except:
        print("Error: Unable to get IP address")
        gotIpAddress = False
        return()


#  2" PaPiRus Display size is:  200 X 96 pixels

try:
    #textPu = PapirusTextPos(True)       # Initialize with update=True for partial updates
    text = PapirusTextPos(False)        # Initialize with update=False for full screen updates
except:
    print("Error: Unable to initialize PapirusTextPos.  Display not attached?\r\n   Program will exit")
    exit()

getIpAddress()      # Get the IP address of the PaPiRus Pi
if gotIpAddress:
    displayAddreses()   # Display PaPiRus Pi IP addresses on PaPiRus
else:
    ePaper_ipaddr = "No WiFi!!"
    displayAddreses()   # Display "No WiFi"

print("Waiting 10 seconds for PaPiRus display and USB OTG to be ready")
time.sleep(10)

# Define serial link to TronView via USB OTG cable
try:
    tronview_serial = serial.Serial(
        port='/dev/ttyGS0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None  # Set a timeout for reading
    )
except Exception as e:
    print("Error: Unable to open serial port")
    print("Error with serial port:", e)
    text.Clear()
    text.UpdateText("Line-1", "Serial err")
    text.UpdateText("Line-2", "Is USB cable")
    text.UpdateText("Line-3", "in OTG port?")
    time.sleep(1)
    text.WriteAll()
    time.sleep(5.0)
    exit()

if tronview_serial.is_open:
    rmt_rapi_bytes = ePaper_ipaddr.encode()
    rmt_rapi_bytes += b'\r\n'
    tronview_serial.reset_output_buffer() # Clear any existing data in the serial buffer
    print("Sending rmt_rapi_bytes to TV: ", rmt_rapi_bytes)
    tronview_serial.write(rmt_rapi_bytes)
    wait_time = time.time()
    while True:
        tronview_serial.reset_input_buffer() # Clear any existing data in the serial buffer
        time.sleep(.001)
        if time.time() - wait_time > 10: break
        tronview_bytes = tronview_serial.read_until(b'\r\n', None)
        if len(tronview_bytes) < 10:
            print("Received: ", len(tronview_bytes), " bytes from TronView, waiting and retry...")
            tronview_bytes = tronview_serial.read_until(b'\r\n', None)
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
            print(tronview_str)
            break
        time.sleep(0.75)
else:                       # no link to TronView RaPi so wait until there is...
    pass                    # ToDo Retry code


text.UpdateText("Line-4-Addr", tronview_ipaddr)
print("tronview_ipaddr:", tronview_ipaddr)
time.sleep(1.0)
text.WriteAll()
print("Displayed updated TronView IP Address on PaPiRus")

logfile = open("/home/pi/1TM/serial-papirus.log", "r+")
data=logfile.readlines()[-1]
logfile.close()
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
    except Exception as e:
        print("Error with logfile:", e)

update     = False
loop_count = 0

time.sleep(5.0)
text.Clear()

text.AddText(registration,         30,  0, 39, Id="Line-1")
text.AddText(f"{last_fuel} Fuel",  20, 37, 30, Id="Line-2")
text.AddText(f"{last_smoke} Smoke",20, 66, 30, Id="Line-3")
time.sleep(1.0)
text.WriteAll()

while True:
    tronview_serial.reset_input_buffer()
    tronview_bytes = tronview_serial.read_until(b'\r\n', None)
    if len(tronview_bytes) < 10:
        print("Received: ", len(tronview_bytes), " bytes from TronView, retrying...")
        continue
    
    tronview_str = tronview_bytes.decode()
    
    if tronview_str[0] != '!':
        print("Error: Invalid message format from TronView, expected '!' at start")
        print("Received: ", tronview_str)
        print("Reading again...")
        tronview_bytes = tronview_serial.read_until(b'\r\n', None)
        tronview_str = tronview_bytes.decode()
        if tronview_str[0] != '!': continue  # Skip to next iteration if still invalid format

    if tronview_str[1] == "5":
        print("Recieved TV IP Address")
        try:
            tronview_ipaddr = tronview_str[3:18]
        except Exception as e:
            print("Error parsing TronView IP address:", e)
            print("tronview_str: ", tronview_str)
            continue  # Skip to the next iteration if parsing fails
        tronview_str = "Received TronView IP: " + tronview_ipaddr
        print("Sending PaPiRus IP Address back to TronView:", tronview_str)
        papirus_bytes = ePaper_ipaddr.encode()
        papirus_bytes += b'\r\n'
        tronview_serial.reset_output_buffer() # Clear any existing data in the serial buffer
        tronview_serial.write(papirus_bytes)
        displayAddreses()
        time.sleep(5)
        displayRegFuelSmoke()
        continue

    if tronview_str[1] == "4": 
        try:
            tronview_str1 = tronview_str.strip()
            tronview_str2 = tronview_str1.split(",")
            #for i in range(len(tronview_str2)):
            #    print("Value : ", tronview_str2[i], " at index:", i)
            
            engine_status_prev = engine_status  # Save previous engine status
            registration = tronview_str2[1]
            tvName1 = tronview_str2[2]  # Fuel remaining
            tvValue1 = tronview_str2[3]  # Fuel value
            tvName2 = tronview_str2[4]  # Hobbs time
            tvValue2 = tronview_str2[5]  # Hobbs value
            tvName3 = tronview_str2[6]  # Smoke level
            tvValue3 = tronview_str2[7]  # Smoke value
            engine_status = tronview_str2[8]  # Engine status
        except Exception as e:
            print("Error parsing TronView data:", e)
            print("tronview_str: ", tronview_str)
            continue  # Skip to the next iteration if parsing fails

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
            smoke_change = abs(smoke_gal - last_smoke)
            if smoke_change >= 0.1:  # Update if smoke changes by at least 0.1 gallons
                gallonsF = "{:.1f}".format(smoke_gal)
                gallonsF = gallonsF + "  Smoke"
                if smoke_gal < 0.25: 
                    print("Smoke level is low, setting to --EMPTY--")
                    gallonsF = "--EMPTY--"
                #textPu.UpdateText("Line-3", gallonsF)
                text.UpdateText("Line-3", gallonsF)
                print("Updated Line-3 withGallonsF:", gallonsF)
                last_smoke = smoke_gal
                update = True
        except Exception as e:
            print("Error updating Line-3 with gallonsF:", e)
            print("Error updating Line-3 with Smoke Level:", e)

        try:
            fuel = float(tvValue1)
            fuel_change = abs(fuel - last_fuel)
            if fuel_change >= 0.1:  # Update if fuel changes by at least 0.1 gallons
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
            hobbs_change = abs(hobbs - last_hobbs)
            if hobbs_change >= 0.1:
                print("New hobbsF:", hobbsF)
                last_hobbs = hobbs
        except Exception as e:
            print("Error updating Hobbs:", e)
        #print("loop_count:", loop_count)

    if sw2_pressed: engine_status = "s"  # Debug to test engine status change
    if engine_status == "s" and engine_status_prev == "r":      # Engine stopped and was running
        if hobbs < 10000:  hobbsF = hobbsF + " TT"
        text.Clear()
        time.sleep(1)
        #text.UpdateText("Line-1", hobbsF)
        text.AddText(hobbsF,                0,  0, 37, Id="Line-1")
        text.AddText(f"{last_fuel} Fuel",  20, 37, 30, Id="Line-2")
        text.AddText(f"{last_smoke} Smoke",20, 66, 30, Id="Line-3")

        print("Engine stopped, updating Line-1 with Hobbs")
        logfile = open("/home/pi/1TM/serial-papirus.log", "r+")
        logfile.write(f"{registration},{last_hobbs},{last_fuel},{last_smoke}\n")
        logfile.close()
        text.WriteAll()
        print("PaPiRus display updated with Hobbs time")
        GPIO.cleanup()
        time.sleep(5.0)
        sys.exit(0)
    
    if update or loop_count > 500:
        print("Loop count:", loop_count, "Update:", update)
        update = False
        loop_count = 0
        print('Updating PaPiRus display with new values')
        print(tvName1, '{0:3.1f}' .format(fuel), 'Gallons')
        print(tvName2, '{0:6.1f}'.format(hobbs), ' Hours')
        print('Smoke Level:', '{0:3.1f}' .format(smoke_gal), 'Gallons')
        print()

        text.WriteAll()
        time.sleep(1.0)
    loop_count += 1

    

