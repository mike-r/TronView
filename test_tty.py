# test_tty.py
import serial
from time import sleep
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

while True:
    automationhat_bytes = automationhat_serial.read_until(b'\r\n', None)
#    automationhat_bytes = automationhat_serial.readline()
    print(automationhat_bytes)
    automationhat_str = automationhat_bytes.decode()
    automationhat_str.strip()
    print(automationhat_str)
    print()
    sleep(2)