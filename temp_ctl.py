import serial
import time

# port = "/dev/ttyUSB0"
port3 = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0"
port2 = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-port0"
rate = 9600

s1 = serial.Serial(port3, rate)
#s1.flushInput()

s2 = serial.Serial(port2, rate)
#s2.flushInput()

while True:
 #   print s1.inWaiting()
 #   time.sleep(1)

    if s1.inWaiting()>0:
        arduinoMessage = s1.readline()
        print(arduinoMessage)
        #time.sleep(1)

    if s2.inWaiting()>0:
        arduinoMessage = s2.readline()
        print(arduinoMessage)
        #time.sleep(1)

