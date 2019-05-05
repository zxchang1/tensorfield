import serial
import time

port = "/dev/ttyUSB0"
rate = 9600

s1 = serial.Serial(port, rate)
#s1.flushInput()

while True:
 #   print s1.inWaiting()
 #   time.sleep(1)

   # if s1.inWaiting()>0:
    arduinoMessage = s1.readline()
    print(arduinoMessage)

