#manage air/oil cylinder pressure maintenance under normal operation and manage oil filling routine.
#to include oil volume readings at later date

import smbus
import time 
import Adafruit_ADS1x15

gpb_state=[0,0,0,0,0,0,0,0] #this will map to MCP23017 pins GPB0-GPB7

pin_pump_motor=5 #change as appropriate. 5 means GPB5 or GPA5
pin_pump_direction=6
pin_pump_stop_valve=7

adcpin_airpressure=0

DEVICE = 0x20 #Device address
IODIRB = 0x01 #Pin direction register
OLATB = 0x15 #Register for outputs
GPIOB = 0x13 #Register for inputs
#IODIRA = 0x00 <--if we want to use the A pins rather than B pins as above
#OLATA = 0x14
#GPIOA = 0x12

#while True:
#    bus = smbus.SMBus(1)
#    bus.write_byte_data(DEVICE,IODIRB,0x00)
#    bus.write_byte_data(DEVICE,OLATB,0b01000000)
#    time.sleep(0.1)
#    bus.write_byte_data(DEVICE,OLATB,0b00100000)
#    time.sleep(0.1)
#    bus.write_byte_data(DEVICE,OLATB,0b10000000)
#    time.sleep(0.1)
#    





adc_aircylinder = Adafruit_ADS1x15.ADS1015(address=0x48, busnum=1)


GAIN = 1
values = [0]*4
#while True:
#
#    for i in range(4):
#        values[i] = adc_aircylinder.read_adc(i,gain=GAIN)
#        print(values)
#        time.sleep(0.5)
#

def activate_airpump(gpb_state,direction):
    print("air pump set to "+direction)
    return;

def readpressure():
    adcRaw_pressure=adc_aircylinder.read_adc(adcpin_airpressure,gain=GAIN)
    print(adcRaw_pressure)
    pressure = 200  
    return pressure



#function to maintain air pressure in cylinder at given pressure

def pressurize_cylinder(op_pressure):
    "This maintains the cylinder at the given pressure in kPa"

    print(op_pressure)
    return;

if __name__ == "__main__":
    
    print(readpressure())
    exit()
    pressurize_cylinder(op_pressure = 275)

    bus = smbus.SMBus(1)

    bus.write_byte_data(DEVICE,IODIRB,0x00)

    bus.write_byte_data(DEVICE,OLATB,0b11100000)

    print 'GPB state ' + str(bin(bus.read_byte_data(DEVICE,GPIOB))) #or GPIOA

