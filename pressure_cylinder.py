#manage air/oil cylinder pressure maintenance under normal operation and manage oil filling routine.
#to include oil volume readings at later date
#we are using ADS1015 as ADC over i2c bus
#we are using MCP23017 as GPIO expander over i2c bus

import smbus
import time 
import Adafruit_ADS1x15

def operate_on_loop(bus, pins, DEVICE, OLATB):
    maintain = True
    while True:
        if maintain:
            maintain_pressure(bus, pins, DEVICE, GPIOB, OLATB, 0.7)
        else:
            vacuum()

def vacuum():
    pass

def maintain_pressure(bus, pins, DEVICE, GPIOB, OLATB, threshold): 
    pins = read_pins(bus, DEVICE, GPIOB)
    pressure = read_pressure(bus)
    if pressure < threshold:
        pump_operate(bus, pins, DEVICE, OLATB, 'blow')
    else:
        pump_operate(bus, pins, DEVICE, OLATB, 'off')
    set_pins(bus, pins, DEVICE, OLATB)

def pump_operate(bus, pins, DEVICE, OLATB, direction):

    if direction == 'blow':
        pins[pin_pump_motor]=1
        pins[pin_pump_direction]=0
        pins[pin_pump_stop_valve]=1
    elif direction == 'off':
        pins[pin_pump_motor]=0
        pins[pin_pump_direction]=0
        pins[pin_pump_stop_valve]=0
    elif direction == 'vacuum':
        pins[pin_pump_motor]=1
        pins[pin_pump_direction]=1
        pins[pin_pump_stop_valve]=1

    set_pins(bus, pins, DEVICE, OLATB)
    print("air pump set to " + direction)

def read_pressure(bus):
    adcRaw_pressure=adc_aircylinder.read_adc(adcpin_airpressure, gain=GAIN)
    print(adcRaw_pressure)
    pressure = 280  
    return pressure

def uint8_to_binary_string(uint8):
    return bin(uint8)[2:].zfill(8)

def read_pins(bus, DEVICE, GPIOB):
    print(DEVICE, GPIOB)
    int_pins = bus.read_byte_data(DEVICE, GPIOB)
    print(int_pins)
    str_pins = uint8_to_binary_string(int_pins)
    print(str_pins)
    pins = list(map(int, reversed(str_pins)))
    print pins
    return pins

def set_pins(bus, pins, DEVICE, OLATB):
    print(pins)
    print(type(pins))
    intBinOut = pins[::-1]
    print(intBinOut)
    pins_set = int(''.join(map(str,intBinOut)),base=2)
    bus.write_byte_data(DEVICE, OLATB, pins_set)


if __name__ == "__main__":
    
    pin_pump_motor = 5 #change as appropriate. 5 means GPB5 or GPA5
    pin_pump_direction = 6
    pin_pump_stop_valve = 7
    
    adcpin_airpressure = 0
    adcpin_cylinderpos = 1
    
    DEVICE = 0x20 #Device address
    IODIRB = 0x01 #Pin direction register
    OLATB = 0x15 #Register for outputs
    GPIOB = 0x13 #Register for inputs
    #IODIRA = 0x00 <--if we want to use the A pins rather than B pins as above
    #OLATA = 0x14
    #GPIOA = 0x12
    
    adc_aircylinder = Adafruit_ADS1x15.ADS1015(address=0x48, busnum=1)
    
    GAIN = 1
    values = [0]*4


    bus = smbus.SMBus(1)
    bus.write_byte_data(DEVICE, IODIRB, 0x00) #sets all to outputs
    pins = read_pins(bus, DEVICE, GPIOB) #this maps to MCP23017 pins GPB0-GPB7

    maintain_pressure(bus, pins, DEVICE, GPIOB, OLATB, 275) #threshold in kPa

    set_pins(bus, [0,0,0,0,0,1,0,1], DEVICE, OLATB)
    time.sleep(0.5)
    print(read_pins(bus, DEVICE, GPIOB)) 
    






