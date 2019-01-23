#manage air/oil cylinder pressure maintenance under normal operation and manage oil filling routine.

#to include oil volume readings at later date
#we are using ADS1015 as ADC over i2c bus
#we are using MCP23017 as GPIO expander over i2c bus

import smbus
import time 
import Adafruit_ADS1x15
import VL53L0X

def dev2pin(dev):
    dict = {'pump_motor': 4, 'valve_blow': 5, 'valve_suck': 6, 'valve_stop': 7, \
            'toggle': 0, 'button_blow': 0, 'button_suck': 5, 'LED': 3}
    return dict[dev]


def dev2block(dev):
    dict = {'pump_motor': 0x15, 'valve_blow': 0x15, 'valve_suck': 0x15, 'valve_stop': 0x15,\
            'toggle': 0x13, 'button_blow': 0x12, 'button_suck': 0x12, 'LED': 0x14}
    return dict[dev]        

def maintain_pressure(bus, pins, DEVICE, GPIOB, OLATB, threshold): 
    pins = read_pins(bus, DEVICE, GPIOB)
    pressure = read_pressure(bus)
    if pressure < threshold:
        pump_operate(bus, pins, DEVICE, OLATB, 'blow')
    else:
        pump_operate(bus, pins, DEVICE, OLATB, 'off')
    set_pins(bus, pins, DEVICE, OLATB)

def manual_mode(bus, pins, DEVICE, GPIOB, OLATB, GPIOA, OLATA):
    man_mode = True
    while man_mode == True:
        read_pressure(bus)
        if readpin(dev2pin('button_blow'), bus, DEVICE, dev2block('button_blow')) == 1 and \
            readpin(dev2pin('button_suck'), bus, DEVICE, dev2block('button_suck')) == 0:
                pump_operate(bus, pins, DEVICE, OLATB, 'blow')
        elif readpin(dev2pin('button_blow'), bus, DEVICE, dev2block('button_blow')) == 0 and \
            readpin(dev2pin('button_suck'), bus, DEVICE, dev2block('button_suck')) == 1:
                pump_operate(bus, pins, DEVICE, OLATB, 'vacuum')
        elif readpin(dev2pin('button_blow'), bus, DEVICE, dev2block('button_blow')) == 1 and \
            readpin(dev2pin('button_suck'), bus, DEVICE, dev2block('button_suck')) == 1:
                pump_operate(bus, pins, DEVICE, OLATB, 'refill')
        else:
            pump_operate(bus, pins, DEVICE, OLATB, 'off')
        if readpin(dev2pin('toggle'), bus, DEVICE, dev2block('toggle')) == 0:
                man_mode = False
        time.sleep(0.25)


def pump_operate(bus, pins, DEVICE, OLATB, direction):
    if direction == 'blow':
        pins[dev2pin('pump_motor')] = 1
        pins[dev2pin('valve_blow')] = 1
        pins[dev2pin('valve_suck')] = 0
        pins[dev2pin('valve_stop')] = 0
    elif direction == 'off':
        pins[dev2pin('pump_motor')] = 0
        pins[dev2pin('valve_blow')] = 0
        pins[dev2pin('valve_suck')] = 0
        pins[dev2pin('valve_stop')] = 0
    elif direction == 'vacuum':
        pins[dev2pin('pump_motor')] = 1
        pins[dev2pin('valve_blow')] = 0
        pins[dev2pin('valve_suck')] = 1
        pins[dev2pin('valve_stop')] = 0
    elif direction == 'refill':
        pins[dev2pin('pump_motor')] = 1
        pins[dev2pin('valve_blow')] = 0
        pins[dev2pin('valve_suck')] = 1
        pins[dev2pin('valve_stop')] = 1

    set_pins(bus, pins, DEVICE, OLATB)
    print("air pump set to " + direction)

def read_pressure(bus):
    GAIN = 1 #-2048 to 2047 ADC represents -4.096V to +4.096V
    adcRaw_pressure = adc_aircylinder.read_adc(adcpin_airpressure, gain=GAIN)
    #voltage of sensor = (4/10^3kPa)*pressure + 1 <--from PSE530-R06 spec sheet
    #voltage = ADC output x (4.096/2047) <--from ADS1015 specs with GAIN = 1
    print('ADC Raw Pressure Reading = ' + str(adcRaw_pressure))
    pressure = (adcRaw_pressure * (4.096/2047) - 1) * (1000/4) #pressure in kPa
    print('Pressure = ' + str(pressure) + 'kPa')

    return pressure

def uint8_to_binary_string(uint8):
    return bin(uint8)[2:].zfill(8)

def read_pins(bus, DEVICE, GPIO):
    print(DEVICE, GPIO)
    int_pins = bus.read_byte_data(DEVICE, GPIO)
    print(int_pins)
    str_pins = uint8_to_binary_string(int_pins)
    print(str_pins)
    pins = list(map(int, reversed(str_pins)))
    print pins
    return pins

def readpin(pin, bus, DEVICE, GPIO):
    int_pins = bus.read_byte_data(DEVICE, GPIO)
    str_pins = uint8_to_binary_string(int_pins)
    pins = list(map(int, reversed(str_pins)))
    readpin = pins[pin]
    return readpin

def set_pins(bus, pins, DEVICE, OLAT):
    print(pins)
    #print(type(pins))
    intBinOut = pins[::-1]
    #print(intBinOut)
    pins_set = int(''.join(map(str,intBinOut)),base=2)
    bus.write_byte_data(DEVICE, OLAT, pins_set)

def read_distance():

    # Create a VL53L0X object
    tof = VL53L0X.VL53L0X()

    # Start ranging
    tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

    distance = tof.get_distance()
    if (distance > 0):
        print ("%d mm, %d cm" % (distance, (distance/10)))

    tof.stop_ranging()

    return distance

if __name__ == "__main__":
    
    adcpin_airpressure = 0
    adcpin_cylinderpos = 1
    
    DEVICE = 0x20 #Device address
    IODIRB = 0x01 #Pin direction register for toggle switch input, solenoid and pump outputs
    OLATB = 0x15 #Register for outputs
    GPIOB = 0x13 #Register for inputs
    IODIRA = 0x00 #Pin direction register for positive/vacuum pressure button inputs and LED output
    OLATA = 0x14
    GPIOA = 0x12

    adc_aircylinder = Adafruit_ADS1x15.ADS1015(address=0x48, busnum=1)
    
    values = [0]*4

    bus = smbus.SMBus(1)
    bus.write_byte_data(DEVICE, IODIRB, 0b00000001) #sets GPB0 as input and GPB 1-7 as output
    bus.write_byte_data(DEVICE, IODIRA, 0b11110111) #sets GPA3 as output (LED) and all others as input

    pins = read_pins(bus, DEVICE, GPIOB) #this maps to MCP23017 pins GPB0-GPB7


    while True:
        print('Toggle Value Is ' + str(readpin(dev2pin('toggle'), bus, DEVICE, GPIOB)))
        if readpin(dev2pin('toggle'), bus, DEVICE, GPIOB) == 1:
           bus.write_byte_data(DEVICE, OLATA, 0b00001000) 
           manual_mode(bus, pins, DEVICE, GPIOB, OLATB, GPIOA, OLATA)
        else:
            bus.write_byte_data(DEVICE, OLATA, 0b00000000)
            maintain_pressure(bus, pins, DEVICE, GPIOB, OLATB, 275) #threshold in kPa
        time.sleep(0.5)

    set_pins(bus, [0,0,0,0,1,0,0,0], DEVICE, OLATB)
    time.sleep(0.5)
    print(read_pins(bus, DEVICE, GPIOB)) 

    distance = read_distance()
    print("%d mm, %d cm" % (distance, (distance/10)))






