#!/usr/bin/python

import smbus, time, math

'''
The program is suited to testing servos.

Plug servo in Ch 0 on Sparkfun servo hat.

Running this program will move the servo between PWMRange specified in variables.
'''

print('----------------')
print('Initialise servo hat.')
print('----------------')

bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
addr = 0x40           # I2C address of the PWM chip.
bus.write_byte_data(addr, 0, 0x20)     # enable the chip
bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write

print('----------------')
print('Initialise variables.')
print('----------------')

PWMRange = 800

print('----------------')
print('Initialise Ch0 PWM on Servo Hat.')
print('----------------')

ChStartID0 = 0x06
ChStopID0 = 0x08

print('----------------')
print ('Start ID = ' + '%2.0f' % ChStartID0)
print ('Stop ID = ' + '%2.0f' % ChStopID0)
print('----------------')

## CHANNEL 0 - Micro servo
bus.write_word_data(addr, 0x06, 0)     # chl 0 start time = 0us
time.sleep(1)   # pause at neutral for one seconds
#print('1250')
#bus.write_word_data(addr, 0x08, 1250)  # chl 0 end time = 1.5ms

pwm = 0
bus.write_word_data(addr, int(ChStopID0), pwm)

print('----------------')
print('Press BREAK now to stop pwm at ZERO.')
print('----------------')
time.sleep(1)

x = 0

while True:
    try:    
        b = bus.read_word_data(addr, ChStopID0)
        print('----------------')
        print ('Read word data :' + '%2.0f' % b)
        print('----------------')

        pwm = int(1250-PWMRange + PWMRange*2*(math.sin(x*math.pi/360)))
        print('x: ', x)
        x += 5
        if x > 360:
            x = 0

        bus.write_word_data(addr, ChStopID0, pwm)

        print('pwm: ' + str(pwm))
        print('Press Ctrl + C to stop.')
        print('----------------')
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

pwm = 0
bus.write_word_data(addr, ChStopID0, pwm)
print('----------------')
print('Turing power off.')
print('pwm: ' + str(pwm))
print('----------------')
time.sleep(0.5)

print('----------------')
print('Program End')
print('----------------')
