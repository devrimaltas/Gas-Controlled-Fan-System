# Graphic Library and Time Library
import numpy as np
import matplotlib.pyplot as plt
import datetime as dt

# MQ-X Sensor Library,ADC Library and GPIO Library
import os
from time import sleep
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import RPi.GPIO as GPIO

# GPIO and PWM Setup
DC_MOTOR = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(DC_MOTOR, GPIO.OUT)
GPIO.setwarnings(False)
pwm = GPIO.PWM(DC_MOTOR, 100)  # PWM Adjustment
pwm.start(0)  # PWM Start

k = 0
m = 0
# Create degerler.txt
open("degerler.txt", "w").close()

# Create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# Create the cs
cs = digitalio.DigitalInOut(board.D22)
# Create the mcp object
mcp = MCP.MCP3008(spi, cs)
# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

print('Raw ADC Value: ', chan0.value)
print('ADC Voltage: ' + str(chan0.voltage) + 'V')

last_read = 0  # this keeps track of the last potentiometer value
tolerance = 250  # to keep from being jittery we'll only change


# Convert 16-bit ADC
def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min
    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)
    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

# Values and Time Write in .txt file
def yazdir():
    deger = str(PWM)
    f = open("degerler.txt", "a")
    f.write("{},{}\n".format(dt.datetime.now().strftime('%H:%M'), deger))
    f.close()

fig, ax = plt.subplots()

while True:

    trim_pot_changed = False
    # Read the analog pin(channel-0 )
    trim_pot = chan0.value

    # how much has it changed since the last read?
    pot_adjust = abs(trim_pot - last_read)

    if pot_adjust > tolerance:
        trim_pot_changed = True

    if trim_pot_changed:
        #Calculate first PWM value.
        if (m == 0):
            # Convert 16bit adc (0-65535) trim pot read into 0-100 volume level
            last_read = trim_pot
            PWM = remap_range(trim_pot, 0, 65535, 0, 100)
        #Calculate another PWM value
        if (m == 1):
            # Convert 16bit adc (0-65535) trim pot read into 0-100 volume level
            last_read = trim_pot
            PWM = remap_range(trim_pot, 0, 65535, 0, 100)

        #Writes PWM value to terminal
        print('Volume = {volume}%'.format(volume=PWM))
        set_vol_cmd = 'sudo amixer cset numid=1 -- {volume}% > /dev/null' \
            .format(volume=PWM)
        os.system(set_vol_cmd)
    #Another values write in .txt file
    if (m == 1):
        # Call yazdir() function
        yazdir()
    #First value write in .txt file
    if (m == 0):
        deger = str(PWM)
        f = open("degerler.txt", "a")
        f.write("{},{}\n".format(dt.datetime.now().strftime('%H:%M'), deger))
        f.close()

    # Fan control with PWM
    if (4 < PWM < 31):
        pwm.ChangeDutyCycle(PWM + 31)
    else:
        pwm.ChangeDutyCycle(PWM)

    # Read data in degerler.txt and Create Graphic
    graph_data = open("degerler.txt", "r").read()
    lines = graph_data.split()
    xs = []
    ys = []
    fig.canvas.manager.show()
    for line in lines:
        if len(line) > 1:
            x, y = line.split(",")
            xs.append(x)
            ys.append(int(y))
        ax.clear()
        ax.plot(xs, ys)
    plt.xticks(rotation=70)
    plt.xlabel("Time", fontsize=12, color="red")
    plt.ylabel("% Gas Density", fontsize=12, color="red")
    fig.canvas.draw()
    fig.canvas.flush_events()

    m = 1
    k = k + 1
    # Every hour clear the collected data because Numbers are overlapping on the X axis
    if (k == 60):
        open("degerler.txt", "w").close()
    sleep(60)