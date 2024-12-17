#Reading of 0 means magnet detected,
#reading of 1 means no magnet detected

import RPi.GPIO as IO
import time
import sys
import argparse
import busio
import smbus
from time import sleep
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import math
from board import SCL,SDA
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo
import matplotlib
import matplotlib.pyplot as plt

def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Motor_Start(pca):
   x = input("Press and hold EZ button. Once the LED turns red, immediately relase the button. After the LED blink red once, press 'ENTER'on keyboard.")
   Motor_Speed(pca, 1)
   time.sleep(2)
   y = input("If the LED just blinked TWICE, then press the 'ENTER'on keyboard.")
   Motor_Speed(pca, -1)
   time.sleep(2)
   z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on keyboard to proceed")
   

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
   print(speed/65535)

IO.setwarnings(False)
IO.setmode(IO.BCM)

GPIO_num = 16
IO.setup(GPIO_num,IO.IN,IO.PUD_UP)

pca = Servo_Motor_Initialization()
#Motor_Start(pca)
Motor_Speed(pca, 0.15)
#sleep(1)
#Motor_Speed(pca, 0.13)

last_pin_val = 1
run_time = 5
start_time = time.time()
times = []
speeds = []
prev_magnet_time = start_time
new_magnet_time = 0
distance = math.pi*0.0711

while time.time() - start_time < run_time:
    curr_pin_val = IO.input(GPIO_num)
    if curr_pin_val == 0 and last_pin_val == 1:
        new_magnet_time = time.time()
        dt = new_magnet_time - prev_magnet_time
        speeds.append(distance/dt)
        times.append(new_magnet_time-start_time)
        print(distance/dt)
        prev_magnet_time = new_magnet_time
    last_pin_val = curr_pin_val

Motor_Speed(pca, 0)

x = input('Press 1 to create speed vs. time figure')
if x == '1':
    title = input('Enter filename for the figure (w/ .png)') 
    plt.clf()
    plt.plot(times, speeds)
    plt.grid(True)
    plt.title('Car speed for 15% duty cycle')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.savefig(title)

        
