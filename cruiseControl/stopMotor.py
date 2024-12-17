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
import statistics

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
Motor_Speed(pca, 0)
