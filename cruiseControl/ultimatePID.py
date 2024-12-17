#Reading of 0 means magnet detected,
#reading of 1 means no magnet detected
import argparse
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

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--speed', action='store', type=float, default = 2,
                    help = 'speed in meters per second')
parser.add_argument('--P', action='store', type=float, default = 1,
                    help = 'Proportional gain')
parser.add_argument('--I', action='store', type=float, default = 0,
                    help = 'Integral gain')
parser.add_argument('--D', action='store', type=float, default = 0,
                    help = 'Derivitive gain')
args = parser.parse_args()
input_speed = args.speed
Kp = args.P
Ki = args.I
Kd = args.D


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
   print(speed)
   #if speed < -1:
   #     speed = -1
   #elif speed > 1:
   #     speed = 1
   pca.channels[15].duty_cycle = math.floor(speed)
   #print(speed/65535)

def calc_dc(speed):
   dc = (speed+7.26)/72.1
   return dc

def get_error(curr_speed, target_speed, acc_error, dt):
   error = target_speed - curr_speed
   all_error.append(error)
   acc_error = acc_error+error*dt
   d_error = (error - all_error[-2])/dt
   return error, acc_error, d_error

def PIDControl(Kp, Ti, Td, error, acc_error, d_error):
   newSpeed = Kp*(error+1/Ti*acc_error+Td*d_error)
   return newSpeed

IO.setwarnings(False)
IO.setmode(IO.BCM)

GPIO_num = 16
IO.setup(GPIO_num,IO.IN,IO.PUD_UP)


dc = calc_dc(input_speed)
pca = Servo_Motor_Initialization()
Motor_Speed(pca, dc)

Ti = Kp/Ki
Td = Kp/(4*Ki)

last_pin_val = 1
run_time = 5
start_time = time.time()
times = []
speeds = []
prev_magnet_time = start_time
new_magnet_time = 0
distance = math.pi*0.0711
all_error = [0] #all error calculations
acc_error = 0 #accumulated error
newSpeed = 0
newDC = dc

while time.time() - start_time < run_time:
    curr_pin_val = IO.input(GPIO_num)
    Motor_Speed(pca, newDC)
    if curr_pin_val == 0 and last_pin_val == 1:
        new_magnet_time = time.time()
        dt = new_magnet_time - prev_magnet_time
        curr_speed = distance/dt
        speeds.append(curr_speed)
        times.append(new_magnet_time-start_time)
        error, acc_error, d_error = get_error(curr_speed, input_speed, acc_error, dt)
        print(error)
        newSpeed = PIDControl(Kp, Ti, Td, error, acc_error, d_error)
        newDC = calc_dc(curr_speed+newSpeed)
        Motor_Speed(pca, newDC)
        prev_magnet_time = new_magnet_time
    elif time.time() - prev_magnet_time > 0.5:
        new_magnet_time = time.time()
        speeds.append(0)
        times.append(new_magnet_time-start_time)
        curr_speed = 0
        error, acc_error, d_error = get_error(curr_speed, input_speed, acc_error, dt)
        print(error)
        newSpeed = PIDControl(Kp, Ti, Td, error, acc_error, d_error)
        newDC = calc_dc(curr_speed+newSpeed)
        Motor_Speed(pca, newDC)
        prev_magnet_time = new_magnet_time
    last_pin_val = curr_pin_val

Motor_Speed(pca, 0)

x = input('Press 1 to create speed vs. time figure')
if x == '1':
    title = 'Kp'+ str(Kp) + 'Kd' + str(Kd) + 'Ki' + str(Ki) + '.png'
    plt.clf()
    plt.plot(times, speeds)
    plt.grid(True)
    plt.title('Car Speed Overtime')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.savefig(title)

x = input('Press 1 to write speeds to a file')
if x == '1':
    title_speed = 'SKp'+ str(Kp) + 'Kd' + str(Kd) + 'Ki' + str(Ki) + '.txt'
    title_time = 'TKp'+ str(Kp) + 'Kd' + str(Kd) + 'Ki' + str(Ki) + '.txt'
    with open(title_speed, 'w+') as f:
    	for items in speeds:
        	f.write('%s\n' %items)
    	print("File written successfully")
    f.close()

    with open(title_time, 'w+') as f:
        for items in times:
                f.write('%s\n' %items)
    
        print("File written successfully")
    f.close()

        
print(times)
print('-')
print(speeds)
