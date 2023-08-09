#the code purpose is to test system integration between Pi, Arduino to flight controller
#if certain distance is read from arduino (using serial connection)
# drone will be arm
# success arm means this 3 component able to respond to each other.

import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import math
import serial
import RPi.GPIO as GPIO
import threading
import time
# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)
# Replace '/dev/ttyACM0' with the appropriate serial port name
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)



def arm():
    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("STABILIZE")# vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.flush()
    vehicle.armed   = True
    time.sleep(5)
    print ("ARMING SUCCESS")
    time.sleep(5)



# Function to continuously read data from Arduino
def read_data():
    ser.setDTR(False)
    time.sleep(0.3)
    ser.flushInput()
    ser.setDTR(True)
    time.sleep(0.3)
    while True:
        if ser.in_waiting >0:
            try:
                # Read data from Arduino
                data = ser.readline().decode().rstrip()
                # Convert the received data to an integer
                data = int(data)
                return data
            except UnicodeDecodeError:
                # Handle the UnicodeDecodeError and continue reading
                print("UnicodeDecodeError: Unable to decode the received data")


while True:
    distance=read_data()
    # Print the received data
    print("Distance from obstacle : ", distance)
    if distance<50:
        arm()
        time.sleep(5)
    elif distance >250:
        vehicle.armed =False
    time.sleep(2)
    

