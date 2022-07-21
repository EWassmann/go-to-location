import os
import sys
import time
import smbus
import numpy as np
import threading as Thread
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick
from shapely.geometry import Polygon, Point
import multiprocessing as mp
import serial
from ublox_gps import UbloxGps
import geopy.distance
from numpy import arctan2,random,sin,cos,degrees

arduino = serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 2000000, #perhaps make this lower need to do research
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    timeout = 5,
    xonxoff = False,
    rtscts = False,
    dsrdtr = False,
    writeTimeout = 2
    )

port = serial.Serial('/dev/ttyACM1', baudrate=38400, timeout=1)
gps = UbloxGps(port)
counter = 0
locationlat = 90
locationlon = 70

def Left():
    arduino.write("1".encode()) 
    global b
    b = 1
def Forward():
    arduino.write("0".encode())
    global b
    b = 0
def Right():
    arduino.write("2".encode())
    global b
    b = 2
def Stop():
    arduino.write("4".encode())
    global b
    b = 4
def Back():
    arduino.write("3".encode())
    global b
    b = 3
def Search():
    arduino.write("5".encode())
    global b
    b = 5



final = (locationlat, locationlon)
Geofencecoordinates = list()
sensorfusion = madgwick.Madgwick(0.5)
input("press any key and enter to continue")
def begintrack():
    global counter
    global Geofencecoordinates
    while True:
        try:
            geo = gps.geo_coords()
            global x
            x = geo.lat
            #print(x)
            global y
            y = geo.lon
            #print(y)
        except (ValueError, IOError) as err:
            print(err)
            

track = mp.Process(target = begintrack)  

address = 0x68
bus = smbus.SMBus(8)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

imu.loadCalibDataFromFile("/home/gilblankenship/Projects/PythonCode/calib.json")


currTime = time.time()
print_count = 0
def mmagdwick():
    global roll
    global pitch
    global yaw
    while True:
        imu.readSensor()
        for i in range(10):
            newTime = time.time()
            dt = newTime - currTime
            currTime = newTime

            sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
                                        imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
            roll = sensorfusion.roll
            pitch = sensorfusion.pitch
            yaw = sensorfusion.yaw

        time.sleep(0.01)

mmagd = mp.Process(target = mmagdwick)

def howfar():
    global distance
    current = (x, y)
    distance = geopy.distance.distance(current,final).m
far = mp.Process(target = howfar)
track.start()
mmagd.start()
far.start()


while distance > .5:
    dl = locationlon - y
    equis = cos(x)*sin(dl)
    e = cos(locationlat)*sin(x)-sin(locationlat)*cos(x)*cos(dl)
    bearing = ((degrees(arctan2(equis,e))+360) % 360)
    if yaw > bearing - 10 and yaw< bearing +10 and b != 0:
        Forward()
    if yaw <bearing - 10 or yaw > bearing + 10 and b != 1:
        Left()

