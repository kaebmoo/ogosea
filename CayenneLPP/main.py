import machine
import math
import network
import os
import time
import utime
import gc
from machine import RTC
from machine import SD
from L76GNSS import L76GNSS
from pytrack import Pytrack
from network import LoRa
from network import WLAN
import socket
import ubinascii
import struct
import socket
import binascii
import pycom
from CayenneLPP import CayenneLPP
from LIS2HH12 import LIS2HH12

print('Pytrack: ogosea version 0.1 by kaebmoo@gmail.com')
time.sleep(2)
gc.enable()

# configure the WLAN subsystem in station mode (the default is AP)
#wlan = WLAN(mode=WLAN.STA)
#wlan.scan()     # scan for available networks
#wlan.connect(ssid='Red', auth=(WLAN.WPA2, '12345678'))
#while not wlan.isconnected():
#    pass
# print(wlan.ifconfig())

# setup rtc
# rtc = machine.RTC()
# rtc.ntp_sync("pool.ntp.org")
# utime.sleep_ms(1000)
# print('\nRTC Set from NTP to UTC:', rtc.now())
# utime.timezone(25200)
# print('Adjusted from UTC to EST timezone', utime.localtime(), '\n')


py = Pytrack()
l76 = L76GNSS(py, timeout=30)
accelerometer = LIS2HH12(py)


# display the reset reason code and the sleep remaining in seconds
# possible values of wakeup reason are:
# WAKE_REASON_ACCELEROMETER = 1
# WAKE_REASON_PUSH_BUTTON = 2
# WAKE_REASON_TIMER = 4
# WAKE_REASON_INT_PIN = 8

print("Wakeup reason: " + str(py.get_wake_reason()))
print("Approximate sleep remaining: " + str(py.get_sleep_remaining()) + " sec")
time.sleep(5)


# enable wakeup source from INT pin
# py.setup_int_pin_wake_up(False)

# enable activity and also inactivity interrupts, using the default callback handler
# py.setup_int_wake_up(True, True)

# set the acceleration threshold to 2000mG (2G) and the min duration to 200ms
# accelerometer.enable_activity_interrupt(2000, 200)

# sd = SD()
# os.mount(sd, '/sd')
# f = open('/sd/gps-record.txt', 'w')

# Initialise LoRa in LORAWAN mode.
# Please pick the region that matches where you are using the device:
# Asia = LoRa.AS923
# Australia = LoRa.AU915
# Europe = LoRa.EU868
# United States = LoRa.US915
lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.AS923)

# remove all the non-default channels
# for i in range(3, 16):
#     lora.remove_channel(i)

# create an ABP authentication params
dev_addr = struct.unpack(">l", ubinascii.unhexlify('74801631'))[0]
nwk_swkey = ubinascii.unhexlify('28AED22B7E1516A609CFABF715884F3C')
app_swkey = ubinascii.unhexlify('1628AE2B7E15D2A6ABF7CF4F3C158809')

# join a network using ABP (Activation By Personalization)
lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))
# wait until the module has joined the network
while not lora.has_joined():
    time.sleep(2.5)
    print('Not yet joined...')
    time.sleep(1.0)

print('LoRaWAN joined.')
# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

# make the socket blocking
# (waits for the data to be sent and for the 2 receive windows to expire)
s.setblocking(True)

while (True):
    lpp = CayenneLPP()
    print('\n\n** 3-Axis Accelerometer (LIS2HH12)')
    print('Acceleration', accelerometer.acceleration())
    print('Roll', accelerometer.roll())
    print('Pitch', accelerometer.pitch())
    lpp.add_accelerometer(2, accelerometer.acceleration()[0], accelerometer.acceleration()[1], accelerometer.acceleration()[2])
    lpp.add_gryrometer(2, accelerometer.roll(), accelerometer.pitch(), 0)

    coord = l76.coordinates()

    #f.write("{} - {}\n".format(coord, rtc.now()))
    print("{} - {} - {}".format(coord, rtc.now(), gc.mem_free()))

    # if coord[1] is None or coord[2] is None:
    if None in coord:
        print("No coordinates")
        lat_d = "0"
        lon_d = "0"
    else:
        lat_d = str(coord[0])
        lon_d = str(coord[1])
        lpp.add_gps(2, coord[0], coord[1], 0)

    # send some data
    # s.send(bytes([0x03]))
    # s.send(bytes(lat_d))
    # s.send(bytes([0x04]))
    # s.send(bytes([lon_d]))
    # data = "{ \"api_key\": \"TC7PKSGHP4JIBUMM\"," + "\"field1\":"  + lat_d + "," + "\"field2\":"  + lon_d + "}"
    # data = '{ "api_key": "TC7PKSGHP4JIBUMM",' + '"field1":'  + lat_d + ',' + '"field2":'  + lon_d + '}'
    # print(data)
    # s.send(data)

    print('Sending data (uplink)...')
    s.send(bytes(lpp.get_buffer()))

    # make the socket non-blocking
    # (because if there's no data received it will block forever...)
    s.setblocking(False)

    # get any data received (if any...)
    data = s.recv(64)
    print(data)
    print(lora.stats())


    pycom.heartbeat(False)
    pycom.rgbled(0x000000)
    time.sleep(1)
    pycom.rgbled(0xFF0000)  # Red
    time.sleep(1)
    pycom.rgbled(0x00FF00)  # Green
    time.sleep(1)
    pycom.rgbled(0x0000FF)  # Blue
    time.sleep(1)
    time.sleep(300)

    gc.collect()
# go to sleep for 5 minutes maximum if no accelerometer interrupt happens
# py.setup_sleep(300)
# py.go_to_sleep()
