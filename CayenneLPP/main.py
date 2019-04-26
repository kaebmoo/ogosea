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
rtc = machine.RTC()
rtc.ntp_sync("pool.ntp.org")
utime.sleep_ms(1000)
print('\nRTC Set from NTP to UTC:', rtc.now())
utime.timezone(25200)
print('Adjusted from UTC to EST timezone', utime.localtime(), '\n')

py = Pytrack()
l76 = L76GNSS(py, timeout=30)
li = LIS2HH12(py)

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
    print('Acceleration', li.acceleration())
    print('Roll', li.roll())
    print('Pitch', li.pitch())
    lpp.add_accelerometer(1, li.acceleration()[0], li.acceleration()[1], li.acceleration()[2])
    lpp.add_gryrometer(1, li.roll(), li.pitch(), 0)

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
        lpp.add_gps(1, coord[0], coord[1], 0)

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
    time.sleep(30)
