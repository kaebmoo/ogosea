/*
Copyright (c) 2017, SODAQ
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <Sodaq_UBlox_GPS.h>
#include <Wire.h>
#include <Sodaq_RN2483.h>
#include "Timer.h"
#include <Sodaq_wdt.h>

#define ADC_AREF 3.3f
// #define BATVOLT_R1 2.0f // One v1
// #define BATVOLT_R2 2.0f // One v1
#define BATVOLT_R1 4.7f // One v2
#define BATVOLT_R2 10.0f // One v2
#define BATVOLT_PIN BAT_VOLT

#define MySerial        SERIAL_PORT_MONITOR
#define ARRAY_DIM(arr)  (sizeof(arr) / sizeof(arr[0]))

// List of interval values to be used in loop()
// to measure how long it takes to get a fix.
uint32_t intervals[] = {

        // Do a few tests with 1 minute delay
        1UL * 60 * 1000,
        1UL * 60 * 1000,
        1UL * 60 * 1000,

        // Try a few longer delays
        2UL * 60 * 1000,
        2UL * 60 * 1000,
        5UL * 60 * 1000,
        5UL * 60 * 1000,

        // Slowly increase the delays
        15UL * 60 * 1000,
        30UL * 60 * 1000,
        1UL * 60 * 60 * 1000,
        3UL * 60 * 60 * 1000,
        4UL * 60 * 60 * 1000,
        8UL * 60 * 60 * 1000,
};
size_t interval_ix = 0;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 60 * 1000;           // interval at which to blink (milliseconds)

void find_fix(uint32_t delay_until);
void do_flash_led(int pin);

void setup()
{
    delay(3000);
    while (!SerialUSB) {
        // Wait for USB to connect
    }

    MySerial.begin(57600);

    digitalWrite(LED_RED, HIGH);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);
    pinMode(LED_BLUE, OUTPUT);

    do_flash_led(LED_RED);
    do_flash_led(LED_GREEN);
    do_flash_led(LED_BLUE);

    MySerial.println("SODAQ LoRaONE test_gps is starting ...");

    sodaq_gps.init(GPS_ENABLE);

    // This is for debugging to see more details, more messages
    // Use this in combination with setDiag()
    // sodaq_gps.setMinNumOfLines(10);

    // Uncomment the next line if you want to see the incoming $GPxxx messages
    // sodaq_gps.setDiag(MySerial);

    SerialUSB.println(getBatteryVoltage());
    // First time finding a fix
    find_fix(0);
}

void loop()
{
    uint32_t wait_ms = intervals[interval_ix];
    if (++interval_ix > ARRAY_DIM(intervals)) {
        interval_ix = 0;
    }
    // find_fix(wait_ms);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      do_flash_led(LED_RED);
      SerialUSB.println(getBatteryVoltage());
      find_fix(0);
    }
}

/*!
 * Find a GPS fix, but first wait a while
 */
void find_fix(uint32_t delay_until)
{
    MySerial.println(String("delay ... ") + delay_until + String("ms"));
    delay(delay_until);

    uint32_t start = millis();
    uint32_t timeout = 900L * 1000;
    MySerial.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
    if (sodaq_gps.scan(false, timeout)) {
        MySerial.println(String(" time to find fix: ") + (millis() - start) + String("ms"));
        MySerial.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
        MySerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        MySerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
        MySerial.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
        do_flash_led(LED_GREEN);
    } else {
        MySerial.println("No Fix");
        do_flash_led(LED_BLUE);
    }
}

void do_flash_led(int pin)
{
    for (size_t i = 0; i < 2; ++i) {
        delay(100);
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
}

uint16_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));

    return voltage;
}
