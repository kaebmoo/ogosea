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
#include "RTCZero.h"
#include <ArduinoJson.h>
#include <CayenneLPP.h>
#include <Sodaq_LSM303AGR.h>


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

#define debugSerial SERIAL_PORT_MONITOR
#define DEBUG_STREAM SerialUSB


#if defined(ARDUINO_AVR_SODAQ_MBILI)
#define loraSerial Serial1
#define LORA_STREAM Serial1
#define BEE_VCC 20

#elif defined(ARDUINO_SODAQ_AUTONOMO) || defined(ARDUINO_SODAQ_ONE) || defined(ARDUINO_SODAQ_ONE_BETA)
#define loraSerial Serial1
#define LORA_STREAM Serial1

#elif defined(ARDUINO_SODAQ_EXPLORER)
#define loraSerial Serial2
#define LORA_STREAM Serial2

#else
// please select a sodaq board
debugSerial.println("Please select a sodaq board!!");
#endif

// ABP
const uint8_t devAddr[4] = { 0x74, 0x80, 0x16, 0x31 };
const uint8_t appSKey[16] = { 0x16, 0x28, 0xAE, 0x2B, 0x7E, 0x15, 0xD2, 0xA6, 0xAB, 0xF7, 0xCF, 0x4F, 0x3C, 0x15, 0x88, 0x09 };
const uint8_t nwkSKey[16] = { 0x28, 0xAE, 0xD2, 0x2B, 0x7E, 0x15, 0x16, 0xA6, 0x09, 0xCF, 0xAB, 0xF7, 0x15, 0x88, 0x4F, 0x3C };

// OTAA

uint8_t DevEUI[8] = { 0x12, 0x96, 0xD0, 0x11, 0x74, 0x80, 0x16, 0x31 };
uint8_t AppEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppKey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t busy = 0;
volatile bool minuteFlag;


Timer timerLED, timerSendData;
RTCZero rtc;
uint8_t _isDebugOn = 0;
uint8_t _isLedEnabled = 1;

CayenneLPP lpp(60);
Sodaq_LSM303AGR accelerometer;

// The interrupt pin for the Accelerometer is attached to D4
#define ACC_INT_PIN 4

// Threshold for interrupt trigger
double threshold = -0.8;
int now = millis();

void setup()
{
  //Power up the LoRaBEE
  #if defined(ARDUINO_AVR_SODAQ_MBILI) || defined(ARDUINO_SODAQ_AUTONOMO)
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
  #endif
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  delay(3000);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Wire.begin();

  accelerometer.rebootAccelerometer();
  delay(1000);

  // Enable the Accelerometer
  accelerometer.enableAccelerometer();

  // Attach interrupt event fot the Accelerometer
  pinMode(ACC_INT_PIN, INPUT);
  attachInterrupt(ACC_INT_PIN, interrupt_event, RISING);

  // Enable interrupts on the SAMD 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
      GCLK_CLKCTRL_GEN_GCLK1 |
      GCLK_CLKCTRL_CLKEN;

  // If Z goes below threshold the interrupt is triggered
  accelerometer.enableInterrupt1(accelerometer.ZLow, threshold, 0, accelerometer.PositionRecognition);

  
    delay(3000);
    while (!SerialUSB && (millis() < 10000)) {
        // Wait for USB to connect
    }

    MySerial.begin(57600);
    loraSerial.begin(LoRaBee.getDefaultBaudRate());

    digitalWrite(LED_RED, HIGH);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);
    pinMode(LED_BLUE, OUTPUT);

    // Debug output from LoRaBee
  LoRaBee.setDiag(debugSerial); // optional
  LoRaBee.init(loraSerial, LORA_RESET);

  RED();
  //connect to the LoRa Network
  setupLoRa();
  LEDOFF();

    do_flash_led(LED_RED);
    do_flash_led(LED_GREEN);
    do_flash_led(LED_BLUE);

    MySerial.println("SODAQ LoRaONE test_gps is starting ...");

    sodaq_gps.init(GPS_ENABLE);
    sodaq_gps.setDiag(MySerial);

    // This is for debugging to see more details, more messages
    // Use this in combination with setDiag()
    // sodaq_gps.setMinNumOfLines(10);

    // Uncomment the next line if you want to see the incoming $GPxxx messages
    // sodaq_gps.setDiag(MySerial);

    SerialUSB.println(getBatteryVoltage());
    // First time finding a fix
    // find_fix(0);

    
    sendData();
    timerSendData.every(300000, sendData);

    // Sleep sketch startup delay
    delay(10000);
    // initRtc();
}

void loop()
{
  /*
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
  */
  // Print sensor readings every second
    if ((now + 1000) < millis())
    {
        now = millis();
        read_AccMeter();
    }
    // systemSleep();
    timerSendData.update();
}



bool scanGPS()
{
  int Hour;
  
  MySerial.println("GPS scan.");
  if (sodaq_gps.scan(true))
    {
      Hour = sodaq_gps.getHour()+7; // time zone GMT+7
      MySerial.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
      MySerial.println(String(" time = ") + Hour + String(":") + sodaq_gps.getMinute() + String(":") + sodaq_gps.getSecond());
      MySerial.print("We are at latitude ");
      MySerial.print(sodaq_gps.getLat(), 6);
      MySerial.print(" longitude ");
      MySerial.print(sodaq_gps.getLon(), 6);
      MySerial.print(" altitude ");
      MySerial.print(sodaq_gps.getAlt(), 1);
      MySerial.print(" and HDOP ");
      MySerial.println(sodaq_gps.getHDOP(), 2);
      return true;
    }
  return false;
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

void setupLoRaABP(){  
  
  if (LoRaBee.initABP(Serial1, devAddr, appSKey, nwkSKey, false))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("Communication to LoRaBEE failed!");
  }
}

void setupLoRaOTAA(){
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, false))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("OTAA Setup failed!");
  }
}


void setupLoRa(){
  // ABP
  setupLoRaABP();
  
  // OTAA
  // setupLoRaOTAA();
  // LoRaBee.setSpreadingFactor(9);
}

void sendPacket() // String packet
{
  LEDOFF();
  BLUE();
  // switch (LoRaBee.send(1, (uint8_t*)packet.c_str(), packet.length()))
  switch (LoRaBee.sendReqAck(1, lpp.getBuffer(), lpp.getSize(), 10))
    {
    case NoError:  
      GREEN();
      debugSerial.println("Successful transmission.");
      break;
    case NoResponse:
      RED();
      debugSerial.println("There was no response from the device.");
      setupLoRa();
      break;
    case Timeout:
      RED();
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 2sec.");
      delay(2000);
      break;
    case PayloadSizeError:
      RED();
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      RED();
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case Busy:
      RED();
      debugSerial.println("The device is busy. Sleeping for 2 extra seconds.");      
      delay(2000);
      setupLoRa(); 
      sendData();     
      break;
    case NetworkFatalError:
      RED();
      debugSerial.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NotConnected:
      RED();
      debugSerial.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NoAcknowledgment:
      RED();
      debugSerial.println("There was no acknowledgment sent back!");
      // When you this message you are probaly out of range of the network.
      break;
    default:
      break;
    }
  LEDOFF();
}

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void LEDOFF() 
{
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void sendData()
{
  String packet;

  read_AccMeter();
  scanGPS();
  lpp.reset();
  lpp.addAnalogInput(3, (float) getBatteryVoltage()/1000);
  lpp.addAccelerometer(3, accelerometer.getX(), accelerometer.getY(), accelerometer.getZ());
  lpp.addGPS(3, sodaq_gps.getLat(), sodaq_gps.getLon(), sodaq_gps.getAlt());

  
  // sendPacket(packet);
  sendPacket();
}

/**
 * Initializes the CPU sleep mode.
 */
void initSleep()
{
    // Set the sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * Initializes the RTC.
 */
void initRtc()
{
    rtc.begin();
    debugSerial.println("Init RTC.");  
    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

    // Attach handler
    rtc.attachInterrupt(rtcAlarmHandler);


    // Set sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // This sets it to 2000-01-01
    // rtc.setEpoch(0);
}

/**
 * Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler()
{
  debugSerial.println("RTC Alarm Handler.");  
  minuteFlag = true;
  digitalWrite(LED_GREEN, LOW);
  delayMicroseconds(500000);
  digitalWrite(LED_GREEN, HIGH);
  sendData();
  digitalWrite(LED_RED, LOW);
  delayMicroseconds(500000);
  digitalWrite(LED_RED, HIGH);
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    debugSerial.println("Flush LoRa Serial.");
    LORA_STREAM.flush();

    debugSerial.println("Turn LED Off.");    
    LEDOFF();

    
      // Disable USB
      USBDevice.detach();

      Serial.println("going to sleep");
      //Enter sleep mode
      // SAMD sleep
      __WFI();
      // ...Sleep
  
      // Enable USB and wait for resume if attached
      USBDevice.attach();
      USB->DEVICE.CTRLB.bit.UPRSM = 0x01u;
      while (USB->DEVICE.CTRLB.bit.UPRSM);
    
      
    // Enable USB
    USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
  
    Serial.println("awake");
    // Stay awake for two seconds
    delay(2000);

    /*
    // setGpsActive(false); // explicitly disable after resetting the pins
    
    
    // go to sleep, unless USB is used for debugging
    if (!_isDebugOn || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        noInterrupts();
        if (!(sodaq_wdt_flag || minuteFlag)) {
            interrupts();
            debugSerial.println("Going to sleep.");
            __WFI(); // SAMD sleep
        }
        interrupts();
    }
    */
    
}

void read_AccMeter()
{
    debugSerial.print("x = ");
    debugSerial.print(accelerometer.getX());

    debugSerial.print("\ty = ");
    debugSerial.print(accelerometer.getY());

    debugSerial.print("\tz = ");
    debugSerial.println(accelerometer.getZ());
}

void interrupt_event()
{
    // Do not print in an interrupt event when sleep is enabled.
    debugSerial.println("Board flipped");
}
