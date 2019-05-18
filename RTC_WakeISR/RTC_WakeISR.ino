#include "RTCZero.h"
#include <Wire.h>

RTCZero rtc;

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Wire.begin();
  delay(3000);
  while (!SerialUSB && (millis() < 10000)) {
      // Wait for USB to connect
  }
  Serial.begin(115200);
  
  
  rtc.begin();
  // Set the alarm at the 59 second mark
  rtc.setAlarmSeconds(59);

  // Match only seconds (Periodic alarm every minute)
  rtc.enableAlarm(RTCZero::MATCH_SS);

  // Attach ISR
  rtc.attachInterrupt(RTC_ISR);
  
  // Set sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  rtc.setEpoch(0);


  Serial.println("start");
  // Sleep sketch startup delay
  delay(10000);
  do_flash_led(LED_BLUE);
}

void loop()
{
  sleepMode();

  rtc.setTime(0,0,0);
  rtc.setAlarmSeconds(59);
  rtc.enableAlarm(RTCZero::MATCH_SS);
  
}


void sleepMode()
{
  
  // Disable USB
  // USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

  Serial.println("going to sleep");
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  delay(5000);
  Serial1.begin(57600);
  Serial1.println("sys sleep 30000");
  delay(100);
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW);

  
  
  //Enter sleep mode
  __WFI();
  
  // ...Sleep
  
  // Enable USB
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  Serial.println("awake");
  // Stay awake for two seconds
  delay(5000);
  
}

void RTC_ISR()
{
  Serial.println("ISR");
  do_flash_led(LED_RED);
  do_flash_led(LED_GREEN);
  do_flash_led(LED_BLUE);
}

void do_flash_led(int pin)
{
    for (size_t i = 0; i < 6; ++i) {
        delay(100);
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
}
