/*
   @file    LSM6DSV16X_Free_Fall_Detection.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X Free Fall Detection
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/
#include <LSM6DSV16XSensor.h>

#define INT1_pin PA4

LSM6DSV16XSensor LSM6DSV16X(&Wire);

//Interrupts.
volatile int mems_event = 0;
void INT1Event_cb();


void setup()
{

  // Initlialize serial.
  Serial.begin(115200);
  delay(1000);

  // Initlialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initlialize i2c.
  Wire.begin();

  // Enable INT1 pin.
  attachInterrupt(INT1_pin, INT1Event_cb, RISING);

  // Initlialize components.
  LSM6DSV16X.begin();
  LSM6DSV16X.Enable_X();

  // Enable Free Fall Detection.
  LSM6DSV16X.Enable_Free_Fall_Detection(LSM6DSV16X_INT1_PIN);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16X_Event_Status_t status;
    LSM6DSV16X.Get_X_Event_Status(&status);

    if (status.FreeFallStatus) {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Free Fall Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
