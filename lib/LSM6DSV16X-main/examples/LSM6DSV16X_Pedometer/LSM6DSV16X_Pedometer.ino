/*
   @file    LSM6DSV16X_Pedometer.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X Pedometer
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
uint16_t step_count = 0;
uint32_t previous_tick;
char report[256];

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

  // Enable Pedometer.
  LSM6DSV16X.Enable_Pedometer(LSM6DSV16X_INT1_PIN);

  previous_tick = millis();
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16X_Event_Status_t status;
    LSM6DSV16X.Get_X_Event_Status(&status);

    if (status.StepStatus) {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

      LSM6DSV16X.Get_Step_Count(&step_count);
      snprintf(report, sizeof(report), "Step counter: %d", step_count);
      Serial.println(report);
    }
  }
  // Print the step counter in any case every 3000 ms
  uint32_t current_tick = millis();
  if ((current_tick - previous_tick) >= 3000) {
    LSM6DSV16X.Get_Step_Count(&step_count);
    snprintf(report, sizeof(report), "Step counter: %d", step_count);
    Serial.println(report);
    previous_tick = millis();
  }

}

void INT1Event_cb()
{
  mems_event = 1;
}
