/*
   @file    LSM6DSV16X_DataLog_Terminal.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X inertial measurement sensor
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

LSM6DSV16XSensor sensor(&Wire);
int32_t accel[3], angrate[3];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();
}

void loop()
{
  sensor.Get_X_Axes(accel);
  sensor.Get_G_Axes(angrate);

  Serial.print("Accel-X[mg]:");
  Serial.print(accel[0]);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel[1]);
  Serial.print(",Accel-Z[mg]:");
  Serial.println(accel[2]);

  Serial.print("AngRate-X[mdps]:");
  Serial.print(angrate[0]);
  Serial.print(",AngRate-Y[mdps]:");
  Serial.print(angrate[1]);
  Serial.print(",AngRate-Z[mdps]:");
  Serial.println(angrate[2]);

  blink(LED_BUILTIN);
}

inline void blink(int pin)
{
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
