/*
   @file    LSM6DSV16X_Sensor_Fusion.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X library with Sensor Fusion Low Power.
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

#include <LSM6DSV16XSensor.h>

#define ALGO_FREQ  120U /* Algorithm frequency 120Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
unsigned long startTime, elapsedTime;

LSM6DSV16XSensor AccGyr(&Wire);
uint8_t status = 0;
uint32_t k = 0;

uint8_t tag = 0;
float quaternions[4] = {0};

void setup()
{

  Serial.begin(115200);
  while (!Serial) {
    yield();
  }

  Wire.begin();
  // Initialize LSM6DSV16X.
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  // Enable Sensor Fusion
  status |= AccGyr.Set_X_FS(4);
  status |= AccGyr.Set_G_FS(2000);
  status |= AccGyr.Set_X_ODR(120.0f);
  status |= AccGyr.Set_G_ODR(120.0f);
  status |= AccGyr.Set_SFLP_ODR(120.0f);
  status |= AccGyr.Enable_Rotation_Vector();
  status |= AccGyr.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

  if (status != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to init/configure");
    while (1);
  }
  Serial.println("LSM6DSV16X SFLP Demo");
}

void loop()
{
  uint16_t fifo_samples;
  // Get start time of loop cycle
  startTime = millis();

  // Check the number of samples inside FIFO
  if (AccGyr.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // Read the FIFO if there is one stored sample
  if (fifo_samples > 0) {
    for (int i = 0; i < fifo_samples; i++) {
      AccGyr.FIFO_Get_Tag(&tag);
      if (tag == 0x13u) {
        AccGyr.FIFO_Get_Rotation_Vector(&quaternions[0]);

        // Print Quaternion data
        Serial.print("Quaternion: ");
        Serial.print(quaternions[3], 4);
        Serial.print(", ");
        Serial.print(quaternions[0], 4);
        Serial.print(", ");
        Serial.print(quaternions[1], 4);
        Serial.print(", ");
        Serial.println(quaternions[2], 4);

        // Compute the elapsed time within loop cycle and wait
        elapsedTime = millis() - startTime;

        if ((long)(ALGO_PERIOD - elapsedTime) > 0) {
          delay(ALGO_PERIOD - elapsedTime);
        }
      }
    }
  }
}
