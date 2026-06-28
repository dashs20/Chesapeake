/*
   @file    LSM6DSV16X_MLC.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSV16X Machine Learning Core
 *******************************************************************************
   Copyright (c) 2023, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/
// Includes
#include "LSM6DSV16XSensor.h"
#include "lsm6dsv16x_activity_recognition_for_mobile.h"

#define INT_1 PA4

//Interrupts.
volatile int mems_event = 0;

LSM6DSV16XSensor sensor(&Wire);

// MLC
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

void INT1Event_cb();
void printMLCStatus(uint8_t status);

void setup()
{
  lsm6dsv16x_mlc_out_t  mlc_out;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initlialize i2c.
  Wire.begin();

  // Initlialize components.
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();

  /* Feed the program to Machine Learning Core */
  /* Activity Recognition Default program */
  ProgramPointer = (ucf_line_t *)lsm6dsv16x_activity_recognition_for_mobile;
  TotalNumberOfLine = sizeof(lsm6dsv16x_activity_recognition_for_mobile) / sizeof(ucf_line_t);
  Serial.println("Activity Recognition for LSM6DSV16X MLC");
  Serial.print("UCF Number Line=");
  Serial.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if (sensor.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
      Serial.print("Error loading the Program to LSM6DSV16XSensor at line: ");
      Serial.println(LineCounter);
      while (1) {
        // Led blinking.
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
      }
    }
  }

  Serial.println("Program loaded inside the LSM6DSV16X MLC");

  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  /* We need to wait for a time window before having the first MLC status */
  delay(3000);

  sensor.Get_MLC_Output(&mlc_out);
  printMLCStatus(mlc_out.mlc1_src);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    lsm6dsv16x_mlc_status_mainpage_t  status;
    sensor.Get_MLC_Status(&status);
    if (status.is_mlc1) {
      lsm6dsv16x_mlc_out_t mlc_out;
      sensor.Get_MLC_Output(&mlc_out);
      printMLCStatus(mlc_out.mlc1_src);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

void printMLCStatus(uint8_t status)
{
  switch (status) {
    case 0:
      Serial.println("Activity: Stationary");
      break;
    case 1:
      Serial.println("Activity: Walking");
      break;
    case 4:
      Serial.println("Activity: Jogging");
      break;
    case 8:
      Serial.println("Activity: Biking");
      break;
    case 12:
      Serial.println("Activity: Driving");
      break;
    default:
      Serial.println("Activity: Unknown");
      break;
  }
}
