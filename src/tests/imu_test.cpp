#include <LSM6DSV16XSensor.h>
#include <Arduino.h>
#include <SPI.h>

const int IMU_CS_PIN = 9;
const int IMU_SCK_PIN = 10;
const int IMU_MOSI_PIN = 11;
const int IMU_MISO_PIN = 12;

// Instantiate LSM6DSV16X on SPI1 bus
LSM6DSV16XSensor imu(&SPI1, IMU_CS_PIN);

void setup() {
  Serial.begin(115200);

  // Wait a moment for the USB CDC to connect before spamming
  delay(3000);

  Serial.println("\n\n--- Starting ICM-42688 SPI Test ---");

  // Do NOT explicitly set SCK/TX/RX, let the core handle the defaults
  // Initialize IMU
  SPI1.setSCK(IMU_SCK_PIN);
  SPI1.setTX(IMU_MOSI_PIN);
  SPI1.setRX(IMU_MISO_PIN);
  SPI1.begin();

  if (imu.begin() != LSM6DSV16X_OK) {
    Serial.println("IMU initialization failed.");
    while (1) {
      delay(1000); // keep yielding so serial flushes
    }
  }

  Serial.println("LSM6DSV16X initialized successfully.");

  imu.Enable_X();
  imu.Enable_G();
  imu.Set_X_ODR(960.0f);
  imu.Set_G_ODR(960.0f);
  imu.Set_G_FS(2000);
}

void loop() {
  // Read data from sensor
  int32_t gyro_raw[3];
  imu.Get_G_Axes(gyro_raw);

  // Print gyro data (dps)
  Serial.print("Gyro X: ");
  Serial.print(gyro_raw[0] / 1000.0);
  Serial.print(" | Y: ");
  Serial.print(gyro_raw[1] / 1000.0);
  Serial.print(" | Z: ");
  Serial.println(gyro_raw[2] / 1000.0);

  delay(10);
}
