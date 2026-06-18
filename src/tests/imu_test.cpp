#include "ICM42688.h"
#include <Arduino.h>
#include <SPI.h>

// SPI Pin Mapping for Seeed XIAO RP2350
// The default hardware SPI on the XIAO RP2350 uses:
// SCK -> D8
// MISO -> D9
// MOSI -> D10
// We use D11 (Bottom Pad) for Chip Select.

const int CS_PIN = D11;

// Instantiate ICM42688 on SPI bus
ICM42688 imu(SPI, CS_PIN);

void setup() {
  Serial.begin(115200);

  // Wait a moment for the USB CDC to connect before spamming
  delay(3000);

  Serial.println("\n\n--- Starting ICM-42688 SPI Test ---");

  // Do NOT explicitly set SCK/TX/RX, let the core handle the defaults
  // Initialize IMU
  int status = imu.begin();
  if (status < 0) {
    Serial.print("IMU initialization failed with status code: ");
    Serial.println(status);
    while (1) {
      delay(1000); // keep yielding so serial flushes
    }
  }

  Serial.println("ICM-42688 initialized successfully.");

  // Set output data rate to 100 Hz
  imu.setGyroODR(ICM42688::odr100);
}

void loop() {
  // Read data from sensor
  imu.getAGT();

  // Print gyro data (dps)
  Serial.print("Gyro X: ");
  Serial.print(imu.gyrX());
  Serial.print(" | Y: ");
  Serial.print(imu.gyrY());
  Serial.print(" | Z: ");
  Serial.println(imu.gyrZ());

  delay(10);
}
