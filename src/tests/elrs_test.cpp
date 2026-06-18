#include <Arduino.h>
#include <AlfredoCRSF.h>
#include "pin_config/pin_config.hpp"

AlfredoCRSF crsf;

void setup() {
  Serial.begin(115200);
  
  // Wait a moment for USB CDC Serial to connect
  delay(3000);
  Serial.println("\n\n--- Starting ELRS CRSF Receiver Test ---");
  Serial.println("Referencing pinmap: RX Pin = D7 (GPIO 1), TX Pin = D6 (GPIO 0)");

  // Configure Serial1 to use the pins defined in the pinmap.csv
  Serial1.setRX(ELRS_RX_PIN);
  Serial1.setTX(ELRS_TX_PIN);
  
  // Initialize Serial1 with CRSF standard baudrate
  Serial1.begin(CRSF_BAUDRATE);
  
  crsf.begin(Serial1);
  Serial.println("ELRS Receiver initialized on Serial1.");
}

void loop() {
  // Update receiver state (must be called frequently to parse incoming packets)
  crsf.update();

  // Print receiver status and channel values periodically
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();

    if (crsf.isLinkUp()) {
      Serial.print("Link: UP | ");
      for (int i = 1; i <= 8; i++) {
        Serial.print("Ch");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(crsf.getChannel(i));
        Serial.print(" | ");
      }
      
      // Print link statistics
      const crsfLinkStatistics_t* stats = crsf.getLinkStatistics();
      if (stats) {
        Serial.print("LQ: ");
        Serial.print(stats->uplink_Link_quality);
        Serial.print("% | RSSI: -");
        Serial.print(stats->uplink_RSSI_1); // standard CRSF RSSI is usually positive in the struct representing -dBm
        Serial.print(" dBm | Power: ");
        Serial.print(stats->uplink_TX_Power);
        Serial.print(" mW");
      }
      Serial.println();
    } else {
      Serial.println("Link: DOWN (No CRSF packets received)");
    }
  }
}
