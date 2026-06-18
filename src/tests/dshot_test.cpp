#include <Arduino.h>
#include <PIO_DShot.h>

DShotX4 *esc;

void setup() {
  esc = new DShotX4(D8, 1, 300);

  // Try arming with raw 0 for 5 seconds
  uint16_t arm_throttles[4] = {0, 0, 0, 0};
  uint32_t start = millis();
  while (millis() - start < 5000) {
    esc->sendThrottles(arm_throttles);
    delay(1);
  }
}

void loop() {
  uint16_t throttles[4] = {300, 0, 0, 0};

  // Constant throttle 300
  esc->sendThrottles(throttles);
  delay(1);
}
