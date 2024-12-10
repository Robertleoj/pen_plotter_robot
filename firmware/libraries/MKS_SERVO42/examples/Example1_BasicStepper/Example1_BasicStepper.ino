#include "MKS_SERVO42.h"

byte const stepperId = 1;
uint8_t const speed = 1600;
uint32_t const numberOfPulses = 5000;

MKS_SERVO42 stepper;

void setup() {
  Serial.begin(115200);
  while (!Serial.available())
    delay(0);
  Serial1.begin(38400);
  stepper.initialize(&Serial1);
  stepper.ping(stepperId);
}

byte direction = 1;

void loop() {
  Serial.println(stepper.getCurrentPosition(stepperId));
  direction = direction == 0 ? 1 : 0;
  stepper.setTargetPosition(stepperId, direction, speed, numberOfPulses);
  delay(5000);
}
