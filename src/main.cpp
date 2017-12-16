#include "Arduino.h"

const int SPEAKER_PIN = 10;
const int LED_PIN = 13;

void setup() {
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(SPEAKER_PIN, 127);
}

auto state = HIGH;

void loop() {
  digitalWrite(LED_PIN, state);
  if (state == HIGH){
    state = LOW;
  } else {
    state = HIGH;
  }
  delay(500);
}
