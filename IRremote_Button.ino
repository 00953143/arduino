#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#define DECODE_NEC
const byte ledPin = 2;       // Builtin-LED pin
const byte interruptPin = 0; // BOOT/IO0 button pin
volatile byte state = LOW;

void setup() {
  pinMode(LED_BULTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("START"__FILE__"from"__DATE__"\r\nUsing library version"VERSION_IRREMOTE"

  IrSender.begin();

  Serial.print(F("Ready to semd IR signals at pin ");
  Serial.print(IR_SEND_PIN);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  digitalWrite(ledPin, state);
}

void blink() {
  state = !state;
}
