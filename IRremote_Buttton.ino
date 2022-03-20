#include <Arduino.h>

const byte ledPin = 2;       
const byte interruptPin = 0;
volatile byte state = LOW;

#define DECODE_NEC          
#define DECODE_DISTANCE     

#include "PinDefinitionsAndMore.h"
#define INFO 

#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000
 
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), IRsender, FALLING);
  #if defined(_IR_MEASURE_TIMING) && defined(_IR_TIMING_TEST_PIN)
    pinMode(_IR_TIMING_TEST_PIN, OUTPUT);
#endif

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) || defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000);
#endif
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

#if defined(IR_SEND_PIN)
    IrSender.begin(); 
#else
    IrSender.begin(3, ENABLE_LED_FEEDBACK); 
#endif

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.print(F("at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
    Serial.println(IR_RECEIVE_PIN_STRING);
#else
    Serial.println(IR_RECEIVE_PIN);
#endif
    Serial.print(F("Ready to send IR signals at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
    Serial.println(IR_SEND_PIN_STRING);
#else
    Serial.println(IR_SEND_PIN);
#endif

#if FLASHEND >= 0x3FFF  

#  if !defined(SEND_PWM_BY_TIMER) && !defined(USE_NO_SEND_PWM) && !defined(ESP32)

    IrSender.enableIROut(38); // Call it with 38 kHz to initialize the values printed below
    Serial.print(F("Send signal mark duration is "));
    Serial.print(IrSender.periodOnTimeMicros);
    Serial.print(F(" us, pulse correction is "));
    Serial.print(IrSender.getPulseCorrectionNanos());
    Serial.print(F(" ns, total period is "));
    Serial.print(IrSender.periodTimeMicros);
    Serial.println(F(" us"));
#  endif

    Serial.print(RECORD_GAP_MICROS);
    Serial.println(F(" us is the (minimum) gap, after which the start of a new IR packet is assumed"));
    Serial.print(MARK_EXCESS_MICROS);
    Serial.println(F(" us are subtracted from all marks and added to all spaces for decoding"));
#endif
}

uint16_t sAddress = 0x0102;
uint8_t sCommand = 0x34;
uint8_t sRepeats = 1;
  
}

void loop() {
    Serial.println("IR receiving...");
    Serial.println();
    Serial.print(F("address=0x"));
    Serial.print(sAddress, HEX);
    Serial.print(F(" command=0x"));
    Serial.print(sCommand, HEX);
    Serial.print(F(" repeats="));
    Serial.println(sRepeats);
    Serial.flush();

    send_ir_data();
    delay((RECORD_GAP_MICROS / 1000) + 5);
    receive_ir_data();

    sAddress += 0x0101;
    sCommand += 0x11;
    sRepeats++;

    delay(100); 
}

void IRsender() {
    Serial.println("IR sending...");
    Serial.print(F("Sending: 0x"));
    Serial.print(sAddress, HEX);
    Serial.print(sCommand, HEX);
    Serial.println(sRepeats, HEX);

    if (sRepeats > 4) {
        sRepeats = 4;
    }
    IrSender.sendNEC(sAddress, sCommand, sRepeats);
}
