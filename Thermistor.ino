int16_t ADCvalue;

void setup(){
  Serial.begin(9600);
}

void loop() {
  ADCvalue = analogRead(A0);
  Thermistor(ADCvalue);
}
