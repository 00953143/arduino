#include <Arduino.h>
#include <esp32-hal-dac.h>

#define LED1 25
#define LED2 26
 
void setup(){
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
}
 
void loop()
{
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle = dutyCycle + 1)
  {
    dacWrite(LED1, dutyCycle); 
    dacWrite(LED2, 255 - dutyCycle);  
    delay(5);
  }
 
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle = dutyCycle - 1)
  {
    dacWrite(LED1, dutyCycle);  
    dacWrite(LED2, 255 - dutyCycle);  
    delay(5);
  }
}
