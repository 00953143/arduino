
#include <ESP32Servo.h>

Servo myservo;  
int pos = 0;    
#if defined(ARDUINO_ESP32S2_DEV)
int servoPin = 17;
#else
int servoPin = 18;
#endif

void setup() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    
	myservo.attach(servoPin, 1000, 2000); 
	
}

void loop() {

	for (pos = 0; pos <= 180; pos += 30) { 
		
		myservo.write(pos);    
		delay(15);             
	}
	for (pos = 180; pos >= 0; pos -= 30) { 
		myservo.write(pos);   
		delay(15);            
	}
}
