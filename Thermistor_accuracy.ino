V = ReadVoltage(A0); //accuracy improved

double ReadVoltage(byte pin){
  double reading = analogRead(pin); 
  if(reading < 1 || reading > 4095) return 0;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} 
V = ReadVoltage(A0); 
R = 9990 * V / (5 - V); 
T = 1 / (1/T0 + (log(R)-log(R0)) / B ); 
Temp = T - 273.15;

Serial.println(Temp);
