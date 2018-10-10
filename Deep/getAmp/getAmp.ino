#include <TimerOne.h>

//const int mic = A0;

int mic = 0;

void setup() {
  // put your setup code here, to run once:
  Timer1.initialize(2000); //triggers 5 times a second
  Timer1.attachInterrupt(getMicVal);
  Serial.begin(115200);
}

void loop() {

  ///unsigned long micCopy;

  

  
  
  
  
  
  
  
}

void getMicVal(void)
{
  mic = analogRead(A0);
  Serial.println(mic);
}
