#include <TimerOne.h>

//const int mic = A0;

IntervalTimer timerAmp; //we can have upto 4 timers going simultaniously but unlimited timer objects

int mic = 0;

void setup() {
  // put your setup code here, to run once:
  //Timer1.initialize(2000); //triggers 5 times a second
  //Timer1.attachInterrupt(getMicVal);
  Serial.begin(115200);
  timerAmp.begin(getMicVal, 150000); //getMicVal() will run every .15 seconds
}

void loop() {

  
  
}

void getMicVal()
{
  mic = analogRead(A0);
  Serial.println(mic);
}
