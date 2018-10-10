int motorPin = 9;
void setup() {
  // put your setup code here, to run once:
  
  pinMode(motorPin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(motorPin, 100); 
  
  
}
