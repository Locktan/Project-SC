int one = 0;
int two = 0;
int three = 0;
int four = 0;


void setup() {
  // ut your setup code here, to run once:
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  

    one = analogRead(A0);
    two = analogRead(A1);
    three = analogRead(A2);
    four = analogRead(A3);

    //delay(2);
    
    Serial.print(one);
    Serial.print(" ");
    Serial.print(two);
    Serial.print(" ");
    Serial.print(three);
    Serial.print(" ");
    Serial.print(four);
    Serial.println(" ");
    delay(50);
    
}
