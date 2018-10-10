#define arrlen 20000  //this should allow us to record for 3 seconds. We can make this longer by incorperating DMA
#define samplingFreq 2000
//#define samplesPerPeriod 5
 //(samplingFreq * secs * samplesPerPeriod)

unsigned int samplingPeriod;
unsigned long microseconds;
unsigned long millitime = 0;
int t1 = 0;
int t2 = 0;




int mic1[arrlen];
//int mic2[arrlen];
//int mic3[arrlen];
//int mic4[arrlen];

int micro1 = 0;





void setup() 
{

 Serial.begin(9600);
 samplingPeriod = round(1000000 * (1.0/samplingFreq));
 
}

void loop() 
{
  
  /*micro1 = analogRead(A0);
  Serial.println(micro1);
  delayMicroseconds(100);
  */

  
  
  while(Serial.available() == 0) { } //wait till keypress
  Serial.flush();


  Serial.println("Collecting data...");
  t1 = micros();
  for(int i=0; i<arrlen; i++)
    {
      
      microseconds = micros();
      mic1[i] = analogRead(A0);
      //mic2[i] = analogRead(A1);
      //mic3[i] = analogRead(A2);
      //mic4[i] = analogRead(A3);
    
    
        while(micros() < (microseconds + samplingPeriod))
        {
          //wait
        }
    }
   t2 = micros();
  
      Serial.println("Done Collecting Data.");
 
      Serial.println(((double)(t2 - t1))/1000002);
      
      
      //Serial.println("mic1\tmic2\tmic3\tmic4");
      //Serial.print("{");
      for(int i = 0; i < arrlen; i++)
      {
        Serial.println(mic1[i]);
        //Serial.print(", ");
        //Serial.print("\t");
        
//        Serial.print(mic2[i]);
//        Serial.print("\t");
//
//        Serial.print(mic3[i]);
//        Serial.print("\t");
//
//        Serial.print(mic4[i]);
//        Serial.println("\t");
         
      }
      //Serial.print("0}");

       

}
