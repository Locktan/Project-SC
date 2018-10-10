//calibration code

#include "arduinoFFT.h"
#define samples 128
#define samplingFreq 25000 //needs to be atleast 21k plus headroom to see if the fft is working properly

 //change this to correct


arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microseconds;

double vReal1[samples];
double vImag1[samples];



int incomingByte;

void setup() 
{
 // pinMode(motor, OUTPUT);
  Serial.begin(115200);
  samplingPeriod = round(1000000*(1.0/samplingFreq));
}

void loop() 
{
   //while(Serial.available() == 0) { } //wait till keypress
   FFTVerification();

   
   

}


boolean FFTVerification() //dac goes 0-3.3V @ 1mA
{
      //analogWrite(motorPin, 100) //pwm is 0-255 so this gives us about 39% duty cycle and a voltage of 1.02V not 0-4096. 4096 is for the dac for finer signals
      
     for(int i=0; i<samples; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vReal1[i] = analogRead(A0);
        vImag1[i] = 0;

        

        
     
        while(micros() < (microseconds + samplingPeriod))
        {
          //wait
        }

        
    }

        FFT.Windowing(vReal1, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal1, vImag1, samples, FFT_FORWARD);
        double peak1 = FFT.MajorPeak(vReal1, samples, samplingFreq);
        Serial.println(peak1);

        
     
}
