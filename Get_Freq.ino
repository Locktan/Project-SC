#include "arduinoFFT.h"
 
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC

#define loopSample 80
 
arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

int fftArray[loopSample];

void setup() 
{
  Serial.begin(115200);

  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
}

void loop() 
{
  int incomingChar = 'y';

  if (incomingChar == 'n')
  {
    //send data only when you receive data:
    if (Serial.available() > 0)
    {
      // read the incoming byte:
      incomingChar = Serial.read();
    }
  }
  
  if (incomingChar == 'y')
  {
    //Serial.println("Run start!!!");
    for (int p = 0; p < loopSample; p++)
    {
      for(int i=0; i<SAMPLES; i++)
      {
          microseconds = micros();    //Overflows after around 70 minutes!
       
          vReal[i] = analogRead(A0);
          vImag[i] = 0;
       
          while(micros() < (microseconds + sampling_period_us))
          {}
      }
   
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
      int peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

      //fftArray[p] = peak;

      Serial.println(peak);     //Print out what frequency is the most dominant.
      //Serial.print(", ");
      //delay(25);
    }
/*
    for (int q = 0; q < loopSample; q++)
    {
      Serial.print(fftArray[q]);     //Print out frequency in array
      Serial.print(", ");
    }*/
  }
}
