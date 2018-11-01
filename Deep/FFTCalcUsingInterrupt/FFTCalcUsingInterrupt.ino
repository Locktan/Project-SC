//takes from 12k to 35k microseonds to take each FFT
#include <TimerOne.h>

IntervalTimer timerFFT;

#include "arduinoFFT.h"
#define samples 256
#define samplingFreq 10000//needs to be atleast 21k plus headroom to see if the fft is working properly


arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microseconds;
unsigned long microsecondsFFTTime;

double vReal1[samples];
double vImag1[samples];


void setup() 
{
  Serial.begin(115200);
  samplingPeriod = round(1000000*(1.0/samplingFreq));
  timerFFT.begin(FFTCalc, 250000); //runs 4 times a second
  //timerAmp.begin(getAmp, 20); 
}

void loop() 
{
     
   

}


void FFTCalc() //dac goes 0-3.3V @ 1mA
{
        microsecondsFFTTime = micros();
        getMicVals();
        FFT.Windowing(vReal1, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal1, vImag1, samples, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal1, vImag1, samples);
        double peak1 = FFT.MajorPeak(vReal1, samples, samplingFreq);
        microsecondsFFTTime = micros() - microsecondsFFTTime; 
        Serial.print(peak1);
        Serial.print(" in ");
        Serial.println(microsecondsFFTTime);
            
}

void getMicVals()
{
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
}
