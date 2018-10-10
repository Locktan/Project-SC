#include "arduinoFFT.h" //fft

#define samples 128 //fft
#define samplingFreq 1000 //fft
#define samplingSirenFreq //main

arduinoFFT FFT = arduinoFFT(); //fft

unsigned int samplingPeriodFFT; //fft //recMicVals
unsigned int samplingPeriodAmp; //not used currently
unsigned long microseconds;//fft //recMicVals
//unsigned long millis; //main


double vReal[samples]; //fft
double VImag[samples]; //fft

int xCorrSize = 7; //sample size xcorr
//int shiftSize = (trueArraySize * 2) - 1; //xcorr


int sumSirenSquare = 0; //xcorr

int siren[xCorrSize] = {}//copy and paste data here for the siren; //xcorr

unsigned long millitime = 0; //remicvals

int t1 = 0; //recmicvals
int t2 = 0; //recmicvals

int recMicVals[samples]; //recmicvals
int micro1 = 0;//recmicvals




int mic1 = 0;





void setup() {
   Serial.begin(115200);
   samplingPeriod = round(1000000 * (1.0/samplingFreq));
   samplingPeriodFFT = round(1000 * (1.0/samplingSirenFreq))

}

void loop() {

  ////////////////////////////////making FFT array
  
  //int fftArray[30] = {0};
  mic1 = analogRead(A0);
  Serial.println(mic1);
  
  //for(int x = 0; x < 30;x++)

  
  ////////////////////////////////
  

}
void recMicVals(int numOfSamples) //modifies the global array
{
  Serial.println("Collecting data...");
  t1 = micros();
  for(int i = 0; i < numOfSamples; i++)
  {
      microseconds = micros();
      recMicVals[i] = analogRead(A0);

      while(micros() < microseconds + samplingPeriod))
      {
        //wait
      }
   t2 = micros();

   Serial.println("Done Collecting Data(micros):" + (((double)(t2 - t1))/1000000));
   

  }
  
  
  
}

int FFT() //returns the most prominant freuency in the array
{
  //for(int i = 0; i < samples; i++) //runnning through the whole array to sample
  //{
    //microseconds = micros()
     
    vReal = recMicVals(samples);//need to change this to make it more versitile
    //vImag[i] = 0;

    //while(micros() < microseconds + samplingPeriod)
    //{
      //do nothing
    //}
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  double peak = FFT.MajorPeak(vReal, samples, samplingFreq);

  return peak
  
}
