#include "arduinoFFT.h" //fft

#define samplesAmp 256 //fft
#define samplingAmpFreq 10000 //fft
#define samplesFFT 30
#define samplingFFTFreq 10

arduinoFFT FFT = arduinoFFT(); //fft

unsigned int samplingPeriodFFT;

unsigned int samplingPeriodAmp;
unsigned long microseconds;
unsigned long microsecondsFFT;

double vReal[samplesAmp]; //fft
double vImag[samplesAmp]; //fft


int recMicValsAmp[samplesAmp];
//int recSirenFFT[samplesFFT];
double FFTArray[samplesFFT];

double FFToutput = 0;



void setup() {

   Serial.begin(2000000);
   samplingPeriodAmp = round(1000000 * (1.0/samplingAmpFreq));
   samplingPeriodFFT = round(1000000 * (1.0/samplingFFTFreq));

}



void loop() {

  
  printFFTToArray(samplesFFT);
  
}

void recMicVals(int numOfSamples) //modifies the global array
{
 
 
  for(int i = 0; i < numOfSamples; i++)
  {
      microseconds = micros();
      vReal[i] = analogRead(A0);
      vImag[i] = 0;

      while(micros() < microseconds + samplingPeriodAmp)
      {
        //wait
      }
  } 
}

double FFTCalc() //returns the most prominant freuency in the array
{

  recMicVals(samplesAmp);
  
  FFT.Windowing(vReal, samplesAmp, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samplesAmp, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samplesAmp);
  double peak = FFT.MajorPeak(vReal, samplesAmp, samplingAmpFreq);

  return peak;
  
}

void printFFTToArray(int FFTArrSize)
{
  Serial.print("{");
  for(int i = 0; i < FFTArrSize; i++)
  {
      microsecondsFFT = micros();

      Serial.print(FFTCalc());
      Serial.print(", ");
      

      while(micros() < microsecondsFFT + samplingPeriodFFT)
      {
        //wait
      }
  }

  Serial.println("}");

  
  
}
