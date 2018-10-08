
#include "arduinoFFT.h"
//Deep Mistry test

#define samples 256 //samples per FFT
#define fftSamples 30 //fftSampleFreq * 5 for 5 seconds
#define sirenArrSize 30
#define xCorrSize 59

#define samplingFreq 10000 
#define fftSamplingFreq 10


arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned int fftSamplingPeriod;

unsigned long microseconds;

double vReal[samples];
double vImag[samples];


//int trueArraySize = samples;



double sumSirenSquare = 0.00;
double samplingFFT[fftSamples];// sample 5 times per sec for 5 secs
double samplingFFTPadded[xCorrSize];


double sirenFFT[sirenArrSize] = {439.13, 439.71, 438.43, 440.00, 439.18, 439.28, 439.39, 439.08, 439.26, 439.28, 438.89, 439.41, 439.73, 438.76, 439.28, 439.08, 439.44, 438.92, 439.93, 439.85, 438.84, 439.47, 439.42, 439.08, 440.02};
 //this is the full high detail array that we compare the the sampleFFT to
double sirenFFTPadded[xCorrSize];


double xCorrArray[xCorrSize]; //final xcorr arr



void setup() {
  Serial.begin(115200);
  samplingPeriod = round(1000000 * (1.0/samplingFreq));
  fftSamplingPeriod = round(1000000 * (1.0/fftSamplingFreq));
  
  

}

void loop() 
{
  //populateFFTArray();
  correlation();
  /*for(int x = 0; x < xCorrSize; x++)
  {
    Serial.println(sirenFFT[x]);
  }*/
  delay(5000);

  
}

int CalcFFT() //returns the most prominant freuency in the array
{
  for(int i = 0; i < samples; i++) //runnning through the whole array to sample
  {
    microseconds = micros();
     
    vReal[i] = analogRead(A0);//need to change this to make it more versitile
    vImag[i] = 0;

    while(micros() < microseconds + samplingPeriod)
    {
      //do nothing
    }
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  double peak = FFT.MajorPeak(vReal, samples, samplingFreq);

  return peak;
  
}

void populateFFTArray()
{
  for(int x = 0; x < fftSamples; x++) //populate the samplingFFT[] array
  {
    samplingFFT[x] = CalcFFT();

    while(micros() < microseconds + fftSamplingPeriod);
    {
      //wait
    }
  }
}

void correlation()
{

  double shiftTemp = 0;
  double xCorrTemp = 0;

 /* for(int i = fftSamples; i < xCorrSize; i++) //zero padding the samplefftArray to be the same size as the xCorrFFT
  {
    Serial.print("samplingPadded:");
    if(i == fftSamples)
    {
      for(int j = 0; j< fftSamples; j++)
      {
        samplingFFTPadded[j] = samplingFFT[j];
        Serial.println(samplingFFTPadded[j]);
        
      } 
    }
    samplingFFTPadded[i] = 0;
    Serial.println(samplingFFTPadded[i]);
    
  } 
*/

  for(int j = 0; j< sirenArrSize; j++)
      {
        sirenFFTPadded[j] = sirenFFT[j];
        //Serial.print("sirenFFTPadded[j]:");
        ///Serial.println(sirenFFTPadded[j]);
      } 
      //Serial.println("Done filling");

  for(int k = sirenArrSize; k < xCorrSize; k++) //zero padding the sirenFFTArray to be the same size as the xCorrFFT
  {
    
 
    sirenFFTPadded[k] = 0;
    //Serial.print("sirenFFTPadded[i]:");
    //Serial.println(sirenFFTPadded[k]);
    
  }
  //Serial.println("Done padding");

  
  
  for(int x = 0; x < xCorrSize; x++) //shifts values over one
  {
    shiftTemp = samplingFFTPadded[xCorrSize - 1]; //stores the last value of the array of FFT samples
    for(int y = xCorrSize - 1; y > 0; y--) //start from the end and work towards the front
    {
      samplingFFTPadded[y] = samplingFFTPadded[y-1]; //shifts the data to the position on its right
      
    }
    samplingFFTPadded[0] = shiftTemp; //moves end to begin

    for(int x = 0; x < xCorrSize; x++)
    {
      Serial.print("samplingFFTPadded: ");
      Serial.println(samplingFFTPadded[x]);
    }
  
  
    for(int z = 0; z < xCorrSize; z++) //runs through every value of both arrays to find xCorr value
    {
      xCorrTemp += sirenFFTPadded[z] * sirenFFTPadded[z];

      //Serial.print(sirenFFTPadded[z]);
     // Serial.print(xCorrTemp);
    }

    xCorrArray[x] = xCorrTemp;
    Serial.println(xCorrTemp);
    
    xCorrTemp = 0;
    

    
  
  }





}


  
 
  
  

 
