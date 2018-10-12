#include "arduinoFFT.h"
 
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC

//For FFT
arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

//For corrrelation
int trueArraySize = 80; //test
int shiftSize = (trueArraySize*2) - 20; //for test ans = 13, change this for smaller shift 13 or less and remember to change the siren(less zero)
  
//Store the siren but with zero in front
long sumSirenSquareFire = 0.00;
long sumSirenSquareSmoke = 0.00;
long sumSirenSquareTornado1 = 0.00;
long sumSirenSquareTornado2 = 0.00;

//double siren[7] = {0.1, 0.2, -0.1, 4.1, -2.0, 1.5, -0.1}; //test but not going to work with unsigned long
int sirenFire[80] = {3361, 3345, 3343, 3373, 3713, 3387, 3571, 3550, 3324, 3291, 3493, 3509, 3324, 3349, 3398, 3378, 3325, 466, 582, 420, 256, 749, 263, 269, 375, 1003, 368, 582, 547, 737, 3664, 3376, 3722, 3333, 3325, 3502, 3471, 3793, 3599, 3397, 3345, 3334, 3528, 3566, 3329, 3378, 3343, 3347, 3352, 3565, 568, 1149, 661, 476, 454, 375, 369, 539, 553, 573, 1017, 868, 574, 923, 872, 3721, 3694, 3420, 3401, 3370, 3351, 3380, 3374, 3349, 3639, 3646, 3692, 3301, 3557, 3549};
int sirenSmoke[80] = {275, 552, 413, 372, 378, 280, 594, 804, 358, 510, 375, 249, 926, 573, 545, 853, 539, 584, 340, 608, 575, 589, 656, 565, 274, 547, 565, 580, 1003, 580, 572, 439, 614, 380, 575, 484, 371, 239, 835, 952, 574, 960, 270, 561, 571, 983, 502, 605, 493, 589, 561, 552, 634, 574, 551, 507, 578, 573, 376, 595, 280, 625, 357, 562, 569, 873, 584, 3137, 3137, 3142, 3137, 3135, 3138, 3138, 3136, 3136, 3138, 3137, 3139, 3137};
int sirenTornado1[80] = {625, 628, 631, 624, 630, 623, 630, 632, 624, 2485, 2492, 2483, 2482, 2495, 2481, 2490, 2485, 2500, 2490, 2488, 2488, 2489, 624, 625, 636, 625, 623, 631, 632, 630, 626, 640, 631, 629, 637, 629, 631, 633, 625, 625, 628, 630, 628, 628, 625, 632, 622, 632, 631, 629, 633, 630, 630, 627, 633, 627, 629, 630, 629, 629, 627, 635, 629, 627, 632, 630, 630, 631, 634, 628, 629, 629, 631, 627, 635, 635, 621, 634, 628, 633};
int sirenTornado2[80] = {349, 274, 372, 319, 347, 261, 288, 247, 282, 274, 263, 280, 304, 305, 302, 315, 315, 315, 342, 319, 352, 365, 357, 360, 367, 376, 374, 387, 382, 395, 387, 386, 446, 393, 431, 414, 445, 432, 437, 460, 397, 446, 363, 372, 466, 427, 462, 485, 477, 477, 474, 254, 244, 259, 272, 284, 273, 277, 277, 283, 287, 275, 274, 282, 285, 288, 275, 285, 292, 283, 293, 286, 294, 296, 293, 309, 296, 308, 303, 308};

int newSirenFire[140];
int newSirenSmoke[140];
int newSirenTornado1[140];
int newSirenTornado2[140];

//Fill the array that store the input and will be use for shifting
//double input[7] = {0.1, 4.0, -2.2, 1.6, 0.1, 0.1, 0.2}; //test but not going to work with unsigned long
double input[80];
double newInput[140];
 
void setup() 
{
    Serial.begin(9600);

    //Fill the front of the siren with zero
    for (int i = 0; i < shiftSize;)
    {
      newSirenFire[i] = 0;
      newSirenSmoke[i] = 0;
      newSirenTornado1[i] = 0;
      newSirenTornado2[i] = 0;
      i++;
    
      if (i == trueArraySize - 1)
      {
        for (int j =0; i < shiftSize; i++, j++)
        {
          newSirenFire[i] = sirenFire[j];
          newSirenSmoke[i] = sirenSmoke[j];
          newSirenTornado1[i] = sirenTornado1[j];
          newSirenTornado2[i] = sirenTornado2[j];
        }
      }
    }

    //Sum of siren with ^2
    //For cross corr
    for(int m = 0; m < shiftSize; m++)
    {
      sumSirenSquareFire = sumSirenSquareFire + pow(newSirenFire[m], 2);
      sumSirenSquareSmoke = sumSirenSquareSmoke + pow(newSirenSmoke[m], 2);
      sumSirenSquareTornado1 = sumSirenSquareTornado1 + pow(newSirenTornado1[m], 2);
      sumSirenSquareTornado2 = sumSirenSquareTornado2 + pow(newSirenTornado2[m], 2);
    }
 
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

    //Serial.print("Press 'y + enter' to start: ");
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
      
    //collect input
    for (int i = 0; i < trueArraySize; i++)
    {   
      //SAMPLING
      for(int i=0; i<SAMPLES; i++)
      {
          microseconds = micros();    //Overflows after around 70 minutes!
       
          vReal[i] = analogRead(A0);
          vImag[i] = 0;
       
          while(micros() < (microseconds + sampling_period_us)){
          }
      }
   
      //FFT
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
      int peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  
      //Serial.print(peak);
      //Serial.print(", ");
  
      input[i] = peak;
      delay(25);
    }
    Serial.print("\n");
  
    /*
    //Checking the input
    for(int i = 0; i < trueArraySize; i++)
    {
      Serial.print(input[i]);
      Serial.print(", ");
    }
    Serial.println(""); */
  
  
      correlation();
      //while(1); //Run code once
  }
    
}

void correlation() 
{
  double numTemp1 = 0.00;
  double numTemp2 = 0.00;

  //Fill the input array with zero
  for(int i = trueArraySize; i < shiftSize; i++)
  {
    if(i == trueArraySize)
    {
      for(int j = 0; j < trueArraySize; j++)
        newInput[j] = input[j];
    }

    newInput[i] = 0;
  }
/*
  //Checking the array
  for(int k = 0; k < shiftSize; k++)
  {
    Serial.print(newInput[k]);
    Serial.print(" ");
  }
  Serial.println(""); 
*/

  //Correlation code
  for(int i = 0; i < shiftSize; i++)
  { 
    //Sum of newInput with ^2
    double sumNewInputSquare = 0.00;
    
    for(int l = 0; l < shiftSize; l++)
    {
      sumNewInputSquare = sumNewInputSquare + pow(newInput[l], 2);
    }
      
    //correlation
    double corrFire = 0.00;
    double corrSmoke = 0.00;
    double corrTornado1 = 0.00;
    double corrTornado2 = 0.00;

    for(int k = 0; k < shiftSize; k++)
    {
      corrFire = corrFire + (newInput[k] * newSirenFire[k]);
      corrSmoke = corrSmoke + (newInput[k] * newSirenSmoke[k]);
      corrTornado1 = corrTornado1 + (newInput[k] * newSirenTornado1[k]);
      corrTornado2 = corrTornado2 + (newInput[k] * newSirenTornado2[k]);
    }

    //normalized correlation
    double normCorrFire = corrFire / sqrt(sumSirenSquareFire * sumNewInputSquare);
    double normCorrSmoke = corrSmoke / sqrt(sumSirenSquareSmoke * sumNewInputSquare);
    double normCorrTornado1 = corrTornado1 / sqrt(sumSirenSquareTornado1 * sumNewInputSquare);
    double normCorrTornado2 = corrTornado2 / sqrt(sumSirenSquareTornado2 * sumNewInputSquare);

    if (normCorrFire >= 0.90)
      Serial.println("Fire signal detected");
    else if (normCorrSmoke >= 0.90)
      Serial.println("Smoke signal detected");
    else if (normCorrTornado1 >= 0.90)
      Serial.println("Tornado1 signal detected");
    else if (normCorrTornado2 >= 0.90)
      Serial.println("Tornado2 signal detected");
    /*else 
      Serial.println("No corralation");/*
  
    //Reset tempNum
    numTemp1 = 0.00;
    numTemp2 = 0.00;

    //Shifting
    for(int j = i; j < shiftSize; j++)
    {
      numTemp2 = newInput[j];
      newInput[j] = numTemp1;
      numTemp1 = numTemp2;
    }
/*
    //Checking the shifting
    for(int k = 0; k < shiftSize; k++)
    {
      Serial.print(newInput[k]);
      Serial.print(" ");
    }
    Serial.println("");   
*/
  }
}
