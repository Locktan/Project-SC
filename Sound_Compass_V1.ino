#include "arduinoFFT.h"
 
#define SAMPLES 512             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC
 
arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS  10
#define TFT_RST 9 
#define TFT_DC  8

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define TFT_SCLK 13
#define TFT_MOSI 11

const int arraySize = SAMPLES/2;

double backUpAlarmSumSquare = 0.00;
double emergencyAlertSystem1SumSquare = 0.00;
double emergencyAlertSystem2SumSquare = 0.00;
double fireTruckHornSumSquare = 0.00;
double smokeAlarmNewSumSquare = 0.00;
double fireAlarmNewSumSquare = 0.00;
double tsunamiSirenSumSquare = 0.00;
double nuclearAlarmSumSquare = 0.00;
double policeSirenSumSquare = 0.00;
double tornadoSirenSumSquare = 0.00;

int backUpAlarm[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,88,36,199,216,158,170,105,140,168,167,164,321,263,102,232,214,163,214,181,218,108,38,77,63,60,52,84,113,72,80,94,87,83,79,50,38,162,1290,1413,243,13,8,18,11,12,10,14,5,14,7,10,14,12,8,17,10,13,7,8,13,9,6,10,11,14,10,10,9,9,6,9,15,23,17,16,21,15,13,24,27,24,29,34,30,339,966,527,90,85,92,92,70,39,18,35,29,30,33,31,17,17,8,14,14,11,13,20,16,9,16,10,11,8,8,9,14,13,11,11,13,12,14,16,7,7,7,4,5,6,3,9,9,76,99,19,7,8,6,4,2,4,4,4,6,5,6,5,4,6,5,7,7,9,7,4,8,9,10,6,9,9,7,5,6,6,8,8,9,12,13,7,7,9,6,4,7,6,10,9,7,11,12,11,9,8,9,9,9,3,10,22,10,5,7,8,6,5,6,4,5,13,13,6,4,8,6,7,9,14,15,10,11,11,5,4,11,11,6,12,7,13,17,17,15,25,24,24,411,657,185,39,24,14,8,7,19,14,14,15,6,6,9,9,8,6,4};
int emergencyAlertSystem1[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,9,4,5,3,7,4,6,4,4,5,6,5,6,6,3,3,5,4,6,8,5,7,4,6,6,7,6,3,6,6,7,5,6,5,6,4,3,3,3,3,4,6,6,8,8,8,4,4,4,3,9,7,5,3,4,5,11,5,3,6,4,4,9,7,6,9,8,6,6,5,5,6,2,4,2,3,3,6,7,14,6,4,8,5,5,4,5,4,3,4,6,5,3,5,4,4,4,2,3,4,4,5,4,4,3,4,4,3,3,4,4,6,5,6,3,4,5,4,4,5,4,5,5,5,5,3,4,4,5,4,5,3,3,2,4,3,2,4,3,4,2,4,5,4,3,4,4,3,6,7,3,3,5,4,4,6,5,2,4,3,3,2,3,4,4,4,3,3,2,3,4,3,3,3,3,4,4,3,4,5,6,3,5,5,1,3,5,4,4,6,13,7,5,5,4,3,4,3,2,3,4,5,5,3,4,4,5,5,3,3,3,3,3,3,4,4,3,3,4,3,3,2,3,5,4,2,2,3,3,3,3,2,4,4,4,5,4,4,4,3,4,4,2,3,5,2};
int emergencyAlertSystem2[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,11,13,10,12,13,16,11,15,16,16,16,19,19,20,24,20,25,27,28,31,38,34,47,51,45,1515,5449,3377,338,381,4006,6074,1567,61,52,53,36,39,33,31,31,24,27,22,22,20,21,20,14,16,18,15,15,13,13,14,12,12,10,16,7,11,13,12,13,7,19,20,21,12,8,48,43,9,8,7,9,6,8,7,7,5,8,10,7,6,7,9,3,9,7,6,8,3,9,5,7,9,4,7,6,4,8,6,4,9,4,6,5,4,9,8,8,7,5,6,4,7,5,8,5,6,7,8,5,5,5,7,4,6,8,5,7,7,7,7,5,5,7,3,7,6,7,3,5,7,3,5,6,5,8,7,6,6,6,5,4,7,3,6,6,2,5,2,5,5,3,5,4,5,7,4,6,4,6,7,5,4,6,3,5,7,3,5,4,3,5,4,5,4,15,7,4,8,6,4,6,5,3,6,3,4,6,6,8,9,5,7,7,8,7,6,5,5,3,7,3,5,7,3,3,5,3,4,4,4,5,6,3,7,5,5,3,6,5,6,6,2,4,3,5,2,5,5,6,7};
int fireTruckHorn[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,13,15,9,12,15,20,201,519,254,62,27,23,37,86,42,326,402,94,21,25,57,646,811,214,419,226,26,51,43,253,845,467,69,108,41,25,19,18,31,48,32,95,59,10,9,4,10,36,20,74,110,28,20,23,20,17,13,12,62,39,15,24,25,17,28,31,69,138,53,38,163,161,52,47,83,340,281,60,81,95,59,84,110,331,515,173,123,87,90,53,74,135,306,271,193,89,50,69,89,83,306,675,636,217,51,45,41,81,47,240,232,96,53,29,25,24,25,47,77,35,31,26,17,32,61,84,167,176,65,45,78,73,69,138,233,266,124,41,24,16,30,55,74,71,103,51,38,25,33,45,49,72,172,145,89,82,35,58,105,66,145,227,119,86,45,42,52,85,78,138,111,54,68,58,54,50,36,67,112,59,104,86,85,104,115,127,216,141,122,133,47,65,80,70,176,263,66,64,59,68,56,49,97,144,124,79,73,44,84,78,61,164,277,97,87,91,87,64,138,158,612,564,103,179,99,161,128,127,45,196,126,126,138,91,82,52,126,293,277,158,131,87,118,98,35,157};
int smokeAlarmNew[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,9,4,5,8,5,4,3,6,5,3,7,9,8,7,6,6,4,5,2,5,2,4,4,4,6,4,8,13,6,5,6,7,10,9,7,4,4,6,3,4,4,5,5,4,5,3,4,4,7,6,3,6,4,5,5,4,4,6,5,4,5,4,6,3,5,4,5,4,6,6,6,6,6,5,3,4,2,6,4,6,4,4,9,8,5,5,7,6,7,8,6,9,5,6,5,10,4,10,8,8,8,10,19,23,8,6,12,9,9,13,13,18,12,20,17,25,18,36,18,22,46,102,1452,4822,2854,108,88,73,14,34,30,22,31,36,25,9,16,9,8,9,6,9,5,7,10,8,8,8,7,7,5,7,3,7,5,6,4,7,5,6,4,5,5,4,4,3,9,4,6,5,2,5,5,5,5,5,4,6,6,4,6,6,6,3,3,5,2,6,7,7,14,7,4,5,4,4,4,6,4,4,6,5,6,4,3,4,2,5,3,4,4,6,3,6,6,5,6,4,5,3,5,3,5,6,6,6,4,6,5,4,4,4,7,5,3,26,27,8,4,4,4,3,3,2,5,5};
int fireAlarmNew[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,26,11,33,30,28,32,42,46,44,28,30,26,20,20,16,13,19,17,11,12,21,12,14,21,26,21,15,33,29,22,21,27,17,16,13,9,10,9,6,8,4,4,6,5,5,5,6,4,4,6,8,7,7,7,5,8,6,7,8,5,4,6,13,7,11,11,9,9,5,8,9,11,17,10,7,22,19,18,22,22,27,13,16,41,35,24,20,41,54,48,71,46,59,83,53,24,54,53,29,51,58,49,15,35,42,34,32,46,38,29,37,36,54,23,37,31,41,28,54,113,97,152,161,258,899,694,825,1421,1228,4343,3363,3552,4730,1532,2041,1773,423,149,1319,4287,3004,361,69,806,2076,1110,643,1326,581,1097,805,447,569,198,729,541,169,259,89,47,55,25,23,18,20,17,16,18,10,7,6,6,6,3,9,8,2,8,11,12,12,17,18,33,24,21,51,32,44,40,25,42,26,55,50,25,72,48,29,38,39,32,19,60,53,24,26,23,51,63,33,51,17,39,32,19,45,24,88,63,50,59,21,20,14,9,8,7,6,4,5,5,4,4,7,2,5,10,6,4,5,3,6,5,6,9};
int tsunamiSiren[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,124,135,122,212,245,636,2213,1716,372,189,256,204,156,152,65,77,92,114,77,161,143,756,1949,2270,507,161,96,137,114,151,120,205,91,130,83,68,119,328,566,1412,620,29,23,19,19,11,13,14,10,13,18,21,107,210,193,488,378,49,22,25,22,18,21,26,27,25,29,46,75,62,62,372,421,125,43,30,35,36,35,32,55,43,57,66,139,60,282,1284,2938,1446,171,185,184,154,189,118,63,41,127,161,277,300,257,217,774,599,108,31,37,17,23,50,24,37,44,71,58,21,42,64,112,152,69,18,14,20,13,23,25,10,29,50,45,34,37,64,103,124,69,24,22,19,11,12,16,21,15,20,33,48,35,27,43,144,157,54,12,10,15,17,19,11,10,18,21,23,20,15,33,84,106,36,13,10,14,7,6,13,16,8,8,8,9,13,13,32,47,29,14,8,13,14,17,9,9,9,5,11,9,8,16,15,61,63,22,7,12,19,15,7,6,19,21,19,24,23,31,40,102,131,63,28,19,27,20,31,32,41,37,32,26,19,20,22,96,130,55,23,8,15,13,26,17,7,9,10};
int nuclearAlarm[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,145,100,156,107,47,41,56,46,68,121,133,69,169,861,4640,3961,461,140,1304,5217,3676,213,71,26,88,42,69,72,60,111,57,45,51,30,37,27,28,52,251,375,102,17,9,19,24,12,7,13,134,147,30,14,10,11,7,8,13,7,9,6,5,7,38,98,56,10,7,13,42,25,5,6,13,28,11,16,24,314,529,168,54,11,14,8,20,33,77,480,552,133,39,20,15,20,5,4,8,6,7,6,5,3,9,7,10,18,145,405,225,24,6,69,148,65,8,6,3,6,5,6,3,6,5,3,4,4,6,5,5,7,4,4,4,1,2,8,21,20,6,7,4,3,5,4,4,5,4,6,7,5,10,12,7,6,5,6,4,2,4,7,20,37,14,7,10,35,39,14,7,4,3,4,4,3,4,2,5,3,2,10,15,9,5,5,10,19,14,6,3,6,14,10,7,7,17,39,16,4,3,6,7,6,4,4,5,5,6,6,17,36,19,4,5,5,7,6,4,4,6,4,4,5,3,34,149,105,10,5,4,5,3,6,8,57,100,42,8,6,21,22,7,2,5,5,4,5};
int policeSiren[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,8,19,8,18,40,28,33,14,16,28,36,40,29,41,49,36,25,20,110,257,106,89,108,95,96,41,47,71,108,177,225,152,137,210,186,311,272,220,610,434,2184,4866,2078,61,15,32,27,18,19,21,17,19,17,25,10,26,22,25,27,16,25,11,18,16,12,18,12,23,32,26,31,43,27,18,25,25,37,29,27,48,57,52,77,75,73,93,66,80,82,139,740,2614,4144,1858,192,53,56,91,57,49,47,37,50,41,29,30,29,32,25,17,18,28,46,30,32,28,19,24,20,27,42,20,10,20,31,26,32,32,29,18,13,22,21,26,29,26,9,27,18,20,43,148,407,537,312,47,22,29,36,25,24,11,13,17,38,45,28,14,18,25,28,20,8,19,21,34,28,11,20,17,24,20,11,18,24,31,25,14,34,27,13,26,32,24,22,34,39,33,42,32,32,24,43,61,42,30,36,25,29,39,34,28,23,27,54,33,26,17,30,28,28,26,23,30,33,39,39,56,52,29,34,41,42,77,95,119,117,85,62,69,104,80,58,60,63,54,52,41,55,49,51,30,30,41,64,98};
int tornadoSiren[SAMPLES/2] = {0,0,0,0,0,0,0,0,0,0,29,30,33,124,71,106,78,155,123,252,269,204,249,165,235,1404,8989,8607,1131,276,181,225,242,169,114,200,117,98,127,77,80,79,162,125,184,134,144,103,102,51,55,79,437,804,375,45,37,23,41,22,36,19,24,20,20,17,27,15,43,22,52,82,112,93,36,59,54,66,140,251,240,133,24,85,99,84,65,82,147,151,114,34,39,58,49,40,87,118,322,432,220,159,269,264,500,2502,3478,2499,827,124,80,116,128,68,60,55,53,81,44,46,21,27,26,52,42,54,49,30,37,56,55,294,489,264,150,21,61,38,66,39,42,54,92,126,81,29,19,32,32,21,35,38,36,34,46,44,51,244,740,626,371,318,107,65,98,66,30,65,148,166,119,39,49,61,56,25,60,40,31,65,58,33,49,104,223,269,168,384,306,71,36,74,62,65,82,135,130,70,39,19,37,23,26,35,30,19,35,23,27,47,73,166,110,26,75,70,51,67,95,62,49,115,172,138,47,56,30,28,61,61,80,62,168,254,186,181,463,1195,1108,439,344,445,189,148,121,73,54,73,118,127,64,45,68,50,26,51};

double micInputSumSquare = 0.00;
int micInput[SAMPLES/2];

const float activeSiren = 0.95;

const int sampleWindow = 50; //Sampling Frequency (50 us = 20kHz)

int thresholdActivate = 40;

unsigned long startMillis;

int mic1Sample;
int mic2Sample;
int mic3Sample;
int mic4Sample;
int mic5Sample;
int mic6Sample;
int mic7Sample;
int mic8Sample;

int signalMax1;
int signalMin1;
int signalMax2;
int signalMin2;
int signalMax3;
int signalMin3;
int signalMax4;
int signalMin4;
int signalMax5;
int signalMin5;
int signalMax6;
int signalMin6;
int signalMax7;
int signalMin7;
int signalMax8;
int signalMin8;

int peakToPeak1;
int peakToPeak2;
int peakToPeak3;
int peakToPeak4;
int peakToPeak5;
int peakToPeak6;
int peakToPeak7;
int peakToPeak8;

int store1 = 0;
int store2 = 0;
int store3 = 0;
int store4 = 0;
int store5 = 0;
int store6 = 0;
int store7 = 0;
int store8 = 0;

float avgpeakToPeak1;
float avgpeakToPeak2;
float avgpeakToPeak3;
float avgpeakToPeak4;
float avgpeakToPeak5;
float avgpeakToPeak6;
float avgpeakToPeak7;
float avgpeakToPeak8;

int z = 0;

int a = 0;
int b = 0;
int c = 0;
int d = 0;
int e = 0;
int f = 0;
int g = 0;
int h = 0;

const int pwm = A19;

void setup() 
{
  Serial.begin(9600);
  //Serial.print("Hello! test");

  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  //Sum of (siren^2)
  for (int i = 0; i < arraySize; i++)
  {
    backUpAlarmSumSquare = backUpAlarmSumSquare + pow(backUpAlarm[i], 2);
    emergencyAlertSystem1SumSquare = emergencyAlertSystem1SumSquare + pow(emergencyAlertSystem1[i], 2);
    emergencyAlertSystem2SumSquare = emergencyAlertSystem2SumSquare + pow(emergencyAlertSystem2[i], 2);
    fireTruckHornSumSquare = fireTruckHornSumSquare + pow(fireTruckHorn[i], 2);
    smokeAlarmNewSumSquare = smokeAlarmNewSumSquare + pow(smokeAlarmNew[i], 2);
    fireAlarmNewSumSquare = fireAlarmNewSumSquare + pow(fireAlarmNew[i], 2);
    tsunamiSirenSumSquare = tsunamiSirenSumSquare + pow(tsunamiSiren[i], 2);
    nuclearAlarmSumSquare = nuclearAlarmSumSquare + pow(nuclearAlarm[i], 2);
    policeSirenSumSquare = policeSirenSumSquare + pow(policeSiren[i], 2);
    tornadoSirenSumSquare = tornadoSirenSumSquare + pow(tornadoSiren[i], 2);
  }

  //Initialize screen
  tft.initR(INITR_BLACKTAB);

  tft.fillScreen(ST7735_BLACK);

  pinMode(pwm,OUTPUT);
}

void loop() 
{
  /*for (int i = 0; i < 10; i++)
  {
    //displayDirection(i);
    displaySiren(i, "Nan");
    delay(5000);
  }*/
  
  directionality();
  correlation();
  //delay(100);
  //vibrationMotor(1000);
}

void directionality()
{
  while (z < 5)
  {
    startMillis= millis();  // Start of sample window
      
    peakToPeak1 = 0;   // peak-to-peak level
    peakToPeak2 = 0;
    peakToPeak3 = 0;
    peakToPeak4 = 0;
    peakToPeak5 = 0;   
    peakToPeak6 = 0;
    peakToPeak7 = 0;
    peakToPeak8 = 0;
  
    signalMax1 = 0;
    signalMin1 = 1023;
    signalMax2 = 0;
    signalMin2 = 1023;
    signalMax3 = 0;
    signalMin3 = 1023;
    signalMax4 = 0;
    signalMin4 = 1023;
    signalMax5 = 0;
    signalMin5 = 1023;
    signalMax6 = 0;
    signalMin6 = 1023;
    signalMax7 = 0;
    signalMin7 = 1023;
    signalMax8 = 0;
    signalMin8 = 1023;

    //Collect data for 50 mS or set sampleWindow
    while (millis() - startMillis < sampleWindow)
    {
      mic1Sample = analogRead(A0);
      mic2Sample = analogRead(A1);
      mic3Sample = analogRead(A2);
      mic4Sample = analogRead(A3);
      mic5Sample = analogRead(A4);
      mic6Sample = analogRead(A5);
      mic7Sample = analogRead(A6);
      mic8Sample = analogRead(A7);
      
      if (mic1Sample < 1023)              //Toss out spurious readings
      {
        if (mic1Sample > signalMax1)
          signalMax1 = mic1Sample;        //Save just the max levels
        else if (mic1Sample < signalMin1)
          signalMin1 = mic1Sample;        //Save just the min levels
      }
  
      if (mic2Sample < 1023)  
      {
        if (mic2Sample > signalMax2)
          signalMax2 = mic2Sample;
        else if (mic2Sample < signalMin2)
          signalMin2 = mic2Sample;  
      }
  
      if (mic3Sample < 1023) 
      {
        if (mic3Sample > signalMax3)
          signalMax3 = mic3Sample; 
        else if (mic3Sample < signalMin3)
          signalMin3 = mic3Sample;  
      }
  
      if (mic4Sample < 1023)
      {
        if (mic4Sample > signalMax4)
          signalMax4 = mic4Sample;  
        else if (mic4Sample < signalMin4)
          signalMin4 = mic4Sample;  
      }
      
      if (mic5Sample < 1023)
      {
        if (mic5Sample > signalMax5)
          signalMax5 = mic5Sample;
        else if (mic5Sample < signalMin5)
          signalMin5 = mic5Sample;
      }
      
      if (mic6Sample < 1023)
      {
        if (mic6Sample > signalMax6)
          signalMax6 = mic6Sample;
        else if (mic6Sample < signalMin6)
          signalMin6 = mic6Sample;
      }
      
      if (mic7Sample < 1023)
      {
        if (mic7Sample > signalMax7)
          signalMax7 = mic7Sample;
        else if (mic7Sample < signalMin7)
          signalMin7 = mic7Sample;
      }
      
      if (mic8Sample < 1023)
      {
        if (mic8Sample > signalMax8)
          signalMax8 = mic8Sample;
        else if (mic8Sample < signalMin8)
          signalMin8 = mic8Sample;
      }
    }
        
    peakToPeak1 = (signalMax1 - signalMin1)*0.96;  // max - min = peak-peak amplitude
    peakToPeak2 = (signalMax2 - signalMin2)*0.99;
    peakToPeak3 = (signalMax3 - signalMin3)*1.06;
    peakToPeak4 = (signalMax4 - signalMin4)*1.04;
    peakToPeak5 = (signalMax5 - signalMin5)*1.02;
    peakToPeak6 = (signalMax6 - signalMin6)*1.02;
    peakToPeak7 = (signalMax7 - signalMin7)*0.95;
    peakToPeak8 = (signalMax8 - signalMin8)*0.95;

    if (peakToPeak1 > thresholdActivate || peakToPeak2 > thresholdActivate || peakToPeak3 > thresholdActivate || peakToPeak4 > thresholdActivate 
    || peakToPeak5 > thresholdActivate || peakToPeak6 > thresholdActivate || peakToPeak7 > thresholdActivate || peakToPeak8 > thresholdActivate)
    {
      if (peakToPeak1 > peakToPeak2 && peakToPeak1 > peakToPeak3 && peakToPeak1 > peakToPeak4 && peakToPeak1 > peakToPeak5 && 
         peakToPeak1 > peakToPeak6 && peakToPeak1 > peakToPeak7 && peakToPeak1 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 1 is the loudest");
        store1 = store1+peakToPeak1;
        a++;
      }
      else if (peakToPeak2 > peakToPeak1 && peakToPeak2 > peakToPeak3 && peakToPeak2 > peakToPeak4 && peakToPeak2 > peakToPeak5 && 
              peakToPeak2 > peakToPeak6 && peakToPeak2 > peakToPeak7 && peakToPeak2 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 2 is the loudest");
        store2 = store2+peakToPeak2;
        b++;
      }
      else if (peakToPeak3 > peakToPeak1 && peakToPeak3 > peakToPeak2 && peakToPeak3 > peakToPeak4 && peakToPeak3 > peakToPeak5 && 
              peakToPeak3 > peakToPeak6 && peakToPeak3 > peakToPeak7 && peakToPeak3 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 3 is the loudest");
        store3 = store3+peakToPeak3;
        c++;
      }
      else if (peakToPeak4 > peakToPeak1 && peakToPeak4 > peakToPeak2 && peakToPeak4 > peakToPeak3 && peakToPeak4 > peakToPeak5 && 
              peakToPeak4 > peakToPeak6 && peakToPeak4 > peakToPeak7 && peakToPeak4 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 4 is the loudest");
        store4 = store4+peakToPeak4;
        d++;
      }
      else if (peakToPeak5 > peakToPeak1 && peakToPeak5 > peakToPeak2 && peakToPeak5 > peakToPeak3 && peakToPeak5 > peakToPeak4 && 
              peakToPeak5 > peakToPeak6 && peakToPeak5 > peakToPeak7 && peakToPeak5 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 5 is the loudest");
        store5 = store5+peakToPeak5;
        e++;
      }
      else if (peakToPeak6 > peakToPeak1 && peakToPeak6 > peakToPeak2 && peakToPeak6 > peakToPeak3 && peakToPeak6 > peakToPeak4 && 
              peakToPeak6 > peakToPeak5 && peakToPeak6 > peakToPeak7 && peakToPeak6 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 6 is the loudest");
        store6 = store6+peakToPeak6;
        f++;
      }
      else if (peakToPeak7 > peakToPeak1 && peakToPeak7 > peakToPeak2 && peakToPeak7 > peakToPeak3 && peakToPeak7 > peakToPeak4 && 
              peakToPeak7 > peakToPeak5 && peakToPeak7 > peakToPeak6 && peakToPeak7 > peakToPeak8)
      {
        //Serial.println("Peak decected: Mic 7 is the loudest");
        store7 = store7+peakToPeak7;
        g++;
      }
      else if (peakToPeak8 > peakToPeak1 && peakToPeak8 > peakToPeak2 && peakToPeak8 > peakToPeak3 && peakToPeak8 > peakToPeak4 && 
              peakToPeak8 > peakToPeak5 && peakToPeak8 > peakToPeak6 && peakToPeak8 > peakToPeak7)
      {
        //Serial.println("Peak decected: Mic 8 is the loudest");
        store8 = store8+peakToPeak8;
        h++;
      }
    }

    z++;
  }

  if (a>0)
    avgpeakToPeak1 = store1/a;
  else
    avgpeakToPeak1 = 0; 
  
  if (b>0)   
    avgpeakToPeak2 = store2/b;
  else 
    avgpeakToPeak2 = 0;  
  
  if (c>0)
    avgpeakToPeak3 = store3/c;
  else
    avgpeakToPeak3 = 0; 
  
  if (d>0)  
    avgpeakToPeak4 = store4/d;
  else 
    avgpeakToPeak4 = 0;   
  
  if (e>0)
    avgpeakToPeak5 = store5/e;
  else
    avgpeakToPeak5 = 0; 
  
  if (f>0)   
    avgpeakToPeak6 = store6/f;
  else 
    avgpeakToPeak6 = 0;  

  if (g>0)
    avgpeakToPeak7 = store7/g;
  else
    avgpeakToPeak7 = 0; 
  
  if (h>0)
    avgpeakToPeak8 = store8/h;
  else 
    avgpeakToPeak8 = 0;   

  /*tft.fillRect(0, 76, 128, 84, ST7735_BLACK);
  tft.setCursor(0, 77);
  tft.println(avgpeakToPeak1);
  tft.println(avgpeakToPeak2);
  tft.println(avgpeakToPeak3);
  tft.println(avgpeakToPeak4);
  tft.println(avgpeakToPeak5);
  tft.println(avgpeakToPeak6);
  tft.println(avgpeakToPeak7);
  tft.println(avgpeakToPeak8);*/
  
  if(avgpeakToPeak1 > avgpeakToPeak2 && avgpeakToPeak1 > avgpeakToPeak3 && avgpeakToPeak1 > avgpeakToPeak4 && avgpeakToPeak1 > avgpeakToPeak5
  && avgpeakToPeak1 > avgpeakToPeak6 && avgpeakToPeak1 > avgpeakToPeak7 && avgpeakToPeak1 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 1 is the loudest");
    displayDirection(0);
  }   
  else if(avgpeakToPeak2 > avgpeakToPeak1 && avgpeakToPeak2 > avgpeakToPeak3 && avgpeakToPeak2 > avgpeakToPeak4 && avgpeakToPeak2 > avgpeakToPeak5 
  && avgpeakToPeak2 > avgpeakToPeak6 && avgpeakToPeak2 > avgpeakToPeak7 && avgpeakToPeak2 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 2 is the loudest");  
    displayDirection(1);
  }
  else if(avgpeakToPeak3 > avgpeakToPeak1 && avgpeakToPeak3 > avgpeakToPeak2 && avgpeakToPeak3 > avgpeakToPeak4 && avgpeakToPeak3 > avgpeakToPeak5 
  && avgpeakToPeak3 > avgpeakToPeak6 && avgpeakToPeak3 > avgpeakToPeak7 && avgpeakToPeak3 > avgpeakToPeak8)
  { 
    //Serial.println("Peak decected: Mic 3 is the loudest");
    displayDirection(2);
  } 
  else if(avgpeakToPeak4 > avgpeakToPeak1 && avgpeakToPeak4 > avgpeakToPeak2 && avgpeakToPeak4 > avgpeakToPeak3 && avgpeakToPeak4 > avgpeakToPeak5 
  && avgpeakToPeak4 > avgpeakToPeak6 && avgpeakToPeak4 > avgpeakToPeak7 && avgpeakToPeak4 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 4 is the loudest");
    displayDirection(3);
  }   
  else if(avgpeakToPeak5 > avgpeakToPeak1 && avgpeakToPeak5 > avgpeakToPeak2 && avgpeakToPeak5 > avgpeakToPeak3 && avgpeakToPeak5 > avgpeakToPeak4 
  && avgpeakToPeak5 > avgpeakToPeak6 && avgpeakToPeak5 > avgpeakToPeak7 && avgpeakToPeak5 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 5 is the loudest");
    displayDirection(4);
  }
  else if(avgpeakToPeak6 > avgpeakToPeak1 && avgpeakToPeak6 > avgpeakToPeak2 && avgpeakToPeak6 > avgpeakToPeak3 && avgpeakToPeak6 > avgpeakToPeak4 
  && avgpeakToPeak6 > avgpeakToPeak5 && avgpeakToPeak6 > avgpeakToPeak7 && avgpeakToPeak6 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 6 is the loudest");
    displayDirection(5); 
  }
  else if(avgpeakToPeak7 > avgpeakToPeak1 && avgpeakToPeak7 > avgpeakToPeak2 && avgpeakToPeak7 > avgpeakToPeak3 && avgpeakToPeak7 > avgpeakToPeak4 
  && avgpeakToPeak7 > avgpeakToPeak5 && avgpeakToPeak7 > avgpeakToPeak6 && avgpeakToPeak7 > avgpeakToPeak8)
  {
    //Serial.println("Peak decected: Mic 7 is the loudest");
    displayDirection(6);
  }
  else if(avgpeakToPeak8 > avgpeakToPeak1 && avgpeakToPeak8 > avgpeakToPeak2 && avgpeakToPeak8 > avgpeakToPeak3 && avgpeakToPeak8 > avgpeakToPeak4 
  && avgpeakToPeak8 > avgpeakToPeak5 && avgpeakToPeak8 > avgpeakToPeak6 && avgpeakToPeak8 > avgpeakToPeak7)
  {
    //Serial.println("Peak decected: Mic 8 is the loudest");
    displayDirection(7);
  } 

  //Reset peakToPeak storage, counter for loop and count of peak
  store1 = 0;
  store2 = 0;
  store3 = 0;
  store4 = 0;
  store5 = 0;
  store6 = 0;
  store7 = 0;
  store8 = 0;
  
  z=0;
    
  a=0;
  b=0;
  c=0;
  d=0;
  e=0;
  f=0;
  g=0;
  h=0;
}

void getFFT()
{ 
  //SAMPLING
  for(int i = 0; i < SAMPLES; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!
   
      vReal[i] = analogRead(A1);
      vImag[i] = 0;
   
      while(micros() < (microseconds + sampling_period_us))
      {}
  }

  //FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  //PRINT RESULTS
  //Serial.println(peak);     //Print out what frequency is the most dominant.

  for(int i = 0; i < (SAMPLES/2); i++)
  {
      //View all these three lines in serial terminal to see which frequencies has which amplitudes
       
      //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
      //Serial.print(" ");
      //Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins

      if (i < 10)
        vReal[i] = 0;
        
      micInput[i] = vReal[i];
  }

  //delay(1000);  //Repeat the process every second OR:
  //while(1);       //Run code once
}

void correlation()
{
  getFFT();
  
  //Sum of (input^2)
  for(int i = 0; i < arraySize; i++)
  {
    micInputSumSquare = micInputSumSquare + pow(micInput[i], 2);
  }

  //Correlation
  double backUpAlarmCorrelation = 0.00;
  double emergencyAlertSystem1Correlation = 0.00;
  double emergencyAlertSystem2Correlation = 0.00;
  double fireTruckHornCorrelation = 0.00;
  double smokeAlarmNewCorrelation = 0.00;
  double fireAlarmNewCorrelation = 0.00;
  double tsunamiSirenCorrelation = 0.00;
  double nuclearAlarmCorrelation = 0.00;
  double policeSirenCorrelation = 0.00;
  double tornadoSirenCorrelation = 0.00;

  for(int i = 0; i < arraySize; i++)
  {
    backUpAlarmCorrelation = backUpAlarmCorrelation + (micInput[i] * backUpAlarm[i]);
    emergencyAlertSystem1Correlation = emergencyAlertSystem1Correlation + (micInput[i] * emergencyAlertSystem1[i]);
    emergencyAlertSystem2Correlation = emergencyAlertSystem2Correlation + (micInput[i] * emergencyAlertSystem2[i]);
    fireTruckHornCorrelation = fireTruckHornCorrelation + (micInput[i] * fireTruckHorn[i]);
    smokeAlarmNewCorrelation = smokeAlarmNewCorrelation + (micInput[i] * smokeAlarmNew[i]);
    fireAlarmNewCorrelation = fireAlarmNewCorrelation + (micInput[i] * fireAlarmNew[i]);
    tsunamiSirenCorrelation = tsunamiSirenCorrelation + (micInput[i] * tsunamiSiren[i]);
    nuclearAlarmCorrelation = nuclearAlarmCorrelation + (micInput[i] * nuclearAlarm[i]);
    policeSirenCorrelation = policeSirenCorrelation + (micInput[i] * policeSiren[i]);
    tornadoSirenCorrelation = tornadoSirenCorrelation + (micInput[i] * tornadoSiren[i]);
  }

  //Normalized correlation
  double backUpAlarmNormCorrelation = backUpAlarmCorrelation / sqrt(backUpAlarmSumSquare * micInputSumSquare);
  double emergencyAlertSystem1NormCorrelation = emergencyAlertSystem1Correlation / sqrt(emergencyAlertSystem1SumSquare * micInputSumSquare);
  double emergencyAlertSystem2NormCorrelation = emergencyAlertSystem2Correlation / sqrt(emergencyAlertSystem2SumSquare * micInputSumSquare);
  double fireTruckHornNormCorrelation = fireTruckHornCorrelation / sqrt(fireTruckHornSumSquare * micInputSumSquare);
  double smokeAlarmNewNormCorrelation = smokeAlarmNewCorrelation / sqrt(smokeAlarmNewSumSquare * micInputSumSquare);
  double fireAlarmNewNormCorrelation = fireAlarmNewCorrelation / sqrt(fireAlarmNewSumSquare * micInputSumSquare);
  double tsunamiSirenNormCorrelation = tsunamiSirenCorrelation / sqrt(tsunamiSirenSumSquare * micInputSumSquare);
  double nuclearAlarmNormCorrelation = nuclearAlarmCorrelation / sqrt(nuclearAlarmSumSquare * micInputSumSquare);
  double policeSirenNormCorrelation = policeSirenCorrelation / sqrt(policeSirenSumSquare * micInputSumSquare);
  double tornadoSirenNormCorrelation = tornadoSirenCorrelation / sqrt(tornadoSirenSumSquare * micInputSumSquare);

  //Print results
  //Serial.print(backUpAlarmCorrelation);
  //Serial.print("   ");
  
  /*Serial.print(backUpAlarmNormCorrelation);
  Serial.print("   ");
  Serial.print(emergencyAlertSystem1NormCorrelation);
  Serial.print("   ");
  Serial.print(emergencyAlertSystem2NormCorrelation);
  Serial.print("   ");
  Serial.print(fireTruckHornNormCorrelation);
  Serial.print("   ");
  Serial.print(smokeAlarmNewNormCorrelation);
  Serial.print("   ");
  Serial.print(fireAlarmNewNormCorrelation);
  Serial.print("   ");
  Serial.print(tsunamiSirenNormCorrelation);
  Serial.print("   ");
  Serial.print(nuclearAlarmNormCorrelation);
  Serial.print("   ");
  Serial.print(policeSirenNormCorrelation);
  Serial.print("   ");
  Serial.println(tornadoSirenNormCorrelation);*/
  
  if (backUpAlarmNormCorrelation >= activeSiren)
    displaySiren(0, "backUpAlarm");
  if (emergencyAlertSystem1NormCorrelation >= 0.65)
    displaySiren(1, "emergencyAlertSystem1");
  if (emergencyAlertSystem2NormCorrelation >= activeSiren)
    displaySiren(2, "emergencyAlertSystem2");
  if (fireTruckHornNormCorrelation >= activeSiren)
    displaySiren(3, "fireTruckHorn");
  if (smokeAlarmNewNormCorrelation >= activeSiren)
    displaySiren(4, "smokeAlarmNew");
  if (fireAlarmNewNormCorrelation >= activeSiren)
    displaySiren(5, "fireAlarmNew");
  if (tsunamiSirenNormCorrelation >= activeSiren)
    displaySiren(6, "tsunamiSiren");
  if (nuclearAlarmNormCorrelation >= activeSiren)
    displaySiren(7, "nuclearAlarm");
  if (policeSirenNormCorrelation >= activeSiren)
    displaySiren(8, "policeSiren");
  if (tornadoSirenNormCorrelation >= activeSiren)
    displaySiren(9, "tornadoSiren");

  micInputSumSquare = 0;
}

//Display a arrow that point to a direction
//0 - Northwest
//1 - North
//2 - Northeast
//3 - East
//4 - Southeast
//5 - South
//6 - Southwest
//7 - West
void displayDirection(int directionCoordinate)
{
  tft.fillRect(34, 5, 55, 55, ST7735_BLACK);

  switch (directionCoordinate)
  {
    case 0:
      //Northwest
      tft.fillTriangle(87, 26, 55, 58, 39, 10, ST7735_WHITE);
      tft.fillTriangle(86, 27, 56, 57, 61, 32, ST7735_BLACK);
      break;
    case 1:
      //North
      tft.fillTriangle(39, 54, 83, 54, 61, 10, ST7735_WHITE);
      tft.fillTriangle(40, 54, 82, 54, 61, 40, ST7735_BLACK);
      break;
    case 2:
      //Northeast
      tft.fillTriangle(36, 26, 68, 58, 83, 10, ST7735_WHITE);
      tft.fillTriangle(37, 27, 67, 57, 61, 32, ST7735_BLACK);
      break;
    case 3:
      //East
      tft.fillTriangle(39, 10, 39, 54, 83, 32, ST7735_WHITE);
      tft.fillTriangle(39, 11, 38, 54, 53, 32, ST7735_BLACK);
      break;
    case 4:
      //Southeast
      tft.fillTriangle(35, 39, 67, 7, 83, 54, ST7735_WHITE);
      tft.fillTriangle(36, 38, 66, 8, 61, 32, ST7735_BLACK);
      break;
    case 5:
      //South
      tft.fillTriangle(39, 10, 83, 10, 61, 54, ST7735_WHITE);
      tft.fillTriangle(40, 10, 82, 10, 61, 24, ST7735_BLACK);
      break;
    case 6:
      //Southwest
      tft.fillTriangle(87, 39, 55, 7, 39, 54, ST7735_WHITE);
      tft.fillTriangle(86, 38, 56, 8, 61, 32, ST7735_BLACK);
      break;
    case 7:
      //West
      tft.fillTriangle(83, 10, 83, 54, 39, 32, ST7735_WHITE);
      tft.fillTriangle(83, 11, 83, 53, 69, 32, ST7735_BLACK);
      break;
    default:
      tft.setCursor(39, 10);
      //tft.setTextColor(ST7735_RED);
      tft.setTextSize(2);
      tft.print("No direction input");
      break;
  }

  vibrationMotor(600);
}

void displaySiren(int siren, String inputString)
{
  tft.fillRect(0, 76, 128, 84, ST7735_BLACK);

  //tft.setCursor(0, 77);
  tft.setTextSize(2);

  /*String str1 =  "Adafruit";
  String str2 = "Industries";
  //tft.setRotation(3);
  tft.setCursor(7, 77);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  //tft.setTextWrap(true);
  tft.print(str1);
  tft.setCursor(0, 98);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  //tft.setTextWrap(true);
  tft.print(str2);*/

  /*backUpAlarmSumSquare = backUpAlarmSumSquare + pow(backUpAlarm[i], 2);
    emergencyAlertSystem1SumSquare = emergencyAlertSystem1SumSquare + pow(emergencyAlertSystem1[i], 2);
    emergencyAlertSystem2SumSquare = emergencyAlertSystem2SumSquare + pow(emergencyAlertSystem2[i], 2);
    fireTruckHornSumSquare = fireTruckHornSumSquare + pow(fireTruckHorn[i], 2);
    smokeAlarmNewSumSquare = smokeAlarmNewSumSquare + pow(smokeAlarmNew[i], 2);
    fireAlarmNewSumSquare = fireAlarmNewSumSquare + pow(fireAlarmNew[i], 2);
    tsunamiSirenSumSquare = tsunamiSirenSumSquare + pow(tsunamiSiren[i], 2);
    nuclearAlarmSumSquare = nuclearAlarmSumSquare + pow(nuclearAlarm[i], 2);
    policeSirenSumSquare = policeSirenSumSquare + pow(policeSiren[i], 2);
    tornadoSirenSumSquare = tornadoSirenSumSquare + pow(tornadoSiren[i], 2);*/
  
  switch (siren)
  {
    case 0:
      tft.setCursor(0, 77);
      tft.print(String("Industrial"));
      tft.setCursor(0, 95);
      tft.print(String("Back Up"));
      tft.setCursor(0, 113);
      tft.print(String("Alarm"));
      break;
    case 1:
      tft.setCursor(0, 77);
      tft.print(String("National"));
      tft.setCursor(0, 95);
      tft.print(String("Emergency"));
      tft.setCursor(0, 113);
      tft.print(String("Alert"));
      break;
    case 2:
      tft.setCursor(0, 77);
      tft.print(String("National"));
      tft.setCursor(0, 95);
      tft.print(String("Emergency"));
      tft.setCursor(0, 113);
      tft.print(String("Alert"));
      break;
    case 3:
      tft.setCursor(0, 77);
      tft.print(String("Fire"));
      tft.setCursor(0, 95);
      tft.print(String("Truck"));
      tft.setCursor(0, 113);
      tft.print(String("Horn"));
      break;
    case 4:
      tft.setCursor(0, 77);
      tft.print(String("Smoke"));
      tft.setCursor(0, 95);
      tft.print(String("Alarm"));
      break;
    case 5:
      tft.setCursor(0, 77);
      tft.print(String("Home"));
      tft.setCursor(0, 95);
      tft.print(String("Fire"));
      tft.setCursor(0, 113);
      tft.print(String("Alarm"));
      break;
    case 6:
      tft.setCursor(0, 77);
      tft.print(String("Tsunami"));
      tft.setCursor(0, 95);
      tft.print(String("Siren"));
      break;
    case 7:
      tft.setCursor(0, 77);
      tft.print(String("Nuclear"));
      tft.setCursor(0, 95);
      tft.print(String("Alarm"));
      break;
    case 8:
      tft.setCursor(0, 77);
      tft.print(String("Police"));
      tft.setCursor(0, 95);
      tft.print(String("Siren"));
      break;
    case 9:
      tft.setCursor(0, 77);
      tft.print(String("Tornado"));
      tft.setCursor(0, 95);
      tft.print(String("Siren"));
      break;
    default:
      
      break;
  }

  delay(2000);
  tft.fillRect(0, 76, 128, 84, ST7735_BLACK);
}

void vibrationMotor(int motorRunTime)
{
  //pinMode(pwm,OUTPUT);
  /*pinMode(20,OUTPUT);
  pinMode(21,OUTPUT);*/

  digitalWrite(pwm,HIGH);
  delay(motorRunTime);
  digitalWrite(pwm,LOW);
}
