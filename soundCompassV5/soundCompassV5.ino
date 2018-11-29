//Needed for FFT
#include "arduinoFFT.h"
 
#define SAMPLES 512             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC
 
arduinoFFT FFT = arduinoFFT();

unsigned long timeStart;

unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

//getFFT for directionality2
double vRealMic1[SAMPLES];
double vRealMic2[SAMPLES];
double vRealMic3[SAMPLES];
double vRealMic4[SAMPLES];
double vRealMic5[SAMPLES];
double vRealMic6[SAMPLES];
double vRealMic7[SAMPLES];
double vRealMic8[SAMPLES];

double vImagMic1[SAMPLES];
double vImagMic2[SAMPLES];
double vImagMic3[SAMPLES];
double vImagMic4[SAMPLES];
double vImagMic5[SAMPLES];
double vImagMic6[SAMPLES];
double vImagMic7[SAMPLES];
double vImagMic8[SAMPLES];
 
//Need for screen display
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS  10
#define TFT_RST 9
#define TFT_DC  8 

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define TFT_SCLK 13
#define TFT_MOSI 11

//Used in correlation
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
double fireTruckWailSumSquare = 0.00;
double tornadoSirenSumSquare = 0.00;
double policeYlpSumSquare = 0.00;

int backUpAlarm[arraySize] = {0,0,0,0,0,0,0,0,0,0,88,36,199,216,158,170,105,140,168,167,164,321,263,102,232,214,163,214,181,218,108,38,77,63,60,52,84,113,72,80,94,87,83,79,50,38,162,1290,1413,243,13,8,18,11,12,10,14,5,14,7,10,14,12,8,17,10,13,7,8,13,9,6,10,11,14,10,10,9,9,6,9,15,23,17,16,21,15,13,24,27,24,29,34,30,339,966,527,90,85,92,92,70,39,18,35,29,30,33,31,17,17,8,14,14,11,13,20,16,9,16,10,11,8,8,9,14,13,11,11,13,12,14,16,7,7,7,4,5,6,3,9,9,76,99,19,7,8,6,4,2,4,4,4,6,5,6,5,4,6,5,7,7,9,7,4,8,9,10,6,9,9,7,5,6,6,8,8,9,12,13,7,7,9,6,4,7,6,10,9,7,11,12,11,9,8,9,9,9,3,10,22,10,5,7,8,6,5,6,4,5,13,13,6,4,8,6,7,9,14,15,10,11,11,5,4,11,11,6,12,7,13,17,17,15,25,24,24,411,657,185,39,24,14,8,7,19,14,14,15,6,6,9,9,8,6,4};
int emergencyAlertSystem1[arraySize] = {0,0,0,0,0,0,0,0,0,0,9,4,5,3,7,4,6,4,4,5,6,5,6,6,3,3,5,4,6,8,5,7,4,6,6,7,6,3,6,6,7,5,6,5,6,4,3,3,3,3,4,6,6,8,8,8,4,4,4,3,9,7,5,3,4,5,11,5,3,6,4,4,9,7,6,9,8,6,6,5,5,6,2,4,2,3,3,6,7,14,6,4,8,5,5,4,5,4,3,4,6,5,3,5,4,4,4,2,3,4,4,5,4,4,3,4,4,3,3,4,4,6,5,6,3,4,5,4,4,5,4,5,5,5,5,3,4,4,5,4,5,3,3,2,4,3,2,4,3,4,2,4,5,4,3,4,4,3,6,7,3,3,5,4,4,6,5,2,4,3,3,2,3,4,4,4,3,3,2,3,4,3,3,3,3,4,4,3,4,5,6,3,5,5,1,3,5,4,4,6,13,7,5,5,4,3,4,3,2,3,4,5,5,3,4,4,5,5,3,3,3,3,3,3,4,4,3,3,4,3,3,2,3,5,4,2,2,3,3,3,3,2,4,4,4,5,4,4,4,3,4,4,2,3,5,2};
int emergencyAlertSystem2[arraySize] = {0,0,0,0,0,0,0,0,0,0,11,13,10,12,13,16,11,15,16,16,16,19,19,20,24,20,25,27,28,31,38,34,47,51,45,1515,5449,3377,338,381,4006,6074,1567,61,52,53,36,39,33,31,31,24,27,22,22,20,21,20,14,16,18,15,15,13,13,14,12,12,10,16,7,11,13,12,13,7,19,20,21,12,8,48,43,9,8,7,9,6,8,7,7,5,8,10,7,6,7,9,3,9,7,6,8,3,9,5,7,9,4,7,6,4,8,6,4,9,4,6,5,4,9,8,8,7,5,6,4,7,5,8,5,6,7,8,5,5,5,7,4,6,8,5,7,7,7,7,5,5,7,3,7,6,7,3,5,7,3,5,6,5,8,7,6,6,6,5,4,7,3,6,6,2,5,2,5,5,3,5,4,5,7,4,6,4,6,7,5,4,6,3,5,7,3,5,4,3,5,4,5,4,15,7,4,8,6,4,6,5,3,6,3,4,6,6,8,9,5,7,7,8,7,6,5,5,3,7,3,5,7,3,3,5,3,4,4,4,5,6,3,7,5,5,3,6,5,6,6,2,4,3,5,2,5,5,6,7};
int fireTruckHorn[arraySize] = {0,0,0,0,0,0,0,0,0,0,13,15,9,12,15,20,201,519,254,62,27,23,37,86,42,326,402,94,21,25,57,646,811,214,419,226,26,51,43,253,845,467,69,108,41,25,19,18,31,48,32,95,59,10,9,4,10,36,20,74,110,28,20,23,20,17,13,12,62,39,15,24,25,17,28,31,69,138,53,38,163,161,52,47,83,340,281,60,81,95,59,84,110,331,515,173,123,87,90,53,74,135,306,271,193,89,50,69,89,83,306,675,636,217,51,45,41,81,47,240,232,96,53,29,25,24,25,47,77,35,31,26,17,32,61,84,167,176,65,45,78,73,69,138,233,266,124,41,24,16,30,55,74,71,103,51,38,25,33,45,49,72,172,145,89,82,35,58,105,66,145,227,119,86,45,42,52,85,78,138,111,54,68,58,54,50,36,67,112,59,104,86,85,104,115,127,216,141,122,133,47,65,80,70,176,263,66,64,59,68,56,49,97,144,124,79,73,44,84,78,61,164,277,97,87,91,87,64,138,158,612,564,103,179,99,161,128,127,45,196,126,126,138,91,82,52,126,293,277,158,131,87,118,98,35,157};
int smokeAlarmNew[arraySize] = {0,0,0,0,0,0,0,0,0,0,9,4,5,8,5,4,3,6,5,3,7,9,8,7,6,6,4,5,2,5,2,4,4,4,6,4,8,13,6,5,6,7,10,9,7,4,4,6,3,4,4,5,5,4,5,3,4,4,7,6,3,6,4,5,5,4,4,6,5,4,5,4,6,3,5,4,5,4,6,6,6,6,6,5,3,4,2,6,4,6,4,4,9,8,5,5,7,6,7,8,6,9,5,6,5,10,4,10,8,8,8,10,19,23,8,6,12,9,9,13,13,18,12,20,17,25,18,36,18,22,46,102,1452,4822,2854,108,88,73,14,34,30,22,31,36,25,9,16,9,8,9,6,9,5,7,10,8,8,8,7,7,5,7,3,7,5,6,4,7,5,6,4,5,5,4,4,3,9,4,6,5,2,5,5,5,5,5,4,6,6,4,6,6,6,3,3,5,2,6,7,7,14,7,4,5,4,4,4,6,4,4,6,5,6,4,3,4,2,5,3,4,4,6,3,6,6,5,6,4,5,3,5,3,5,6,6,6,4,6,5,4,4,4,7,5,3,26,27,8,4,4,4,3,3,2,5,5};
int fireAlarmNew[arraySize] = {0,0,0,0,0,0,0,0,0,0,26,11,33,30,28,32,42,46,44,28,30,26,20,20,16,13,19,17,11,12,21,12,14,21,26,21,15,33,29,22,21,27,17,16,13,9,10,9,6,8,4,4,6,5,5,5,6,4,4,6,8,7,7,7,5,8,6,7,8,5,4,6,13,7,11,11,9,9,5,8,9,11,17,10,7,22,19,18,22,22,27,13,16,41,35,24,20,41,54,48,71,46,59,83,53,24,54,53,29,51,58,49,15,35,42,34,32,46,38,29,37,36,54,23,37,31,41,28,54,113,97,152,161,258,899,694,825,1421,1228,4343,3363,3552,4730,1532,2041,1773,423,149,1319,4287,3004,361,69,806,2076,1110,643,1326,581,1097,805,447,569,198,729,541,169,259,89,47,55,25,23,18,20,17,16,18,10,7,6,6,6,3,9,8,2,8,11,12,12,17,18,33,24,21,51,32,44,40,25,42,26,55,50,25,72,48,29,38,39,32,19,60,53,24,26,23,51,63,33,51,17,39,32,19,45,24,88,63,50,59,21,20,14,9,8,7,6,4,5,5,4,4,7,2,5,10,6,4,5,3,6,5,6,9};
int tsunamiSiren[arraySize] = {0,0,0,0,0,0,0,0,0,0,124,135,122,212,245,636,2213,1716,372,189,256,204,156,152,65,77,92,114,77,161,143,756,1949,2270,507,161,96,137,114,151,120,205,91,130,83,68,119,328,566,1412,620,29,23,19,19,11,13,14,10,13,18,21,107,210,193,488,378,49,22,25,22,18,21,26,27,25,29,46,75,62,62,372,421,125,43,30,35,36,35,32,55,43,57,66,139,60,282,1284,2938,1446,171,185,184,154,189,118,63,41,127,161,277,300,257,217,774,599,108,31,37,17,23,50,24,37,44,71,58,21,42,64,112,152,69,18,14,20,13,23,25,10,29,50,45,34,37,64,103,124,69,24,22,19,11,12,16,21,15,20,33,48,35,27,43,144,157,54,12,10,15,17,19,11,10,18,21,23,20,15,33,84,106,36,13,10,14,7,6,13,16,8,8,8,9,13,13,32,47,29,14,8,13,14,17,9,9,9,5,11,9,8,16,15,61,63,22,7,12,19,15,7,6,19,21,19,24,23,31,40,102,131,63,28,19,27,20,31,32,41,37,32,26,19,20,22,96,130,55,23,8,15,13,26,17,7,9,10};
int nuclearAlarm[arraySize] = {0,0,0,0,0,0,0,0,0,0,145,100,156,107,47,41,56,46,68,121,133,69,169,861,4640,3961,461,140,1304,5217,3676,213,71,26,88,42,69,72,60,111,57,45,51,30,37,27,28,52,251,375,102,17,9,19,24,12,7,13,134,147,30,14,10,11,7,8,13,7,9,6,5,7,38,98,56,10,7,13,42,25,5,6,13,28,11,16,24,314,529,168,54,11,14,8,20,33,77,480,552,133,39,20,15,20,5,4,8,6,7,6,5,3,9,7,10,18,145,405,225,24,6,69,148,65,8,6,3,6,5,6,3,6,5,3,4,4,6,5,5,7,4,4,4,1,2,8,21,20,6,7,4,3,5,4,4,5,4,6,7,5,10,12,7,6,5,6,4,2,4,7,20,37,14,7,10,35,39,14,7,4,3,4,4,3,4,2,5,3,2,10,15,9,5,5,10,19,14,6,3,6,14,10,7,7,17,39,16,4,3,6,7,6,4,4,5,5,6,6,17,36,19,4,5,5,7,6,4,4,6,4,4,5,3,34,149,105,10,5,4,5,3,6,8,57,100,42,8,6,21,22,7,2,5,5,4,5};
int policeSiren[arraySize] = {0,0,0,0,0,0,0,0,0,0,8,19,8,18,40,28,33,14,16,28,36,40,29,41,49,36,25,20,110,257,106,89,108,95,96,41,47,71,108,177,225,152,137,210,186,311,272,220,610,434,2184,4866,2078,61,15,32,27,18,19,21,17,19,17,25,10,26,22,25,27,16,25,11,18,16,12,18,12,23,32,26,31,43,27,18,25,25,37,29,27,48,57,52,77,75,73,93,66,80,82,139,740,2614,4144,1858,192,53,56,91,57,49,47,37,50,41,29,30,29,32,25,17,18,28,46,30,32,28,19,24,20,27,42,20,10,20,31,26,32,32,29,18,13,22,21,26,29,26,9,27,18,20,43,148,407,537,312,47,22,29,36,25,24,11,13,17,38,45,28,14,18,25,28,20,8,19,21,34,28,11,20,17,24,20,11,18,24,31,25,14,34,27,13,26,32,24,22,34,39,33,42,32,32,24,43,61,42,30,36,25,29,39,34,28,23,27,54,33,26,17,30,28,28,26,23,30,33,39,39,56,52,29,34,41,42,77,95,119,117,85,62,69,104,80,58,60,63,54,52,41,55,49,51,30,30,41,64,98};
int fireTruckWail[arraySize] = {0,0,0,0,0,0,0,0,0,0,110,93,105,198,228,269,144,118,187,160,168,178,205,246,205,313,426,6727,31957,31427,6052,400,436,290,385,149,179,109,142,103,119,122,213,310,137,97,81,92,57,48,36,60,44,37,28,220,1456,2415,1347,283,64,41,53,46,33,33,35,23,41,23,34,42,40,35,35,33,38,38,33,104,121,93,143,252,2097,5092,3933,2295,622,225,224,193,186,90,86,40,43,79,141,35,105,380,655,552,506,461,212,178,272,248,132,469,1630,6320,5700,4208,3712,644,189,110,122,87,39,38,35,31,43,71,50,32,21,28,18,32,20,25,21,27,40,112,271,655,470,158,535,499,163,101,62,66,24,18,17,29,36,147,128,29,51,169,103,35,37,31,32,61,35,105,120,848,1834,1222,339,972,763,148,35,52,52,17,107,52,25,73,135,91,57,159,147,74,23,33,65,285,884,396,163,210,579,228,622,1132,1170,632,158,135,66,36,22,34,44,62,128,132,126,39,72,90,61,23,57,41,207,470,366,820,1471,1636,907,312,408,670,530,389,322,166,291,332,75,242,594,533,292,122,186,196,247,210,104,84,67,1076,2319,1127,1444,2062};
int tornadoSiren[arraySize] = {0,0,0,0,0,0,0,0,0,0,29,30,33,124,71,106,78,155,123,252,269,204,249,165,235,1404,8989,8607,1131,276,181,225,242,169,114,200,117,98,127,77,80,79,162,125,184,134,144,103,102,51,55,79,437,804,375,45,37,23,41,22,36,19,24,20,20,17,27,15,43,22,52,82,112,93,36,59,54,66,140,251,240,133,24,85,99,84,65,82,147,151,114,34,39,58,49,40,87,118,322,432,220,159,269,264,500,2502,3478,2499,827,124,80,116,128,68,60,55,53,81,44,46,21,27,26,52,42,54,49,30,37,56,55,294,489,264,150,21,61,38,66,39,42,54,92,126,81,29,19,32,32,21,35,38,36,34,46,44,51,244,740,626,371,318,107,65,98,66,30,65,148,166,119,39,49,61,56,25,60,40,31,65,58,33,49,104,223,269,168,384,306,71,36,74,62,65,82,135,130,70,39,19,37,23,26,35,30,19,35,23,27,47,73,166,110,26,75,70,51,67,95,62,49,115,172,138,47,56,30,28,61,61,80,62,168,254,186,181,463,1195,1108,439,344,445,189,148,121,73,54,73,118,127,64,45,68,50,26,51};
int policeYlp[arraySize] = {0,0,0,0,0,0,0,0,0,0,101,63,38,46,30,78,93,73,72,66,85,55,26,62,84,81,38,26,40,53,67,53,31,36,47,39,32,32,45,33,13,18,12,7,7,5,10,7,7,6,4,3,3,4,6,8,4,4,5,4,5,4,4,4,3,6,6,4,4,5,4,2,4,4,6,4,3,5,7,8,4,3,3,2,5,4,2,4,5,4,5,5,4,3,3,4,4,5,4,4,4,3,3,4,3,3,4,3,3,2,3,5,4,4,5,5,4,3,3,4,4,5,6,6,5,4,4,5,4,4,4,4,3,4,6,5,3,4,3,6,5,4,3,3,4,4,5,4,4,3,7,6,3,3,5,7,6,8,9,5,5,7,7,7,7,5,5,5,5,3,4,5,4,6,6,5,4,3,3,2,4,4,5,5,3,4,2,4,4,5,7,5,2,3,3,4,4,4,3,4,10,7,5,4,4,5,5,5,4,3,3,3,4,6,6,6,4,6,7,5,3,2,4,5,6,5,3,4,3,3,4,4,4,6,5,4,5,4,3,4,2,3,9,11,4,3,4,4,4,6,5,3,3,4,4,5};

double micInputSumSquare = 0.00;
int micInput[arraySize];

const float activeSiren = 0.90;

const int sampleWindow = 50; //Sampling Frequency (50 us = 20kHz)

int thresholdActivate = 0;

int thresholdActivateSumMethod = 0;

int thresholdActivateSumFFTMethod = 0;

unsigned long startMillis;

//Vibration motor
const int pwm = A13;

//Used in checkMmic
const int upperBound = 600;
const int lowerBound = 400;

//Analog connected to the MEM mics
const int mic1 = A0;
const int mic2 = A1;
const int mic3 = A2;
const int mic4 = A3;
const int mic5 = A4;
const int mic6 = A5;
const int mic7 = A6;
const int mic8 = A7;

//The FFT mic 
const int micFFT = mic2; 

void setup() 
{
  Serial.begin(9600);

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
    fireTruckWailSumSquare = fireTruckWailSumSquare + pow(fireTruckWail[i], 2);
    tornadoSirenSumSquare = tornadoSirenSumSquare + pow(tornadoSiren[i], 2);
    policeYlpSumSquare = policeYlpSumSquare + pow(policeYlp[i], 2);    
  }

  //Initialize screen
  tft.initR(INITR_BLACKTAB);

  tft.fillScreen(ST7735_BLACK);

  pinMode(pwm,OUTPUT);

  bool micsFail = true;
  
  Serial.println("Initialize Teensy succesful");

  while(micsFail)
  {
    micsFail = checkMics(micsFail);
  }
  
  Serial.println("Microphones check successful");
  Serial.println("Starting program");
}

void loop() 
{/*
  for (int i = 0; i < 9; i++)
  {
    displayDirection("10", i);
    delay(500);
  }

  for (int i = 0; i < 4; i++)
  {
    displaySiren("siren1", i, "str1", "str2", "str3");
    delay(500);
    displaySiren("siren2", i, "str1", "str2", "str3");
    delay(500);
  }
  */

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
    for(int i = 0; i < 1; i++)
      {
        directionality();
        //delay(44);
      }
  }
  
  //correlation();
  
  //delay(100);
  //vibrationMotor(600);
}

void directionality() //TimeTest
{
  int samplingTime = 867;

  double ppkTheshold = 30;

  int z = 0;

  double avgSum1 = 0;
  double avgSum2 = 0;
  double avgSum3 = 0;
  double avgSum4 = 0;
  double avgSum5 = 0;
  double avgSum6 = 0;
  double avgSum7 = 0;
  double avgSum8 = 0;
  
  int store1 = 0;
  int store2 = 0;
  int store3 = 0;
  int store4 = 0;
  int store5 = 0;
  int store6 = 0;
  int store7 = 0;
  int store8 = 0;
  
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  int e = 0;
  int f = 0;
  int g = 0;
  int h = 0;

  double micsSum[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  double mic1FirstPpkAndSum[2] = {0, 0}; 
  double mic2FirstPpkAndSum[2] = {0, 0};   
  double mic3FirstPpkAndSum[2] = {0, 0};
  double mic4FirstPpkAndSum[2] = {0, 0};
  double mic5FirstPpkAndSum[2] = {0, 0};
  double mic6FirstPpkAndSum[2] = {0, 0};
  double mic7FirstPpkAndSum[2] = {0, 0};
  double mic8FirstPpkAndSum[2] = {0, 0};   

  //Peak-to-peak level
  double mic1ppk[samplingTime]; 
  double mic2ppk[samplingTime];   
  double mic3ppk[samplingTime];
  double mic4ppk[samplingTime];
  double mic5ppk[samplingTime];
  double mic6ppk[samplingTime];
  double mic7ppk[samplingTime];
  double mic8ppk[samplingTime];
    
  while (z < 5)
  {
    mic1FirstPpkAndSum[1] = 0; 
    mic2FirstPpkAndSum[1] = 0;   
    mic3FirstPpkAndSum[1] = 0;
    mic4FirstPpkAndSum[1] = 0;
    mic5FirstPpkAndSum[1] = 0;
    mic6FirstPpkAndSum[1] = 0;
    mic7FirstPpkAndSum[1] = 0;
    mic8FirstPpkAndSum[1] = 0; 
  
    double signalMax1 = 0;
    double signalMin1 = 1023;
    double signalMax2 = 0;
    double signalMin2 = 1023;
    double signalMax3 = 0;
    double signalMin3 = 1023;
    double signalMax4 = 0;
    double signalMin4 = 1023;
    double signalMax5 = 0;
    double signalMin5 = 1023;
    double signalMax6 = 0;
    double signalMin6 = 1023;
    double signalMax7 = 0;
    double signalMin7 = 1023;
    double signalMax8 = 0;
    double signalMin8 = 1023;

    //Collect data for 50 mS or set sampleWindow
    for (int i = 0; i < samplingTime; i++)
    {  
      double micsAnalog[8] = {analogRead(mic1)*0.9994, analogRead(mic2)*0.9990, analogRead(mic3), analogRead(mic4)*0.9989, 
                            analogRead(mic5)*0.9944, analogRead(mic6)*0.9988, analogRead(mic7)*1.0016, analogRead(mic8)*0.9967};
      /*double micsAnalog[8] = {analogRead(mic1), analogRead(mic2), analogRead(mic3), analogRead(mic4), 
                            analogRead(mic5), analogRead(mic6), analogRead(mic7), analogRead(mic8)}; analogRead(mic3)*1.0114*/

      /*if (micsAnalog[0] < 0)
        micsAnalog[0] = 0.0;
      if (micsAnalog[1] < 0)
        micsAnalog[1] = 0.0;
      if (micsAnalog[2] < 0)
        micsAnalog[2] = 0.0;
      if (micsAnalog[3] < 0)
        micsAnalog[3] = 0.0;
      if (micsAnalog[4] < 0)
        micsAnalog[4] = 0.0;
      if (micsAnalog[5] < 0)
        micsAnalog[5] = 0.0;
      if (micsAnalog[6] < 0)
        micsAnalog[6] = 0.0;
      if (micsAnalog[7] < 0)
        micsAnalog[7] = 0.0;*/

      if (micsAnalog[0] < 1023)                       //Toss out spurious readings
      {
        if (micsAnalog[0] > signalMax1)
          signalMax1 = micsAnalog[0];                 //Save just the max levels
        else if (micsAnalog[0] < signalMin1)
          signalMin1 = micsAnalog[0];                 //Save just the min levels

        //Get peak to peak  
        mic1ppk[i] = (signalMax1 - signalMin1);       //max - min = peak-peak amplitude

        if (mic1ppk[i] < ppkTheshold)
          mic1ppk[i] = 0;
      }
  
      if (micsAnalog[1] < 1023)  
      {
        if (micsAnalog[1] > signalMax2)
          signalMax2 = micsAnalog[1];
        else if (micsAnalog[1] < signalMin2)
          signalMin2 = micsAnalog[1];
        
        mic2ppk[i] = (signalMax2 - signalMin2);

        if (mic2ppk[i] < ppkTheshold)
          mic2ppk[i] = 0;
      }
  
      if (micsAnalog[2] < 1023) 
      {
        if (micsAnalog[2] > signalMax3)
          signalMax3 = micsAnalog[2]; 
        else if (micsAnalog[2] < signalMin3)
          signalMin3 = micsAnalog[2];
        
        mic3ppk[i] = (signalMax3 - signalMin3); 
        
        if (mic3ppk[i] < ppkTheshold)
          mic3ppk[i] = 0; 
      }
  
      if (micsAnalog[3] < 1023)
      {
        if (micsAnalog[3] > signalMax4)
          signalMax4 = micsAnalog[3];  
        else if (micsAnalog[3] < signalMin4)
          signalMin4 = micsAnalog[3];
            
        mic4ppk[i] = (signalMax4 - signalMin4); 

        if (mic4ppk[i] < ppkTheshold)
          mic4ppk[i] = 0;
      }
      
      if (micsAnalog[4] < 1023)
      {
        if (micsAnalog[4] > signalMax5)
          signalMax5 = micsAnalog[4];
        else if (micsAnalog[4] < signalMin5)
          signalMin5 = micsAnalog[4];
          
        mic5ppk[i] = (signalMax5 - signalMin5); 

        if (mic5ppk[i] < ppkTheshold)
          mic5ppk[i] = 0;
      }
      
      if (micsAnalog[5] < 1023)
      {
        if (micsAnalog[5] > signalMax6)
          signalMax6 = micsAnalog[5];
        else if (micsAnalog[5] < signalMin6)
          signalMin6 = micsAnalog[5];
          
        mic6ppk[i] = (signalMax6 - signalMin6);

        if (mic6ppk[i] < ppkTheshold)
          mic6ppk[i] = 0;
      }
      
      if (micsAnalog[6] < 1023)
      {
        if (micsAnalog[6] > signalMax7)
          signalMax7 = micsAnalog[6];
        else if (micsAnalog[6] < signalMin7)
          signalMin7 = micsAnalog[6];
        
        mic7ppk[i] = (signalMax7 - signalMin7);

        if (mic7ppk[i] < ppkTheshold)
          mic7ppk[i] = 0;
      }
      
      if (micsAnalog[7] < 1023)
      {
        if (micsAnalog[7] > signalMax8)
          signalMax8 = micsAnalog[7];
        else if (micsAnalog[7] < signalMin8)
          signalMin8 = micsAnalog[7];
          
        mic8ppk[i] = (signalMax8 - signalMin8);

        if (mic8ppk[i] < ppkTheshold)
          mic8ppk[i] = 0;
      }

      //Get sum of analog
      mic1FirstPpkAndSum[1] = mic1FirstPpkAndSum[1] + pow(micsAnalog[0], 2);
      mic2FirstPpkAndSum[1] = mic2FirstPpkAndSum[1] + pow(micsAnalog[1], 2);
      mic3FirstPpkAndSum[1] = mic3FirstPpkAndSum[1] + pow(micsAnalog[2], 2);
      mic4FirstPpkAndSum[1] = mic4FirstPpkAndSum[1] + pow(micsAnalog[3], 2);
      mic5FirstPpkAndSum[1] = mic5FirstPpkAndSum[1] + pow(micsAnalog[4], 2);
      mic6FirstPpkAndSum[1] = mic6FirstPpkAndSum[1] + pow(micsAnalog[5], 2);
      mic7FirstPpkAndSum[1] = mic7FirstPpkAndSum[1] + pow(micsAnalog[6], 2);
      mic8FirstPpkAndSum[1] = mic8FirstPpkAndSum[1] + pow(micsAnalog[7], 2);

      /*Serial.print(i);
      Serial.print("   ");
      Serial.print(mic1ppk[i]);
      Serial.print("   ");
      Serial.print(mic2ppk[i]);
      Serial.print("   ");
      Serial.print(mic3ppk[i]);
      Serial.print("   ");
      Serial.print(mic4ppk[i]);
      Serial.print("   ");
      Serial.print(mic5ppk[i]);
      Serial.print("   ");
      Serial.print(mic6ppk[i]);
      Serial.print("   ");
      Serial.print(mic7ppk[i]);
      Serial.print("   ");
      Serial.print(mic8ppk[i]);
      Serial.print("   ");  
      Serial.print("   ");
      Serial.print(mic1FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic2FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic3FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic4FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic5FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic6FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.print(mic7FirstPpkAndSum[1]);
      Serial.print("   ");
      Serial.println(mic8FirstPpkAndSum[1]);*/
    }

    for(int i = 0; i < samplingTime; i++)
    {       
      if (((mic1ppk[i] > 0) || (mic2ppk[i] > 0) || (mic3ppk[i] > 0) || (mic4ppk[i] > 0) || (mic5ppk[i] > 0) || (mic6ppk[i] > 0) || (mic7ppk[i] > 0) || (mic8ppk[i] > 0)))
      {
        if (mic1ppk[i] > 0)
        {
          mic1FirstPpkAndSum[0] = mic1ppk[i];
          
          micsSum[0] = mic1FirstPpkAndSum[1];
        }
        else
        {
          mic1FirstPpkAndSum[0] = 0;
          
          micsSum[0] = 0;
        }
        
        if (mic2ppk[i] > 0)
        {
          mic2FirstPpkAndSum[0] = mic2ppk[i];

          micsSum[1] = mic2FirstPpkAndSum[1];
        }
        else
        {
          mic2FirstPpkAndSum[0] = 0;
          
          micsSum[1] = 0;
        }
        
        if (mic3ppk[i] > 0)
        {
          mic3FirstPpkAndSum[0] = mic3ppk[i];

          micsSum[2] = mic3FirstPpkAndSum[1];
        }
        else
        {
          mic3FirstPpkAndSum[0] = 0;
          
          micsSum[2] = 0;
        }
        
        if (mic4ppk[i] > 0)
        {
          mic4FirstPpkAndSum[0] = mic4ppk[i];

          micsSum[3] = mic4FirstPpkAndSum[1];
        }
        else
        {
          mic4FirstPpkAndSum[0] = 0;
          
          micsSum[3] = 0;
        }
        
        if (mic5ppk[i] > 0)
        {
          mic5FirstPpkAndSum[0] = mic5ppk[i];

          micsSum[4] = mic5FirstPpkAndSum[1];
        }
        else
        {
          mic5FirstPpkAndSum[0] = 0;
          
          micsSum[4] = 0;
        }
        
        if (mic6ppk[i] > 0)
        {
          mic6FirstPpkAndSum[0] = mic6ppk[i];

          micsSum[5] = mic6FirstPpkAndSum[1];
        }
        else
        {
          mic6FirstPpkAndSum[0] = 0;
          
          micsSum[5] = 0;
        }
        
        if (mic7ppk[i] > 0)
        {
          mic7FirstPpkAndSum[0] = mic7ppk[i];

          micsSum[6] = mic7FirstPpkAndSum[1];
        }
        else
        {
          mic7FirstPpkAndSum[0] = 0;
          
          micsSum[6] = 0;
        }
        
        if (mic8ppk[i] > 0)
        {
          mic8FirstPpkAndSum[0] = mic8ppk[i];

          micsSum[7] = mic8FirstPpkAndSum[1];
        }
        else
        {
          mic8FirstPpkAndSum[0] = 0;
          
          micsSum[7] = 0;
        }

        break;
      }
    }
    
    //Values
    /*Serial.print("m1");
    Serial.print("\t");
    Serial.print(mic1FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic1FirstPpkAndSum[1]);
    Serial.print("\t");
    
    Serial.print("m2");
    Serial.print("\t");
    Serial.print(mic2FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic2FirstPpkAndSum[1]);
    Serial.print("\t");
    
    Serial.print("m3");
    Serial.print("\t");
    Serial.print(mic3FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic3FirstPpkAndSum[1]);
    Serial.print("\t");
    
    Serial.print("m4");
    Serial.print("\t");
    Serial.print(mic4FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic4FirstPpkAndSum[1]);
    Serial.print("\t");
    
    Serial.print("m5");
    Serial.print("\t");
    Serial.print(mic5FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic5FirstPpkAndSum[1]);
    Serial.print("\t");
    
    Serial.print("m6");
    Serial.print("\t");
    Serial.print(mic6FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic6FirstPpkAndSum[1]);
    Serial.print("\t");

    Serial.print("m7");
    Serial.print("\t");
    Serial.print(mic7FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.print(mic7FirstPpkAndSum[1]);
    Serial.print("\t");

    Serial.print("m8");
    Serial.print("\t");
    Serial.print(mic8FirstPpkAndSum[0]);
    Serial.print("\t");
    Serial.println(mic8FirstPpkAndSum[1]);*/

    
    //Active when above threshold
    //If active find the highest sound
    if (micsSum[0] > thresholdActivate || micsSum[1] > thresholdActivate || micsSum[2] > thresholdActivate || micsSum[3] > thresholdActivate 
    || micsSum[4] > thresholdActivate || micsSum[5] > thresholdActivate || micsSum[6] > thresholdActivate || micsSum[7] > thresholdActivate)
    {
      int maxVal = 0;
      int posMic  = 0;

      if(micsSum[0] + micsSum[1] + micsSum[2] > maxVal)
      {
        maxVal = micsSum[0] + micsSum[1] + micsSum[2];
        posMic = 2;
      }
      if(micsSum[1] + micsSum[2] + micsSum[3] > maxVal)
      {
        maxVal = micsSum[1] + micsSum[2] + micsSum[3];
        posMic = 3;
      }
      if(micsSum[2] + micsSum[3] + micsSum[4] > maxVal)
      {
        maxVal = micsSum[2] + micsSum[3] + micsSum[4];
        posMic = 4;
      }
      if(micsSum[3] + micsSum[4] + micsSum[5] > maxVal)
      {
        maxVal = micsSum[3] + micsSum[4] + micsSum[5];
        posMic = 5;
      }
      if(micsSum[4] + micsSum[5] + micsSum[6] > maxVal)
      {
        maxVal = micsSum[4] + micsSum[5] + micsSum[6];
        posMic = 6;
      }
      if(micsSum[5] + micsSum[6] + micsSum[7] > maxVal)
      {
        maxVal = micsSum[5] + micsSum[6] + micsSum[7];
        posMic = 7;
      }
      if(micsSum[6] + micsSum[7] + micsSum[0] > maxVal)
      {
        maxVal = micsSum[6] + micsSum[7] + micsSum[0];
        posMic = 8;
      }
      if(micsSum[7] + micsSum[0] + micsSum[1] > maxVal)
      {
        maxVal = micsSum[7] + micsSum[0] + micsSum[1];
        posMic = 1;
      }

      Serial.println(posMic);
      
      if (posMic == 2)
      {
        store1 = store1 + micsSum[0] + micsSum[1] + micsSum[2];
        //a++;
        a = 1;
        //store1 = store1+a;
      }
      else if (posMic == 3)
      {
        store2 = store2 + micsSum[1] + micsSum[2] + micsSum[3];
        //b++;
        b = 1;
        //store2 = store2+b;
      }
      else if (posMic == 4)
      {
        store3 = store3 + micsSum[2] + micsSum[3] + micsSum[4];
        //c++;
        c = 1;
        //store3 = store3+c;
      }
      else if (posMic == 5)
      {
        store4 = store4 + micsSum[3] + micsSum[4] + micsSum[5];
        //d++;
        d = 1;
        //store4 = store4+d;
      }
      else if (posMic == 6)
      {
        store5 = store5 + micsSum[4] + micsSum[5] + micsSum[6];
        //e++;
        e = 1;
        //store5 = store5+e;
      }
      else if (posMic == 7)
      {
        store6 = store6 + micsSum[5] + micsSum[6] + micsSum[7];
        //f++;
        f = 1;
        //store6 = store6+f;
      }
      else if (posMic == 8)
      {
        store7 = store7 + micsSum[6] + micsSum[7] + micsSum[0];
        //g++;
        g = 1;
        //store7 = store7+g;
      }
      else if (posMic == 1)
      {
        store8 = store8 + micsSum[7] + micsSum[0] + micsSum[1];
        //h++;
        h = 1;
        //store8 = store8+h;
      }
    }


    /*Serial.print(store1);
    Serial.print("  ");
    Serial.print(store2);
    Serial.print("  ");
    Serial.print(store3);
    Serial.print("  ");
    Serial.print(store4);
    Serial.print("  ");
    Serial.print(store5);
    Serial.print("  ");
    Serial.print(store6);
    Serial.print("  ");
    Serial.print(store7);
    Serial.print("  ");
    Serial.println(store8);*/
    
    z++;
  }

  /*Serial.print(a);
  Serial.print("   ");
  Serial.print(b);
  Serial.print("   ");
  Serial.print(c);
  Serial.print("   ");
  Serial.print(d);
  Serial.print("   ");
  Serial.print(e);
  Serial.print("   ");
  Serial.print(f);
  Serial.print("   ");
  Serial.print(g);
  Serial.print("   ");
  Serial.println(h);*/

  //Get the average of how often the loudest sound come from what mic
  if (a>0)
    avgSum1 = store1/a;
  else
    avgSum1 = 0; 
  
  if (b>0)   
    avgSum2 = store2/b;
  else 
    avgSum2 = 0;  
  
  if (c>0)
    avgSum3 = store3/c;
  else
    avgSum3 = 0; 
  
  if (d>0)  
    avgSum4 = store4/d;
  else 
    avgSum4 = 0;   
  
  if (e>0)
    avgSum5 = store5/e;
  else
    avgSum5 = 0; 
  
  if (f>0)   
    avgSum6 = store6/f;
  else 
    avgSum6 = 0;  

  if (g>0)
    avgSum7 = store7/g;
  else
    avgSum7 = 0; 
  
  if (h>0)
    avgSum8 = store8/h;
  else 
    avgSum8 = 0;

  //For debugging
  //Print to screen
  /*tft.fillRect(0, 76, 128, 84, ST7735_BLACK);
  tft.setCursor(0, 77);
  tft.println(avgSum1);
  tft.println(avgSum2);
  tft.println(avgSum3);
  tft.println(avgSum4);
  tft.println(avgSum5);
  tft.println(avgSum6);
  tft.println(avgSum7);
  tft.println(avgSum8);*/

  //Print to console
  /*Serial.print(avgSum1);
  Serial.print("   ");
  Serial.print(avgSum2);
  Serial.print("   ");
  Serial.print(avgSum3);
  Serial.print("   ");
  Serial.print(avgSum4);
  Serial.print("   ");
  Serial.print(avgSum5);
  Serial.print("   ");
  Serial.print(avgSum6);
  Serial.print("   ");
  Serial.print(avgSum7);
  Serial.print("   ");
  Serial.println(avgSum8);*/

  if((avgSum1 > avgSum2) && (avgSum1 > avgSum3) && (avgSum1 > avgSum4) && (avgSum1 > avgSum5)
  && (avgSum1 > avgSum6) && (avgSum1 > avgSum7) && (avgSum1 > avgSum8))
  {
    displayDirection("2", 1);
  }   
  else if((avgSum2 > avgSum1) && (avgSum2 > avgSum3) && (avgSum2 > avgSum4) && (avgSum2 > avgSum5) 
       && (avgSum2 > avgSum6) && (avgSum2 > avgSum7) && (avgSum2 > avgSum8))
  {
    displayDirection("3", 2);
  }
  else if((avgSum3 > avgSum1) && (avgSum3 > avgSum2) && (avgSum3 > avgSum4) && (avgSum3 > avgSum5) 
       && (avgSum3 > avgSum6) && (avgSum3 > avgSum7) && (avgSum3 > avgSum8))
  { 
    displayDirection("4", 3);
  } 
  else if((avgSum4 > avgSum1) && (avgSum4 > avgSum2) && (avgSum4 > avgSum3) && (avgSum4 > avgSum5) 
       && (avgSum4 > avgSum6) && (avgSum4 > avgSum7) && (avgSum4 > avgSum8))
  {
    displayDirection("5", 4);
  }   
  else if((avgSum5 > avgSum1) && (avgSum5 > avgSum2) && (avgSum5 > avgSum3) && (avgSum5 > avgSum4) 
       && (avgSum5 > avgSum6) && (avgSum5 > avgSum7) && (avgSum5 > avgSum8))
  {
    displayDirection("6", 5);
  }
  else if((avgSum6 > avgSum1) && (avgSum6 > avgSum2) && (avgSum6 > avgSum3) && (avgSum6 > avgSum4) 
       && (avgSum6 > avgSum5) && (avgSum6 > avgSum7) && (avgSum6 > avgSum8))
  {
    displayDirection("7", 6); 
  }
  else if((avgSum7 > avgSum1) && (avgSum7 > avgSum2) && (avgSum7 > avgSum3) && (avgSum7 > avgSum4)
       && (avgSum7 > avgSum5) && (avgSum7 > avgSum6) && (avgSum7 > avgSum8))
  {
    displayDirection("8", 7);
  }
  else if((avgSum8 > avgSum1) && (avgSum8 > avgSum2) && (avgSum8 > avgSum3) && (avgSum8 > avgSum4) 
       && (avgSum8 > avgSum5) && (avgSum8 > avgSum6) && (avgSum8 > avgSum7))
  {
    displayDirection("1", 0);
  }
  else
  {
    Serial.println("No output, below threshold");
    tft.fillScreen(ST7735_BLACK);
  }
}

void getFFT()
{ 
  //SAMPLING
  for(int i = 0; i < SAMPLES; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!
   
      vReal[i] = analogRead(micFFT);
      vImag[i] = 0;
   
      while(micros() < (microseconds + sampling_period_us))
      {}
  }

  //FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

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
}

void correlation()
{
  getFFT();
  
  //Sum of (input^2)
  for(int i = 0; i < arraySize; i++)
  {
    micInputSumSquare = micInputSumSquare + pow(micInput[i], 2);
  }

  //Correlation initialize
  double backUpAlarmCorrelation = 0.00;
  double emergencyAlertSystem1Correlation = 0.00;
  double emergencyAlertSystem2Correlation = 0.00;
  double fireTruckHornCorrelation = 0.00;
  double smokeAlarmNewCorrelation = 0.00;
  double fireAlarmNewCorrelation = 0.00;
  double tsunamiSirenCorrelation = 0.00;
  double nuclearAlarmCorrelation = 0.00;
  double policeSirenCorrelation = 0.00;
  double fireTruckWailCorrelation = 0.00;
  double tornadoSirenCorrelation = 0.00;
  double policeYlpCorrelation = 0.00;

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
    fireTruckWailCorrelation = fireTruckWailCorrelation + (micInput[i] * fireTruckWail[i]);
    tornadoSirenCorrelation = tornadoSirenCorrelation + (micInput[i] * tornadoSiren[i]);
    policeYlpCorrelation = policeYlpCorrelation + (micInput[i] * policeYlp[i]);
  }

  //Initialize the normalized correlation
  double backUpAlarmNormCorrelation = backUpAlarmCorrelation / sqrt(backUpAlarmSumSquare * micInputSumSquare);
  double emergencyAlertSystem1NormCorrelation = emergencyAlertSystem1Correlation / sqrt(emergencyAlertSystem1SumSquare * micInputSumSquare);
  double emergencyAlertSystem2NormCorrelation = emergencyAlertSystem2Correlation / sqrt(emergencyAlertSystem2SumSquare * micInputSumSquare);
  double fireTruckHornNormCorrelation = fireTruckHornCorrelation / sqrt(fireTruckHornSumSquare * micInputSumSquare);
  double smokeAlarmNewNormCorrelation = smokeAlarmNewCorrelation / sqrt(smokeAlarmNewSumSquare * micInputSumSquare);
  double fireAlarmNewNormCorrelation = fireAlarmNewCorrelation / sqrt(fireAlarmNewSumSquare * micInputSumSquare);
  double tsunamiSirenNormCorrelation = tsunamiSirenCorrelation / sqrt(tsunamiSirenSumSquare * micInputSumSquare);
  double nuclearAlarmNormCorrelation = nuclearAlarmCorrelation / sqrt(nuclearAlarmSumSquare * micInputSumSquare);
  double policeSirenNormCorrelation = policeSirenCorrelation / sqrt(policeSirenSumSquare * micInputSumSquare);
  double fireTruckWailNormCorrelation = fireTruckWailCorrelation / sqrt(fireTruckWailSumSquare * micInputSumSquare);
  double tornadoSirenNormCorrelation = tornadoSirenCorrelation / sqrt(tornadoSirenSumSquare * micInputSumSquare);
  double policeYlpNormCorrelation = policeYlpCorrelation / sqrt(policeYlpSumSquare * micInputSumSquare);

  //For debugging
  //Print results of correlation
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
  Serial.print(fireTruckWailNormCorrelation);
  Serial.print("   ");
  Serial.print(tornadoSirenNormCorrelation);
  Serial.print("   ");
  Serial.println(policeYlpNormCorrelation);*/
  
  //Checks against correlation threshold if true print out the siren
  if (backUpAlarmNormCorrelation >= activeSiren)
    displaySiren("backUpAlarm", 3, "Industrial", "Back Up", "Alarm");
  if (emergencyAlertSystem1NormCorrelation >= activeSiren)
    displaySiren("emergencyAlertSystem1", 3, "National", "Emergency", "Alert");
  if (emergencyAlertSystem2NormCorrelation >= activeSiren)
    displaySiren("emergencyAlertSystem2", 3, "National", "Emergency", "Alert");
  if (fireTruckHornNormCorrelation >= activeSiren)
    displaySiren("fireTruckHorn", 3, "Fire", "Truck", "Horn");
  if (smokeAlarmNewNormCorrelation >= activeSiren)
    displaySiren("smokeAlarmNew", 2, "Smoke", "Alarm", "No_text_here");
  if (fireAlarmNewNormCorrelation >= activeSiren)
    displaySiren("fireAlarmNew", 3, "Home", "Fire", "Alarm");
  if (tsunamiSirenNormCorrelation >= activeSiren)
    displaySiren("tsunamiSiren", 2, "Tsunami", "Siren", "No_text_here");
  if (nuclearAlarmNormCorrelation >= activeSiren)
    displaySiren("nuclearAlarm", 2, "Nuclear", "Alarm", "No_text_here");
  if (policeSirenNormCorrelation >= activeSiren)
    displaySiren("policeSiren", 2, "Police", "Siren", "No_text_here");
  if (fireTruckWailNormCorrelation >= activeSiren)
    displaySiren("fireTruckWail", 3, "Fire", "Truck", "Wail");
  if (tornadoSirenNormCorrelation >= activeSiren)
    displaySiren("tornadoSiren", 2, "Tornado", "Siren", "No_text_here");
  if (policeYlpNormCorrelation >= activeSiren)
    displaySiren("policeYlp", 2, "Police", "Yelp", "No_text_here");

  //Reset
  micInputSumSquare = 0;
}

//Display a arrow that point to a direction
//The argument directionCoordinate defined where the arrow point
//on the screen
//0 - Northwest
//1 - North
//2 - Northeast
//3 - East
//4 - Southeast
//5 - South
//6 - Southwest
//7 - West
void displayDirection(String mic, int directionCoordinate)
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
      Serial.println("Wrong input for the direction");
      tft.setCursor(0, 0);
      //tft.setTextColor(ST7735_RED);
      tft.setTextSize(2);
      tft.print("No direction input");
      break;
  }

  //For debugging
  Serial.print("Mic ");
  Serial.print(mic);
  Serial.println(" is the loudest");

  //Run the vibration motor
  vibrationMotor(600);
}

//Display the siren name and print to the screen
//inputString is the name of the siren and display on the consol
//numwords is how many word need to be display on the screen
//str1 hole the first word, str2 hold the second word, and str3 hold the third word
void displaySiren(String inputString, int numWords, String str1, String str2, String str3)
{
  tft.fillRect(0, 76, 128, 84, ST7735_BLACK);

  tft.setTextSize(2);
  
  switch (numWords)
  {  
    case 2:
      tft.setCursor(0, 77);
      tft.print(str1);
      tft.setCursor(0, 95);
      tft.print(str2);
      break;
    case 3:
      tft.setCursor(0, 77);
      tft.print(str1);
      tft.setCursor(0, 95);
      tft.print(str2);
      tft.setCursor(0, 113);
      tft.print(str3);
      break;
    default:
      Serial.println("Wrong input for the siren");
      tft.setCursor(0, 77);
      //tft.setTextColor(ST7735_RED);
      tft.setTextSize(2);
      tft.print("No siren input");
      break;
  }

  //For debugging
  Serial.println(inputString);
}

//Turn the viration motor on and off
//The time the vibration motor stay on is
//depend on the motorRunTime
void vibrationMotor(int motorRunTime)
{
  digitalWrite(pwm,HIGH);
  delay(motorRunTime);
  digitalWrite(pwm,LOW);
}

//Check the mics to see if they are connected
bool checkMics(bool micsFail)
{
  //Console debug
  Serial.println("Checking microphones...");

  //Screen debug
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println(String("Cheking Teensy"));
  tft.println(String("Cheking screen"));
  tft.println(String("Cheking microphone..."));
  
  bool mic1Fail = true;
  bool mic2Fail = true;
  bool mic3Fail = true;
  bool mic4Fail = true;
  bool mic5Fail = true;
  bool mic6Fail = true;
  bool mic7Fail = true;
  bool mic8Fail = true;
  
  for (int i = 0; i < 10; i++)
  {
    if (analogRead(mic1) > upperBound || analogRead(mic1) < lowerBound)
    {
      mic1Fail = true;
      Serial.println("Mic 1 error");
    }
    else
      mic1Fail = false;

    if (analogRead(mic2) > upperBound || analogRead(mic2) < lowerBound)
    {
      mic2Fail = true;
      Serial.println("Mic 2 error");
    }
    else
      mic2Fail = false;

    if (analogRead(mic3) > upperBound || analogRead(mic3) < lowerBound)
    {
      mic3Fail = true;
      Serial.println("Mic 3 error");
    }
    else
      mic3Fail = false;

    if (analogRead(mic4) > upperBound || analogRead(mic4) < lowerBound)
    {
      mic4Fail = true;
      Serial.println("Mic 4 error");
    }
    else
      mic4Fail = false;

    if (analogRead(mic5) > upperBound || analogRead(mic5) < lowerBound)
    {
      mic5Fail = true;
      Serial.println("Mic 5 error");
    }
    else
      mic5Fail = false;

    if (analogRead(mic6) > upperBound || analogRead(mic6) < lowerBound)
    {
      mic6Fail = true;
      Serial.println("Mic 6 error");
    }
    else
      mic6Fail = false;

    if (analogRead(mic7) > upperBound || analogRead(mic7) < lowerBound)
    {
      mic7Fail = true;
      Serial.println("Mic 7 error");
    }
    else
      mic7Fail = false;

    if (analogRead(mic8) > upperBound || analogRead(mic8) < lowerBound)
    {
      mic8Fail = true;
      Serial.println("Mic 8 error");
    }
    else
      mic8Fail = false;
  
    delay(100);
  }

  if (mic1Fail || mic2Fail || mic3Fail || mic4Fail || mic5Fail || mic6Fail || mic7Fail || mic8Fail)
  {
    micsFail = true;

    Serial.println("Microphones initialize fail");

    //Display the error sign
    displayError();
    
    delay(10000);
  }
  else
  {
    micsFail = false;

    //Serial.println("Microphone initialize successful");

    tft.println(String("Microphones check successful"));
    tft.println(String("Starting program"));
  }
  
  tft.fillScreen(ST7735_BLACK);

  return micsFail;
}

//Display the error sign
void displayError()
{
  tft.fillScreen(ST7735_BLACK);
  
  tft.fillTriangle(9, 130, 117, 130, 63, 10, ST7735_YELLOW);
  tft.fillTriangle(53, 50, 73, 50, 63, 100, ST7735_BLACK);
  tft.fillCircle(63, 112, 4, ST7735_BLACK);

  tft.setTextSize(2);
  tft.setCursor(34, 135);
  tft.print("ERROR");
}
