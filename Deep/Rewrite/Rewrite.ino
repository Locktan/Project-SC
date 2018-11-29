#include "arduinoFFT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
 
#define SAMPLES 512             //Must be a power of 2
#define SAMPLINGFREQUENCY 12000 //Hz, must be less than 10000 due to ADC
#define TFT_CS  10
#define TFT_RST 9
#define TFT_DC  8 
#define TFT_SCLK 13
#define TFT_MOSI 11

arduinoFFT FFT = arduinoFFT();
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


unsigned long timeStart;
unsigned int samplingPeriod;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

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

double avgSum1;
double avgSum2;
double avgSum3;
double avgSum4;
double avgSum5;
double avgSum6;
double avgSum7;
double avgSum8;

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

const int pwm = A13;

const int mic1 = A0;
const int mic2 = A1;
const int mic3 = A2;
const int mic4 = A3;
const int mic5 = A4;
const int mic6 = A5;
const int mic7 = A6;
const int mic8 = A7;

const int sampleTime = 2000;
double micsAnalog[9][sampleTime] = {0};

//function prototypes
int findMin(double[][sampleTime], int, int);
bool checkMics(bool micsFail);


void setup()
{
    Serial.begin(115200);
    samplingPeriod = round(1000000*(1.0/SAMPLINGFREQUENCY));
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

tft.initR(INITR_BLACKTAB);
tft.fillScreen(ST7735_BLACK);
pinMode(pwm,OUTPUT);
bool micsFail = true;
Serial.println("Initialize Teensy succesful");
/*while(micsFail)
  {
    micsFail = checkMics(micsFail);
  }
  */
Serial.println("Microphones check successful");
Serial.println("Starting program");
}

void loop() 
{
int incomingChar = 'y';
if (incomingChar == 'n')
{
    if (Serial.available() > 0)
    {
      incomingChar = Serial.read();
    }
  }
  if (incomingChar == 'y')
  {
    for(int i = 0; i < 1; i++)
      {
        directionality();
      }
  }
  //correlation();
}

void directionality()
{
    //sampleTime = 867;
    
    avgSum1 = 0;
    avgSum2 = 0;
    avgSum3 = 0;
    avgSum4 = 0;
    avgSum5 = 0;
    avgSum6 = 0;
    avgSum7 = 0;
    avgSum8 = 0;    

    //double micsSum[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int powerValue[9] = {0};
    int powerValueAvg[9] = {0};
    int micsAnalogAvg[9] = {0};

    
      for(int j = 0; j < 100; j++)
      {
        micsAnalogAvg[1] += analogRead(mic1);
        micsAnalogAvg[2] += analogRead(mic2);
        micsAnalogAvg[3] += analogRead(mic3);
        micsAnalogAvg[4] += analogRead(mic4);
        micsAnalogAvg[5] += analogRead(mic5);
        micsAnalogAvg[6] += analogRead(mic6);
        micsAnalogAvg[7] += analogRead(mic7);
        micsAnalogAvg[8] += analogRead(mic8);
      }

      micsAnalogAvg[1] = micsAnalogAvg[1]/500;
      micsAnalogAvg[2] = micsAnalogAvg[2]/500;
      micsAnalogAvg[3] = micsAnalogAvg[3]/500;
      micsAnalogAvg[4] = micsAnalogAvg[4]/500;
      micsAnalogAvg[5] = micsAnalogAvg[5]/500;
      micsAnalogAvg[6] = micsAnalogAvg[6]/500;
      micsAnalogAvg[7] = micsAnalogAvg[7]/500;
      micsAnalogAvg[8] = micsAnalogAvg[8]/500;
      
    
    
    
    //for all points in sampleTime: read analog, bias correct, cut amplitude, ^3, and add that value to powerVariable
    for (int s = 0; s < sampleTime; s++)
    {  
        //get analog data from all mics
        micsAnalog[1][s] = analogRead(mic1);
        micsAnalog[2][s] = analogRead(mic2);
        micsAnalog[3][s] = analogRead(mic3);
        micsAnalog[4][s] = analogRead(mic4);
        micsAnalog[5][s] = analogRead(mic5);
        micsAnalog[6][s] = analogRead(mic6);
        micsAnalog[7][s] = analogRead(mic7);
        micsAnalog[8][s] = analogRead(mic8);        
        
        //take that data then bias correct it
        micsAnalog[1][s] = micsAnalog[1][s]*0.9994;
        micsAnalog[2][s] = micsAnalog[2][s]*0.9990;
        micsAnalog[3][s] = micsAnalog[3][s];
        micsAnalog[4][s] = micsAnalog[4][s]*0.9989;
        micsAnalog[5][s] = micsAnalog[5][s]*0.9944; 
        micsAnalog[6][s] = micsAnalog[6][s]*0.9988;
        micsAnalog[7][s] = micsAnalog[7][s]*1.0016;
        micsAnalog[8][s] = micsAnalog[8][s]*0.9967;

   

        //take the bias corrected and cut amplitude and abs and clear noise
        for(int i  = 1; i < 9; i++)
        {
            double temp = micsAnalog[i][s]; 
            micsAnalog[i][s] = micsAnalog[i][s] - micsAnalogAvg[i]; //cut amplitude
            micsAnalog[i][s] = abs(micsAnalog[i][s]); //abs

            //Serial.println(micsAnalog[i][s]);
            if(micsAnalog[i][s] < 4) //clear noise
            {
             // micsAnalog[i][s] = 0;
            }
        }
        
        //take the lower amp values and ^3 them
        for(int m = 1; m < 9; m++)
        {
            micsAnalog[m][s] = pow(micsAnalog[m][s], 3);
            powerValue[m] += micsAnalog[m][s];
        }

        //done finding the powervalue for each mic m up to point s in sampleTime
    }



    
    for(int i = 1; i < 9; i++)
    {
        powerValueAvg[i] = powerValue[i]/sampleTime;
        //Serial.println(powerValueAvg[i]);
    }

    int position = 0; //micx = posx
    int maxVal = 0;

    if(powerValueAvg[1] + powerValueAvg[2] > maxVal)
    {
      maxVal = powerValueAvg[1] + powerValueAvg[2];
      position = 1;
    }
    if(powerValueAvg[2] + powerValueAvg[3] > maxVal)
    {
      maxVal = powerValueAvg[2] + powerValueAvg[3];
      position = 2;
    }
    if(powerValueAvg[3] + powerValueAvg[4] > maxVal)
    {
      maxVal = powerValueAvg[3] + powerValueAvg[4];
      position = 3;
    }
    if(powerValueAvg[4] + powerValueAvg[5] > maxVal)
    {
      maxVal = powerValueAvg[4] + powerValueAvg[5];
      position = 4;
    }
    if(powerValueAvg[5] + powerValueAvg[6] > maxVal)
    {
      maxVal = powerValueAvg[5] + powerValueAvg[6];
      position = 5;
    }
    if(powerValueAvg[6] + powerValueAvg[7] > maxVal)
    {
      maxVal = powerValueAvg[6] + powerValueAvg[7];
      position = 6;
    }
    if(powerValueAvg[7] + powerValueAvg[8] > maxVal)
    {
      maxVal = powerValueAvg[7] + powerValueAvg[8];
      position = 7;
    }
    if(powerValueAvg[8] + powerValueAvg[1] > maxVal)
    {
      maxVal = powerValueAvg[8] + powerValueAvg[1];
      position = 8;
    }
    




   /*if(powerValueAvg[8] + powerValueAvg[1] + powerValueAvg[2] > maxVal)
    {
      maxVal = powerValueAvg[8] + powerValueAvg[1] + powerValueAvg[2] ;
      position = 1;
    }
    if(powerValueAvg[1] + powerValueAvg[2] + powerValueAvg[3] > maxVal)
    {
      maxVal = powerValueAvg[1] + powerValueAvg[2] + powerValueAvg[3] ;
      position = 2;
    }
    if(powerValueAvg[2] + powerValueAvg[3] + powerValueAvg[4] > maxVal)
    {
      maxVal = powerValueAvg[2] + powerValueAvg[3] + powerValueAvg[4];
      position = 3;
    }
    if(powerValueAvg[3] + powerValueAvg[4] + powerValueAvg[5] > maxVal)
    {
       maxVal = powerValueAvg[3] + powerValueAvg[4] + powerValueAvg[5];
      position = 4;
    }
    if(powerValueAvg[4] + powerValueAvg[5] + powerValueAvg[6] > maxVal)
    {
       maxVal = powerValueAvg[4] + powerValueAvg[5] + powerValueAvg[6];
      position = 5;
    }
    if(powerValueAvg[5] + powerValueAvg[6] + powerValueAvg[7] > maxVal)
    {
       maxVal = powerValueAvg[5] + powerValueAvg[6] + powerValueAvg[7];
      position = 6;
    }
    if(powerValueAvg[6] + powerValueAvg[7] + powerValueAvg[8] > maxVal)
    {
       maxVal = powerValueAvg[6] + powerValueAvg[7] + powerValueAvg[8];
      position = 7;
    }
    if(powerValueAvg[7] + powerValueAvg[8] + powerValueAvg[1] > maxVal)
    {
       maxVal = powerValueAvg[7] + powerValueAvg[8] + powerValueAvg[1];
      position = 8;
    }
  */

    Serial.println(position);

    
    //displayDirection("nan", position);

}

/*
    //find minvalue in arr[1 -> size][sampTime]
double findMin(double inputArr[][sampleTime], int size, int sampTime)
   {
       int minValue = 9999999;
       for(int i = 0; i < size; i++)
       {
           if(inputArr[i][sampTime] < minValue)
           {
               minValue = inputArr[i][sampTime];
           }
       }
        return minValue;
   }
*/
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

  int upperBound = 600;
  int lowerBound = 400;
  
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
    //displayError();
    
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

void outputInPlotter1d(int input[], int size)
{
  for(int i = 1; i<size - 1; i++)
  {
    Serial.print(input[i]);
    Serial.print(" ");
  }
  Serial.println(input[size]);
  
}






    

    


    

    
