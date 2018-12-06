//Needed for FFT
#include "arduinoFFT.h"
 
#define SAMPLES 512             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC
 
arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

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

double backUpAlarmNNSumSquare = 0.00;
double backUpAlarmWNSumSquare = 0.00;
double emergencyAlertSystem1SumSquare = 0.00;
double emergencyAlertSystem2SumSquare = 0.00;

double fireTruckHornSumSquare = 0.00;

double smokeAlarmSumSquare = 0.00;
double fireAlarmSumSquare = 0.00;
double tsunamiSirenSumSquare = 0.00;
double nuclearAlarmSumSquare = 0.00;

double policeSirenNNSumSquare = 0.00;
double policeSirenWNSumSquare = 0.00;
double fireTruckSirenNNSumSquare = 0.00;
double fireTruckSirenWNSumSquare = 0.00;

double tornadoSirenNNSumSquare = 0.00;
double tornadoSirenWNSumSquare = 0.00;

double policeYelpNNSumSquare = 0.00;
double policeYelpWNSumSquare = 0.00;
double policeHornNNSumSquare = 0.00;
double policeHornWNSumSquare = 0.00;

double ambulanceSirenNNSumSquare = 0.00;
double ambulanceSirenWNSumSquare = 0.00;

int backUpAlarmNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,77,36,40,43,32,29,22,24,18,30,35,37,32,33,27,29,57,50,27,61,50,29,56,33,29,18,33,32,29,29,46,47,33,26,78,62,487,3968,4392,718,50,62,45,51,53,54,39,27,26,31,11,33,12,31,16,23,12,18,21,27,21,15,8,17,8,17,6,13,7,13,8,12,7,6,9,7,9,5,12,6,4,12,13,7,98,296,158,23,10,14,10,13,7,14,6,13,11,7,7,7,5,9,7,9,4,4,12,8,8,9,6,7,8,8,9,7,7,9,9,8,8,6,4,6,5,6,3,6,4,5,6,5,37,49,12,6,8,7,7,6,6,6,5,3,6,6,6,5,7,4,6,7,4,8,5,7,5,5,4,4,4,4,7,6,6,6,7,3,8,6,7,5,4,6,4,6,5,4,6,3,8,8,4,4,5,5,3,6,4,3,7,5,5,4,7,2,4,4,3,4,3,2,4,4,6,3,3,3,8,4,9,3,6,5,7,4,5,3,4,7,5,5,4,5,5,5,9,17,23,10,9,6,6,3,6,5,5,5,5,3,7,5,5,5,5,4};
int backUpAlarmWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,45,43,50,42,32,43,35,20,20,17,40,48,24,24,42,50,102,83,51,74,68,55,63,57,30,20,26,32,32,31,27,33,10,25,42,31,158,1484,1637,272,57,32,54,52,32,24,22,28,27,29,26,14,28,12,16,14,16,9,10,15,13,7,10,12,10,15,14,13,15,11,9,6,5,5,10,6,9,5,5,6,11,8,12,18,21,98,62,23,17,6,7,15,10,9,9,17,13,9,8,9,4,2,5,6,5,6,5,5,4,2,4,3,4,7,6,7,8,8,6,4,3,9,5,5,7,8,4,4,7,5,8,4,8,10,6,5,6,7,5,4,4,5,8,4,8,9,8,5,5,6,7,6,6,5,5,7,4,5,6,8,6,5,6,3,4,8,6,3,8,8,7,6,5,3,4,8,6,16,23,22,18,14,7,5,5,7,4,5,3,5,6,5,7,6,4,8,9,5,6,6,10,7,6,10,6,8,8,6,7,8,4,4,6,4,1,5,7,6,6,4,5,4,7,6,5,6,6,10,12,3,4,5,4,4,7,7,4,4,3,5,5,2,4,2,4,2};
int emergencyAlertSystem1[arraySize] = {0,0,0,0,0,0,0,0,0,0,9,4,5,3,7,4,6,4,4,5,6,5,6,6,3,3,5,4,6,8,5,7,4,6,6,7,6,3,6,6,7,5,6,5,6,4,3,3,3,3,4,6,6,8,8,8,4,4,4,3,9,7,5,3,4,5,11,5,3,6,4,4,9,7,6,9,8,6,6,5,5,6,2,4,2,3,3,6,7,14,6,4,8,5,5,4,5,4,3,4,6,5,3,5,4,4,4,2,3,4,4,5,4,4,3,4,4,3,3,4,4,6,5,6,3,4,5,4,4,5,4,5,5,5,5,3,4,4,5,4,5,3,3,2,4,3,2,4,3,4,2,4,5,4,3,4,4,3,6,7,3,3,5,4,4,6,5,2,4,3,3,2,3,4,4,4,3,3,2,3,4,3,3,3,3,4,4,3,4,5,6,3,5,5,1,3,5,4,4,6,13,7,5,5,4,3,4,3,2,3,4,5,5,3,4,4,5,5,3,3,3,3,3,3,4,4,3,3,4,3,3,2,3,5,4,2,2,3,3,3,3,2,4,4,4,5,4,4,4,3,4,4,2,3,5,2};
int emergencyAlertSystem2[arraySize] = {0,0,0,0,0,0,0,0,0,0,11,13,10,12,13,16,11,15,16,16,16,19,19,20,24,20,25,27,28,31,38,34,47,51,45,1515,5449,3377,338,381,4006,6074,1567,61,52,53,36,39,33,31,31,24,27,22,22,20,21,20,14,16,18,15,15,13,13,14,12,12,10,16,7,11,13,12,13,7,19,20,21,12,8,48,43,9,8,7,9,6,8,7,7,5,8,10,7,6,7,9,3,9,7,6,8,3,9,5,7,9,4,7,6,4,8,6,4,9,4,6,5,4,9,8,8,7,5,6,4,7,5,8,5,6,7,8,5,5,5,7,4,6,8,5,7,7,7,7,5,5,7,3,7,6,7,3,5,7,3,5,6,5,8,7,6,6,6,5,4,7,3,6,6,2,5,2,5,5,3,5,4,5,7,4,6,4,6,7,5,4,6,3,5,7,3,5,4,3,5,4,5,4,15,7,4,8,6,4,6,5,3,6,3,4,6,6,8,9,5,7,7,8,7,6,5,5,3,7,3,5,7,3,3,5,3,4,4,4,5,6,3,7,5,5,3,6,5,6,6,2,4,3,5,2,5,5,6,7};

int fireTruckHorn[arraySize] = {0,0,0,0,0,0,0,0,0,0,13,15,9,12,15,20,201,519,254,62,27,23,37,86,42,326,402,94,21,25,57,646,811,214,419,226,26,51,43,253,845,467,69,108,41,25,19,18,31,48,32,95,59,10,9,4,10,36,20,74,110,28,20,23,20,17,13,12,62,39,15,24,25,17,28,31,69,138,53,38,163,161,52,47,83,340,281,60,81,95,59,84,110,331,515,173,123,87,90,53,74,135,306,271,193,89,50,69,89,83,306,675,636,217,51,45,41,81,47,240,232,96,53,29,25,24,25,47,77,35,31,26,17,32,61,84,167,176,65,45,78,73,69,138,233,266,124,41,24,16,30,55,74,71,103,51,38,25,33,45,49,72,172,145,89,82,35,58,105,66,145,227,119,86,45,42,52,85,78,138,111,54,68,58,54,50,36,67,112,59,104,86,85,104,115,127,216,141,122,133,47,65,80,70,176,263,66,64,59,68,56,49,97,144,124,79,73,44,84,78,61,164,277,97,87,91,87,64,138,158,612,564,103,179,99,161,128,127,45,196,126,126,138,91,82,52,126,293,277,158,131,87,118,98,35,157};

int smokeAlarm[arraySize] = {0,0,0,0,0,0,0,0,0,0,9,4,5,8,5,4,3,6,5,3,7,9,8,7,6,6,4,5,2,5,2,4,4,4,6,4,8,13,6,5,6,7,10,9,7,4,4,6,3,4,4,5,5,4,5,3,4,4,7,6,3,6,4,5,5,4,4,6,5,4,5,4,6,3,5,4,5,4,6,6,6,6,6,5,3,4,2,6,4,6,4,4,9,8,5,5,7,6,7,8,6,9,5,6,5,10,4,10,8,8,8,10,19,23,8,6,12,9,9,13,13,18,12,20,17,25,18,36,18,22,46,102,1452,4822,2854,108,88,73,14,34,30,22,31,36,25,9,16,9,8,9,6,9,5,7,10,8,8,8,7,7,5,7,3,7,5,6,4,7,5,6,4,5,5,4,4,3,9,4,6,5,2,5,5,5,5,5,4,6,6,4,6,6,6,3,3,5,2,6,7,7,14,7,4,5,4,4,4,6,4,4,6,5,6,4,3,4,2,5,3,4,4,6,3,6,6,5,6,4,5,3,5,3,5,6,6,6,4,6,5,4,4,4,7,5,3,26,27,8,4,4,4,3,3,2,5,5};
int fireAlarm[arraySize] = {0,0,0,0,0,0,0,0,0,0,41,48,35,57,23,19,25,19,18,15,30,22,19,19,12,13,15,22,15,21,23,27,30,18,16,8,12,17,19,25,20,17,12,24,21,23,27,13,33,16,13,25,20,15,26,22,12,20,12,17,15,21,29,21,18,19,18,32,20,17,21,14,28,14,13,13,25,27,21,24,20,26,21,27,30,12,20,17,13,16,21,18,35,25,36,57,48,52,45,43,28,38,38,43,30,36,32,27,17,26,25,20,18,24,14,34,21,31,35,28,32,31,28,40,23,42,46,97,77,92,60,163,329,169,446,288,547,1069,1199,4462,3986,1902,4049,1378,2772,2745,408,320,960,3090,2223,498,150,997,2684,1392,1844,2703,715,1616,1295,1043,1517,407,647,344,178,277,85,69,45,43,46,22,41,35,23,16,27,30,18,29,22,9,14,19,29,25,23,18,15,33,18,28,36,26,42,26,25,28,15,25,22,17,17,18,24,22,31,27,25,55,22,46,41,35,68,30,36,38,17,44,25,69,49,46,54,28,92,54,41,48,25,46,27,23,32,17,12,13,10,7,7,8,14,8,9,10,5,4,4,7,6,8,8,7};
int tsunamiSiren[arraySize] = {0,0,0,0,0,0,0,0,0,0,124,135,122,212,245,636,2213,1716,372,189,256,204,156,152,65,77,92,114,77,161,143,756,1949,2270,507,161,96,137,114,151,120,205,91,130,83,68,119,328,566,1412,620,29,23,19,19,11,13,14,10,13,18,21,107,210,193,488,378,49,22,25,22,18,21,26,27,25,29,46,75,62,62,372,421,125,43,30,35,36,35,32,55,43,57,66,139,60,282,1284,2938,1446,171,185,184,154,189,118,63,41,127,161,277,300,257,217,774,599,108,31,37,17,23,50,24,37,44,71,58,21,42,64,112,152,69,18,14,20,13,23,25,10,29,50,45,34,37,64,103,124,69,24,22,19,11,12,16,21,15,20,33,48,35,27,43,144,157,54,12,10,15,17,19,11,10,18,21,23,20,15,33,84,106,36,13,10,14,7,6,13,16,8,8,8,9,13,13,32,47,29,14,8,13,14,17,9,9,9,5,11,9,8,16,15,61,63,22,7,12,19,15,7,6,19,21,19,24,23,31,40,102,131,63,28,19,27,20,31,32,41,37,32,26,19,20,22,96,130,55,23,8,15,13,26,17,7,9,10};
int nuclearAlarm[arraySize] = {0,0,0,0,0,0,0,0,0,0,145,100,156,107,47,41,56,46,68,121,133,69,169,861,4640,3961,461,140,1304,5217,3676,213,71,26,88,42,69,72,60,111,57,45,51,30,37,27,28,52,251,375,102,17,9,19,24,12,7,13,134,147,30,14,10,11,7,8,13,7,9,6,5,7,38,98,56,10,7,13,42,25,5,6,13,28,11,16,24,314,529,168,54,11,14,8,20,33,77,480,552,133,39,20,15,20,5,4,8,6,7,6,5,3,9,7,10,18,145,405,225,24,6,69,148,65,8,6,3,6,5,6,3,6,5,3,4,4,6,5,5,7,4,4,4,1,2,8,21,20,6,7,4,3,5,4,4,5,4,6,7,5,10,12,7,6,5,6,4,2,4,7,20,37,14,7,10,35,39,14,7,4,3,4,4,3,4,2,5,3,2,10,15,9,5,5,10,19,14,6,3,6,14,10,7,7,17,39,16,4,3,6,7,6,4,4,5,5,6,6,17,36,19,4,5,5,7,6,4,4,6,4,4,5,3,34,149,105,10,5,4,5,3,6,8,57,100,42,8,6,21,22,7,2,5,5,4,5};

int policeSirenNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,118,44,35,37,23,34,35,23,20,24,8,15,18,13,10,13,19,21,22,31,18,44,146,240,370,467,582,875,795,650,979,682,207,130,210,139,223,497,460,462,266,174,152,171,182,116,75,37,78,109,83,105,169,243,50,93,142,117,66,51,109,71,90,63,63,49,44,36,27,25,22,16,11,25,28,15,15,13,18,13,12,16,16,30,37,16,20,15,26,49,54,45,32,28,30,31,23,20,32,35,39,61,67,32,26,48,55,21,42,31,25,28,33,16,17,10,13,16,10,24,19,23,14,7,22,22,24,9,17,11,18,9,11,14,10,8,5,12,6,8,11,6,9,10,12,16,19,11,12,19,19,4,10,16,9,10,8,18,16,27,17,18,15,15,17,23,20,19,15,13,15,10,8,13,11,9,9,12,9,8,4,7,10,9,6,2,9,13,8,7,3,7,8,3,3,6,7,11,5,5,6,7,3,2,5,5,3,7,9,6,7,6,6,8,4,5,4,5,5,6,5,5,6,8,9,9,6,4,4,7,8,7,9,8,6,7,4,7,7,5,6,6,4,3,1,4};
int policeSirenWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,49,58,45,25,21,30,23,21,38,37,17,14,30,58,77,56,88,136,123,184,184,53,88,46,44,58,73,70,94,81,79,127,37,109,182,120,200,383,383,159,271,240,394,188,103,193,524,381,136,306,453,426,550,548,362,321,307,186,178,181,152,115,73,12,20,20,18,15,12,21,18,9,5,14,13,11,9,21,20,21,11,26,33,62,71,65,49,25,37,34,17,25,24,13,9,7,17,29,14,14,11,7,5,10,10,20,22,24,28,22,24,27,19,8,11,10,10,17,10,18,10,6,8,8,15,9,8,5,7,11,11,10,12,14,11,8,9,17,21,17,11,14,10,18,32,38,15,9,6,12,9,10,8,5,6,13,19,12,9,17,17,11,9,8,6,12,12,6,9,11,7,8,7,6,10,14,9,10,14,10,8,6,10,10,10,7,6,7,7,11,15,13,15,15,11,4,5,7,4,6,8,10,14,8,4,4,4,7,10,7,10,6,4,3,2,5,5,7,5,8,9,6,10,9,6,4,5,7,8,5,5,8,3,10,9,6,6,3,3,4,7,5,5,5,10,7};
//int fireTruckSirenNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,89,27,25,37,39,37,44,499,1080,596,244,223,249,192,196,125,95,117,90,107,173,140,151,178,416,1382,3338,2226,431,493,313,230,316,492,586,452,380,448,517,383,342,483,1162,2406,6169,4510,924,448,187,71,45,82,163,232,355,508,460,315,135,244,260,1014,1723,1036,250,120,92,54,54,39,30,14,28,18,17,23,31,72,115,258,393,344,161,148,99,73,69,82,74,92,78,79,57,150,237,208,215,291,803,830,527,149,75,69,49,32,25,31,24,21,30,89,87,71,88,180,220,233,177,101,77,105,77,52,38,38,41,42,37,47,68,70,243,445,725,721,412,153,72,62,81,143,174,130,90,43,40,47,74,70,98,103,66,192,244,67,64,55,70,66,21,13,18,24,44,39,34,22,16,21,12,20,11,11,8,15,10,9,9,15,16,8,11,25,26,38,64,66,53,72,55,35,13,30,27,12,13,25,28,33,31,22,33,40,16,13,21,17,43,31,29,13,22,25,54,39,30,10,26,38,25,31,57,45,60,40,65,71,35,19,16,10,14,46,43,13,7,25,20,5,19,36,34,14,17,26};
int fireTruckSirenNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,110,93,105,198,228,269,144,118,187,160,168,178,205,246,205,313,426,6727,31957,31427,6052,400,436,290,385,149,179,109,142,103,119,122,213,310,137,97,81,92,57,48,36,60,44,37,28,220,1456,2415,1347,283,64,41,53,46,33,33,35,23,41,23,34,42,40,35,35,33,38,38,33,104,121,93,143,252,2097,5092,3933,2295,622,225,224,193,186,90,86,40,43,79,141,35,105,380,655,552,506,461,212,178,272,248,132,469,1630,6320,5700,4208,3712,644,189,110,122,87,39,38,35,31,43,71,50,32,21,28,18,32,20,25,21,27,40,112,271,655,470,158,535,499,163,101,62,66,24,18,17,29,36,147,128,29,51,169,103,35,37,31,32,61,35,105,120,848,1834,1222,339,972,763,148,35,52,52,17,107,52,25,73,135,91,57,159,147,74,23,33,65,285,884,396,163,210,579,228,622,1132,1170,632,158,135,66,36,22,34,44,62,128,132,126,39,72,90,61,23,57,41,207,470,366,820,1471,1636,907,312,408,670,530,389,322,166,291,332,75,242,594,533,292,122,186,196,247,210,104,84,67,1076,2319,1127,1444,2062};
int fireTruckSirenWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,42,36,33,52,65,68,62,54,436,747,260,89,122,107,91,59,41,129,142,112,21,63,37,30,60,182,199,2202,3759,1383,108,33,60,67,31,42,31,50,28,56,16,52,106,167,387,1768,4947,6212,2263,449,150,194,63,34,39,23,29,23,19,25,47,39,56,62,146,345,284,109,46,13,9,10,9,13,7,8,7,7,11,21,38,21,109,495,977,1057,582,202,62,61,22,8,18,9,6,13,6,21,16,60,25,125,649,1052,954,534,142,45,23,13,8,8,8,8,10,9,16,12,23,34,91,168,323,283,130,101,100,51,39,12,3,6,11,13,6,20,60,100,58,86,194,268,202,109,43,34,7,8,18,10,1,9,5,17,28,33,47,68,185,304,285,227,141,103,45,24,22,9,9,10,6,10,13,13,10,9,12,13,15,25,39,13,13,27,20,9,13,8,10,11,4,8,5,9,20,21,37,40,37,45,64,37,16,6,7,12,5,13,13,9,17,20,16,31,81,133,163,172,149,81,41,47,22,13,20,12,18,7,7,18,18,16,8,12,22,30,71,84,47,37,23,13,33,34,15,10};

int tornadoSirenNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,29,30,33,124,71,106,78,155,123,252,269,204,249,165,235,1404,8989,8607,1131,276,181,225,242,169,114,200,117,98,127,77,80,79,162,125,184,134,144,103,102,51,55,79,437,804,375,45,37,23,41,22,36,19,24,20,20,17,27,15,43,22,52,82,112,93,36,59,54,66,140,251,240,133,24,85,99,84,65,82,147,151,114,34,39,58,49,40,87,118,322,432,220,159,269,264,500,2502,3478,2499,827,124,80,116,128,68,60,55,53,81,44,46,21,27,26,52,42,54,49,30,37,56,55,294,489,264,150,21,61,38,66,39,42,54,92,126,81,29,19,32,32,21,35,38,36,34,46,44,51,244,740,626,371,318,107,65,98,66,30,65,148,166,119,39,49,61,56,25,60,40,31,65,58,33,49,104,223,269,168,384,306,71,36,74,62,65,82,135,130,70,39,19,37,23,26,35,30,19,35,23,27,47,73,166,110,26,75,70,51,67,95,62,49,115,172,138,47,56,30,28,61,61,80,62,168,254,186,181,463,1195,1108,439,344,445,189,148,121,73,54,73,118,127,64,45,68,50,26,51};
int tornadoSirenWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,73,51,32,33,24,37,32,26,23,46,136,447,1188,449,64,63,48,40,45,26,17,15,41,44,23,31,44,41,74,98,63,85,398,961,756,137,67,113,100,137,89,90,51,43,44,50,24,27,33,27,48,107,169,234,318,1880,2574,682,70,124,55,105,51,46,63,40,29,28,17,18,18,17,42,27,23,48,161,389,385,116,48,111,76,26,52,62,48,69,76,110,44,50,44,55,47,93,133,47,118,392,452,123,20,9,13,21,57,52,68,56,24,25,26,21,13,23,27,18,24,39,86,243,214,56,11,40,29,51,111,100,65,37,26,35,21,19,15,19,8,9,22,29,108,269,225,78,22,17,10,25,44,36,31,37,26,20,28,23,18,14,13,12,34,39,99,179,86,15,13,13,19,27,73,73,61,27,13,15,14,7,10,9,11,8,15,11,23,41,30,11,7,11,5,6,23,27,19,8,13,12,14,8,12,12,20,25,34,29,72,125,102,37,21,11,15,19,35,50,37,13,5,7,7,14,16,7,10,13,30,30,92,129,67,24,11,10,22,22,47,66,56,12,18,26,17,10};

int policeYelpNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,55,40,29,45,22,25,18,24,28,29,18,26,37,43,64,51,47,55,62,104,157,58,209,231,127,158,203,241,670,973,749,889,1279,2928,3845,3947,2896,1082,160,222,203,128,83,96,64,77,71,67,71,46,41,31,37,21,44,20,46,33,37,45,25,37,50,46,44,74,61,27,23,35,30,43,60,66,119,182,155,186,165,205,255,257,213,129,97,111,59,51,29,42,25,53,16,35,26,15,13,17,25,23,27,33,31,20,30,14,5,11,36,43,43,54,109,154,128,137,126,142,139,110,93,94,39,58,77,80,115,131,97,89,70,19,29,12,30,16,7,7,9,14,14,8,9,6,13,16,14,12,21,10,23,44,46,49,16,9,10,9,8,13,17,15,9,10,9,20,26,17,26,13,20,18,14,11,10,7,7,11,7,8,5,10,8,11,7,8,7,7,10,9,10,12,14,8,2,10,9,19,26,28,26,34,31,28,33,33,22,8,6,10,11,12,19,35,38,37,32,44,55,43,37,38,21,17,18,14,14,12,22,21,31,21,21,19,40,29,37,28,38,23,11,36,39,22,12,11};
int policeYelpWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,68,29,14,58,23,65,39,38,53,62,42,80,39,26,46,54,69,97,145,123,159,77,227,131,34,112,121,226,167,246,230,335,239,664,638,1466,854,4717,12795,14483,6280,1208,132,89,125,70,54,88,59,53,40,52,36,39,37,33,43,30,25,23,45,8,29,29,26,14,33,17,13,16,30,12,27,15,27,28,38,37,34,53,43,78,68,85,192,131,363,594,519,372,174,46,37,23,19,24,7,17,27,10,19,11,16,20,18,16,12,10,31,13,19,10,13,27,11,13,22,22,31,13,42,33,19,16,40,18,59,50,18,37,75,38,124,205,176,286,335,287,188,122,109,54,21,23,35,26,10,11,6,13,19,4,17,11,9,14,14,9,8,12,13,19,4,9,6,8,4,14,12,15,10,10,16,4,7,10,11,9,9,11,4,11,8,10,9,11,5,11,14,4,9,6,9,10,15,8,9,8,8,12,13,7,4,16,10,25,12,16,36,24,19,12,22,19,12,18,14,8,8,5,26,15,11,15,29,24,30,57,18,86,106,74,68,64,56,45,41,30,30,21,20,19,8,12,7,12};
int policeHornNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,50,27,22,20,13,12,14,11,13,18,75,63,16,53,70,22,153,271,89,373,836,345,348,1096,600,57,286,198,88,669,622,75,164,186,70,701,1076,313,497,1058,355,531,1368,702,286,1225,803,83,553,486,79,400,445,105,388,515,111,196,419,155,156,283,114,28,116,60,12,74,58,16,25,20,9,9,13,8,10,19,20,15,8,19,22,32,33,57,116,88,42,150,148,88,154,111,164,133,105,152,127,51,111,98,12,63,73,21,80,111,38,22,51,34,19,67,29,9,16,15,11,59,42,18,64,69,30,30,33,42,43,34,13,17,12,13,18,11,12,10,7,5,18,16,9,18,7,8,13,11,5,7,9,5,6,7,5,6,6,8,6,9,6,4,10,7,3,4,5,6,5,4,4,5,5,6,8,7,3,4,7,4,5,3,4,5,4,3,3,4,4,4,4,3,5,7,6,5,4,6,8,4,3,5,6,5,4,5,6,6,5,4,4,3,4,3,5,3,3,5,5,5,5,7,4,3,5,5,6,4,5,7,5,6,7,7,5,4,3,3,3,4,3,5,3,3,2,3};
int policeHornWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,34,39,35,26,25,35,30,37,36,52,80,69,44,49,83,28,123,216,114,342,700,247,372,1229,784,136,245,245,262,782,664,237,262,259,105,770,1370,492,337,691,523,830,1197,658,496,1116,616,154,477,415,102,320,410,164,364,396,193,257,441,210,163,245,122,46,88,88,25,69,56,30,20,15,7,16,19,13,17,24,26,19,13,24,26,46,25,32,113,83,40,142,173,92,131,135,199,176,131,128,125,77,138,91,38,97,104,42,90,142,45,24,49,34,45,68,24,12,19,22,24,66,59,35,61,82,52,41,38,42,38,26,22,30,28,31,25,11,8,17,18,12,20,20,16,20,12,8,10,6,6,11,4,4,8,10,6,8,10,9,14,12,7,10,17,12,11,12,12,11,8,9,8,3,10,5,5,4,8,5,2,5,5,7,5,8,6,4,8,8,6,9,8,8,6,3,5,6,6,6,7,12,7,6,6,6,12,3,2,4,3,3,6,6,5,5,3,5,6,4,3,4,5,4,6,4,6,5,5,5,7,6,3,5,4,4,4,4,7,8,6,8,6,6,4,4,4,5};

int ambulanceSirenNN[arraySize] = {0,0,0,0,0,0,0,0,0,0,68,75,51,42,19,12,18,17,10,16,18,16,15,16,12,10,9,4,9,5,11,10,34,23,32,35,37,68,52,66,119,101,81,88,199,346,477,787,974,535,1680,3010,2514,1136,468,104,31,15,16,12,10,11,12,10,9,5,10,5,7,5,7,6,8,5,7,9,4,4,5,6,7,7,8,3,7,5,5,5,8,13,16,24,41,32,33,64,47,49,82,112,66,83,123,73,19,45,13,22,16,3,4,4,4,6,4,7,6,10,7,5,7,5,5,7,7,8,11,7,7,6,6,13,16,8,7,7,7,9,8,5,11,14,9,6,8,10,10,20,24,15,18,15,27,43,28,16,19,49,51,31,54,75,30,5,7,6,5,6,4,6,2,6,5,3,2,2,4,2,4,9,4,3,5,7,4,7,4,2,5,4,4,5,5,4,3,4,7,8,7,5,4,3,6,9,10,6,7,5,6,5,4,6,4,8,6,5,4,6,6,4,5,3,4,4,2,2,3,5,4,5,3,3,5,4,3,4,5,5,5,6,4,5,6,6,11,10,6,6,5,6,6,6,6,6,6,5};
int ambulanceSirenWN[arraySize] = {0,0,0,0,0,0,0,0,0,0,42,44,34,42,39,25,44,97,64,24,23,24,46,25,42,40,41,34,19,17,14,18,17,23,21,12,14,33,20,30,23,25,49,41,134,106,129,264,442,288,453,488,468,912,1732,1626,912,488,99,47,17,23,23,20,10,15,10,9,13,6,6,3,16,10,8,3,8,8,6,10,7,9,7,6,8,5,12,6,11,8,7,12,11,12,7,10,11,21,26,23,15,25,20,20,24,19,12,13,20,19,50,51,79,91,21,10,7,9,5,3,6,4,5,10,9,5,6,6,3,7,4,6,7,4,5,6,3,4,4,5,10,5,2,4,6,6,7,6,9,6,7,9,5,10,7,7,6,11,4,10,9,12,8,15,23,41,35,32,80,116,62,10,7,7,9,7,6,6,8,8,6,6,5,8,11,10,6,4,4,9,8,9,11,9,6,3,5,5,6,6,6,6,8,7,8,9,5,5,4,5,6,4,6,4,4,3,7,7,5,5,3,3,7,5,6,6,5,4,7,5,5,7,9,5,5,8,8,7,6,6,6,6,6,5,6,7,6,4,6,5,5,4,4,5,3,2};

double micInputSumSquare = 0.00;
int micInput[arraySize];

const float activeSiren = 0.75;

int storeDirectionCoor = 0;

//Vibration motor connection
const int vibrationMotorPin = A13;

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
int micFFT = mic2; 

void setup() 
{
  Serial.begin(9600);

  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  //Sum of (siren^2)
  //Part of the correlation()
  for (int i = 0; i < arraySize; i++)
  {
    backUpAlarmNNSumSquare = backUpAlarmNNSumSquare + pow(backUpAlarmNN[i], 2);
    backUpAlarmWNSumSquare = backUpAlarmWNSumSquare + pow(backUpAlarmWN[i], 2);
    emergencyAlertSystem1SumSquare = emergencyAlertSystem1SumSquare + pow(emergencyAlertSystem1[i], 2);
    emergencyAlertSystem2SumSquare = emergencyAlertSystem2SumSquare + pow(emergencyAlertSystem2[i], 2);
    
    fireTruckHornSumSquare = fireTruckHornSumSquare + pow(fireTruckHorn[i], 2);
    
    smokeAlarmSumSquare = smokeAlarmSumSquare + pow(smokeAlarm[i], 2);
    fireAlarmSumSquare = fireAlarmSumSquare + pow(fireAlarm[i], 2);  
    tsunamiSirenSumSquare = tsunamiSirenSumSquare + pow(tsunamiSiren[i], 2);
    nuclearAlarmSumSquare = nuclearAlarmSumSquare + pow(nuclearAlarm[i], 2);
    
    policeSirenNNSumSquare = policeSirenNNSumSquare + pow(policeSirenNN[i], 2);
    policeSirenWNSumSquare = policeSirenWNSumSquare + pow(policeSirenWN[i], 2);   
    fireTruckSirenNNSumSquare = fireTruckSirenNNSumSquare + pow(fireTruckSirenNN[i], 2);
    fireTruckSirenWNSumSquare = fireTruckSirenWNSumSquare + pow(fireTruckSirenWN[i], 2);   
    
    tornadoSirenNNSumSquare = tornadoSirenNNSumSquare + pow(tornadoSirenNN[i], 2);
    tornadoSirenWNSumSquare = tornadoSirenWNSumSquare + pow(tornadoSirenWN[i], 2);
    
    policeYelpNNSumSquare = policeYelpNNSumSquare + pow(policeYelpNN[i], 2);
    policeYelpWNSumSquare = policeYelpWNSumSquare + pow(policeYelpWN[i], 2);  
    policeHornNNSumSquare = policeHornNNSumSquare + pow(policeHornNN[i], 2);
    policeHornWNSumSquare = policeHornWNSumSquare + pow(policeHornWN[i], 2);
     
    ambulanceSirenNNSumSquare = ambulanceSirenNNSumSquare + pow(ambulanceSirenNN[i], 2);
    ambulanceSirenWNSumSquare = ambulanceSirenWNSumSquare + pow(ambulanceSirenWN[i], 2); 
  }

  //Initialize screen
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);

  //Initialize vibration motor
  pinMode(vibrationMotorPin,OUTPUT);

  //Used in mics error checking
  bool micsFail = true;
  
  Serial.println("Initialize Teensy succesful");

  //Check for mics error
  while(micsFail)
  {
    micsFail = checkMics(micsFail);
  }
  
  Serial.println("Microphones check successful");
  Serial.println("Starting program");

  //Draw the area of display the arrow for direction
  tft.drawCircle(63, 49, 44, ST7735_WHITE);
}

void loop() 
{
  /*for (int i = 0; i < 9; i++)
  {
    displayDirection("10", i);
    delay(1000);
  }

  for (int i = 2; i < 4; i++)
  {
    storeDirectionCoor = i;
    
    displaySiren("siren1", i, "String 1", "String 2", "String 3");
    delay(2000);
    displaySiren("siren2", i, "String 1", "String 2", "String 3");
    delay(2000);
  }*/

  /*int incomingChar = 'y';

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
        correlation();
        //delay(44);
      }
  }*/
  
  directionality();
  //correlation();
  
  //delay(100);
  //vibrationMotor(600);
}

void directionality()
{
  int samplingTime = 200; //867;

  double ppkTheshold = 30;

  int z = 0;

  int storeMicsCount[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  //Peak-to-peak level
  double mic1ppk[samplingTime]; 
  double mic2ppk[samplingTime];   
  double mic3ppk[samplingTime];
  double mic4ppk[samplingTime];
  double mic5ppk[samplingTime];
  double mic6ppk[samplingTime];
  double mic7ppk[samplingTime];
  double mic8ppk[samplingTime];
    
  while (z < 50)
  {
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
      /*double micsAnalog[8] = {analogRead(mic1)*1.0025, analogRead(mic2)*1.0096, analogRead(mic3)*0.9996, analogRead(mic4)*01.0104, 
                            analogRead(mic5)*0.9977, analogRead(mic6)*1.0085, analogRead(mic7)*1.0003, analogRead(mic8)*1.0113};*/ //The old mics on the first PCB
      double micsAnalog[8] = {analogRead(mic1)*0.9994, analogRead(mic2)*0.9929, analogRead(mic3)*0.9969, analogRead(mic4)*0.9987, 
                            analogRead(mic5)*1.0021, analogRead(mic6)*1.0042, analogRead(mic7)*1.0056, analogRead(mic8)*1.0058}; //The new mics on the second PCB

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
    }

    for(int i = 0; i < samplingTime; i++)
    {       
      if (((mic1ppk[i] > 0) || (mic2ppk[i] > 0) || (mic3ppk[i] > 0) || (mic4ppk[i] > 0) || (mic5ppk[i] > 0) || (mic6ppk[i] > 0) || (mic7ppk[i] > 0) || (mic8ppk[i] > 0)))
      {
        if (mic1ppk[i] > 0)
        {
          storeMicsCount[0] = storeMicsCount[0]+1;
        }
        
        if (mic2ppk[i] > 0)
        {
          storeMicsCount[1] = storeMicsCount[1]+1;
        }
        
        if (mic3ppk[i] > 0)
        {
          storeMicsCount[2] = storeMicsCount[2]+1;
        }
        
        if (mic4ppk[i] > 0)
        {
          storeMicsCount[3] = storeMicsCount[3]+1;
        }
        
        if (mic5ppk[i] > 0)
        {
          storeMicsCount[4] = storeMicsCount[4]+1;
        }
        
        if (mic6ppk[i] > 0)
        {
          storeMicsCount[5] = storeMicsCount[5]+1;
        }
        
        if (mic7ppk[i] > 0)
        {
          storeMicsCount[6] = storeMicsCount[6]+1;
        }

        if (mic8ppk[i] > 0)
        {
          storeMicsCount[7] = storeMicsCount[7]+1;
        }

        break;
      }
    }

    /*Serial.print(storeMicsCount[0]);
    Serial.print("  ");
    Serial.print(storeMicsCount[1]);
    Serial.print("  ");
    Serial.print(storeMicsCount[2]);
    Serial.print("  ");
    Serial.print(storeMicsCount[3]);
    Serial.print("  ");
    Serial.print(storeMicsCount[4]);
    Serial.print("  ");
    Serial.print(storeMicsCount[5]);
    Serial.print("  ");
    Serial.print(storeMicsCount[6]);
    Serial.print("  ");
    Serial.println(storeMicsCount[7]);*/


    z++;
  }
    
  //For debugging
  //Print to screen
  /*tft.fillRect(0, 76, 128, 84, ST7735_BLACK);
  tft.setCursor(0, 77);
  tft.println(storeMicsCount[0]);
  tft.println(storeMicsCount[1]);
  tft.println(storeMicsCount[2]);
  tft.println(storeMicsCount[3]);
  tft.println(storeMicsCount[4]);
  tft.println(storeMicsCount[5]);
  tft.println(storeMicsCount[6]);
  tft.println(storeMicsCount[7]);*/

  //Print to console
  Serial.print(storeMicsCount[0]);
  Serial.print("   ");
  Serial.print(storeMicsCount[1]);
  Serial.print("   ");
  Serial.print(storeMicsCount[2]);
  Serial.print("   ");
  Serial.print(storeMicsCount[3]);
  Serial.print("   ");
  Serial.print(storeMicsCount[4]);
  Serial.print("   ");
  Serial.print(storeMicsCount[5]);
  Serial.print("   ");
  Serial.print(storeMicsCount[6]);
  Serial.print("   ");
  Serial.println(storeMicsCount[7]);

  if(storeMicsCount[0] > storeMicsCount[1] && storeMicsCount[0] > storeMicsCount[2] && storeMicsCount[0] > storeMicsCount[3] && storeMicsCount[0] > storeMicsCount[4]
  && storeMicsCount[0] > storeMicsCount[5] && storeMicsCount[0] > storeMicsCount[6] && storeMicsCount[0] > storeMicsCount[7])
  {
    storeDirectionCoor = 3;//1
    micFFT = mic1;
    displayDirection("1", storeDirectionCoor);

    //Run the vibration motor
    vibrationMotor(300);
  }   
  else if(storeMicsCount[1] > storeMicsCount[0] && storeMicsCount[1] > storeMicsCount[2] && storeMicsCount[1] > storeMicsCount[3] && storeMicsCount[1] > storeMicsCount[4] 
       && storeMicsCount[1] > storeMicsCount[5] && storeMicsCount[1] > storeMicsCount[6] && storeMicsCount[1] > storeMicsCount[7])
  {
    storeDirectionCoor = 2;
    micFFT = mic2;
    displayDirection("2", storeDirectionCoor);

    vibrationMotor(300);
  }
  else if(storeMicsCount[2] > storeMicsCount[0] && storeMicsCount[2] > storeMicsCount[1] && storeMicsCount[2] > storeMicsCount[3] && storeMicsCount[2] > storeMicsCount[4] 
       && storeMicsCount[2] > storeMicsCount[5] && storeMicsCount[2] > storeMicsCount[6] && storeMicsCount[2] > storeMicsCount[7])
  { 
    storeDirectionCoor = 1; //3
    micFFT = mic3;
    displayDirection("3", storeDirectionCoor);

    vibrationMotor(300);
  } 
  else if(storeMicsCount[3] > storeMicsCount[0] && storeMicsCount[3] > storeMicsCount[1] && storeMicsCount[3] > storeMicsCount[2] && storeMicsCount[3] > storeMicsCount[4] 
       && storeMicsCount[3] > storeMicsCount[5] && storeMicsCount[3] > storeMicsCount[6] && storeMicsCount[3] > storeMicsCount[7])
  {
    storeDirectionCoor = 8; //4
    micFFT = mic4;
    displayDirection("4", storeDirectionCoor);

    vibrationMotor(300);
  }   
  else if(storeMicsCount[4] > storeMicsCount[0] && storeMicsCount[4] > storeMicsCount[1] && storeMicsCount[4] > storeMicsCount[2] && storeMicsCount[4] > storeMicsCount[3] 
       && storeMicsCount[4] > storeMicsCount[5] && storeMicsCount[4] > storeMicsCount[6] && storeMicsCount[4] > storeMicsCount[7])
  {
    storeDirectionCoor = 7; //5
    micFFT = mic5;
    displayDirection("5", storeDirectionCoor);

    vibrationMotor(300);
  }
  else if(storeMicsCount[5] > storeMicsCount[0] && storeMicsCount[5] > storeMicsCount[1] && storeMicsCount[5] > storeMicsCount[2] && storeMicsCount[5] > storeMicsCount[3] 
       && storeMicsCount[5] > storeMicsCount[4] && storeMicsCount[5] > storeMicsCount[6] && storeMicsCount[5] > storeMicsCount[7])
  {
    storeDirectionCoor = 6;
    micFFT = mic6;
    displayDirection("6", storeDirectionCoor);

    vibrationMotor(300);
  }
  else if(storeMicsCount[6] > storeMicsCount[0] && storeMicsCount[6] > storeMicsCount[1] && storeMicsCount[6] > storeMicsCount[2] && storeMicsCount[6] > storeMicsCount[3] 
       && storeMicsCount[6] > storeMicsCount[4] && storeMicsCount[6] > storeMicsCount[5] && storeMicsCount[6] > storeMicsCount[7])
  {
    storeDirectionCoor = 5; //7
    micFFT = mic7;
    displayDirection("7", storeDirectionCoor);

    vibrationMotor(300);
  }
  else if(storeMicsCount[7] > storeMicsCount[0] && storeMicsCount[7] > storeMicsCount[1] && storeMicsCount[7] > storeMicsCount[2] && storeMicsCount[7] > storeMicsCount[3] 
       && storeMicsCount[7] > storeMicsCount[4] && storeMicsCount[7] > storeMicsCount[5] && storeMicsCount[7] > storeMicsCount[6])
  {
    storeDirectionCoor = 4; //8
    micFFT = mic8;
    displayDirection("8", storeDirectionCoor);

    vibrationMotor(300);
  }
  else
  {
    storeDirectionCoor = 0;
    displayDirection("", storeDirectionCoor);
  }

  //displayDirection(String(storeDirectionCoor+1), storeDirectionCoor);
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

double getCorrelation(double micInputSumSquare, double sirenSumSquare, int sirenArray[arraySize])
{
  double sirenCorrelation = 0.00;

  for(int i = 0; i < arraySize; i++)
  {
    sirenCorrelation = sirenCorrelation + (micInput[i] * sirenArray[i]);
  }

  return (sirenCorrelation / sqrt(sirenSumSquare * micInputSumSquare));
}

void correlation()
{
  getFFT();
  
  //Sum of (input^2)
  //In the later loop
  /*for(int i = 0; i < arraySize; i++)
  {
    micInputSumSquare = micInputSumSquare + pow(micInput[i], 2);
  }*/

  //Correlation initialize
  double backUpAlarmNNCorrelation = 0.00;
  double backUpAlarmWNCorrelation = 0.00;
  double emergencyAlertSystem1Correlation = 0.00;
  double emergencyAlertSystem2Correlation = 0.00;
  
  double fireTruckHornCorrelation = 0.00;
  
  double smokeAlarmCorrelation = 0.00;
  double fireAlarmCorrelation = 0.00; 
  double tsunamiSirenCorrelation = 0.00;
  double nuclearAlarmCorrelation = 0.00;
  
  double policeSirenNNCorrelation = 0.00;
  double policeSirenWNCorrelation = 0.00;
  double fireTruckSirenNNCorrelation = 0.00;
  double fireTruckSirenWNCorrelation = 0.00; 
  
  double tornadoSirenNNCorrelation = 0.00;
  double tornadoSirenWNCorrelation = 0.00;
  
  double policeYelpNNCorrelation = 0.00;
  double policeYelpWNCorrelation = 0.00;
  double policeHornNNCorrelation = 0.00;
  double policeHornWNCorrelation = 0.00;
  
  double ambulanceSirenNNCorrelation = 0.00;
  double ambulanceSirenWNCorrelation = 0.00;

  for(int i = 0; i < arraySize; i++)
  {
    micInputSumSquare = micInputSumSquare + pow(micInput[i], 2); //Sum of (input^2) for the input
    
    backUpAlarmNNCorrelation = backUpAlarmNNCorrelation + (micInput[i] * backUpAlarmNN[i]);
    backUpAlarmWNCorrelation = backUpAlarmWNCorrelation + (micInput[i] * backUpAlarmWN[i]);
    emergencyAlertSystem1Correlation = emergencyAlertSystem1Correlation + (micInput[i] * emergencyAlertSystem1[i]);
    emergencyAlertSystem2Correlation = emergencyAlertSystem2Correlation + (micInput[i] * emergencyAlertSystem2[i]);
    
    fireTruckHornCorrelation = fireTruckHornCorrelation + (micInput[i] * fireTruckHorn[i]);
    
    smokeAlarmCorrelation = smokeAlarmCorrelation + (micInput[i] * smokeAlarm[i]);   
    fireAlarmCorrelation = fireAlarmCorrelation + (micInput[i] * fireAlarm[i]); 
    tsunamiSirenCorrelation = tsunamiSirenCorrelation + (micInput[i] * tsunamiSiren[i]);
    nuclearAlarmCorrelation = nuclearAlarmCorrelation + (micInput[i] * nuclearAlarm[i]);
    
    policeSirenNNCorrelation = policeSirenNNCorrelation + (micInput[i] * policeSirenNN[i]);
    policeSirenWNCorrelation = policeSirenWNCorrelation + (micInput[i] * policeSirenWN[i]);
    fireTruckSirenNNCorrelation = fireTruckSirenNNCorrelation + (micInput[i] * fireTruckSirenNN[i]);
    fireTruckSirenWNCorrelation = fireTruckSirenWNCorrelation + (micInput[i] * fireTruckSirenWN[i]);
    
    tornadoSirenNNCorrelation = tornadoSirenNNCorrelation + (micInput[i] * tornadoSirenNN[i]);
    tornadoSirenWNCorrelation = tornadoSirenWNCorrelation + (micInput[i] * tornadoSirenWN[i]);
    
    policeYelpNNCorrelation = policeYelpNNCorrelation + (micInput[i] * policeYelpNN[i]);
    policeYelpWNCorrelation = policeYelpWNCorrelation + (micInput[i] * policeYelpWN[i]);
    policeHornNNCorrelation = policeHornNNCorrelation + (micInput[i] * policeHornNN[i]);
    policeHornWNCorrelation = policeHornWNCorrelation + (micInput[i] * policeHornWN[i]);
    
    ambulanceSirenNNCorrelation = ambulanceSirenNNCorrelation + (micInput[i] * ambulanceSirenNN[i]);
    ambulanceSirenWNCorrelation = ambulanceSirenWNCorrelation + (micInput[i] * ambulanceSirenWN[i]);
  }

  //Initialize the normalized correlation
  double backUpAlarmNNNormCorrelation = backUpAlarmNNCorrelation / sqrt(backUpAlarmNNSumSquare * micInputSumSquare);
  double backUpAlarmWNNormCorrelation = backUpAlarmWNCorrelation / sqrt(backUpAlarmWNSumSquare * micInputSumSquare);
  double emergencyAlertSystem1NormCorrelation = emergencyAlertSystem1Correlation / sqrt(emergencyAlertSystem1SumSquare * micInputSumSquare);
  double emergencyAlertSystem2NormCorrelation = emergencyAlertSystem2Correlation / sqrt(emergencyAlertSystem2SumSquare * micInputSumSquare);
  
  double fireTruckHornNormCorrelation = fireTruckHornCorrelation / sqrt(fireTruckHornSumSquare * micInputSumSquare);
  
  double smokeAlarmNormCorrelation = smokeAlarmCorrelation / sqrt(smokeAlarmSumSquare * micInputSumSquare);
  double fireAlarmNormCorrelation = fireAlarmCorrelation / sqrt(fireAlarmSumSquare * micInputSumSquare);
  double tsunamiSirenNormCorrelation = tsunamiSirenCorrelation / sqrt(tsunamiSirenSumSquare * micInputSumSquare);
  double nuclearAlarmNormCorrelation = nuclearAlarmCorrelation / sqrt(nuclearAlarmSumSquare * micInputSumSquare);
  
  double policeSirenNNNormCorrelation = policeSirenNNCorrelation / sqrt(policeSirenNNSumSquare * micInputSumSquare);
  double policeSirenWNNormCorrelation = policeSirenWNCorrelation / sqrt(policeSirenWNSumSquare * micInputSumSquare); 
  double fireTruckSirenNNNormCorrelation = fireTruckSirenNNCorrelation / sqrt(fireTruckSirenNNSumSquare * micInputSumSquare);
  double fireTruckSirenWNNormCorrelation = fireTruckSirenWNCorrelation / sqrt(fireTruckSirenWNSumSquare * micInputSumSquare);
  
  double tornadoSirenNNNormCorrelation = tornadoSirenNNCorrelation / sqrt(tornadoSirenNNSumSquare * micInputSumSquare);
  double tornadoSirenWNNormCorrelation = tornadoSirenWNCorrelation / sqrt(tornadoSirenWNSumSquare * micInputSumSquare);
  
  double policeYelpNNNormCorrelation = policeYelpNNCorrelation / sqrt(policeYelpNNSumSquare * micInputSumSquare);
  double policeYelpWNNormCorrelation = policeYelpWNCorrelation / sqrt(policeYelpWNSumSquare * micInputSumSquare);
  double policeHornNNNormCorrelation = policeHornNNCorrelation / sqrt(policeHornNNSumSquare * micInputSumSquare);
  double policeHornWNNormCorrelation = policeHornWNCorrelation / sqrt(policeHornWNSumSquare * micInputSumSquare);
  
  double ambulanceSirenNNNormCorrelation = ambulanceSirenNNCorrelation / sqrt(ambulanceSirenNNSumSquare * micInputSumSquare);
  double ambulanceSirenWNNormCorrelation = ambulanceSirenWNCorrelation / sqrt(ambulanceSirenWNSumSquare * micInputSumSquare);
  
  //For debugging
  //Print results of correlation
  /*Serial.print(backUpAlarmNNNormCorrelation);
  Serial.print("   ");
  Serial.print(backUpAlarmWNNormCorrelation);
  Serial.print("   ");
  Serial.print(emergencyAlertSystem1NormCorrelation);
  Serial.print("   ");
  Serial.print(emergencyAlertSystem2NormCorrelation);
  Serial.print("   ");
  
  Serial.print(fireTruckHornNormCorrelation);
  Serial.print("   ");

  Serial.print(smokeAlarmNormCorrelation);
  Serial.print("   ");
  Serial.print(fireAlarmNormCorrelation);
  Serial.print("   ");
  Serial.print(tsunamiSirenNormCorrelation);
  Serial.print("   ");
  Serial.print(nuclearAlarmNormCorrelation);
  Serial.print("   ");
  
  Serial.print(policeSirenNNNormCorrelation);
  Serial.print("   ");
  Serial.print(policeSirenWNNormCorrelation);
  Serial.print("   ");
  Serial.print(fireTruckSirenNNNormCorrelation);
  Serial.print("   ");
  Serial.print(fireTruckSirenWNNormCorrelation);
  Serial.print("   ");
  
  Serial.print(tornadoSirenNNNormCorrelation);
  Serial.print("   ");
  Serial.print(tornadoSirenWNNormCorrelation);
  Serial.print("   ");
  
  Serial.print(policeYelpNNNormCorrelation);
  Serial.print("   ");
  Serial.print(policeYelpWNNormCorrelation);
  Serial.print("   ");
  Serial.print(policeHornNNNormCorrelation);
  Serial.print("   ");
  Serial.print(policeHornWNNormCorrelation);
  Serial.print("   ");

  Serial.print(ambulanceSirenNNNormCorrelation);
  Serial.print("   ");
  Serial.println(ambulanceSirenWNNormCorrelation);*/
  
  //Checks against correlation threshold if true print out the siren
  /*if (backUpAlarmNNNormCorrelation >= activeSiren)
    displaySiren("backUpAlarm_noNoise", 3, "Industrial", "Back Up", "Alarm");
  else if (backUpAlarmWNNormCorrelation >= activeSiren)
    displaySiren("backUpAlarm_withNoise", 3, "Industrial", "Back Up", "Alarm");
  //else if (emergencyAlertSystem1NormCorrelation >= 0.9)
  //displaySiren("emergencyAlertSystem1", 3, "National", "Emergency", "Alert");
  else if (emergencyAlertSystem2NormCorrelation >= 0.9)
    displaySiren("emergencyAlertSystem2", 3, "National", "Emergency", "Alert");
    
  else if (fireTruckHornNormCorrelation >= activeSiren)
    displaySiren("fireTruckHorn", 3, "Fire", "Truck", "Horn");*/
    
  if (smokeAlarmNormCorrelation >= activeSiren)
    displaySiren("smokeAlarmNew", 2, "Smoke", "Alarm", "No_text_here");
  else if (fireAlarmNormCorrelation >= activeSiren)
    displaySiren("fireAlarmNew", 2, "Fire", "Alarm", "No_text_here");
  /*else if (tsunamiSirenNormCorrelation >= activeSiren)
    displaySiren("tsunamiSiren", 3, "Fire", "Truck", "Siren"); //rename to fire trck siren
  else if (nuclearAlarmNormCorrelation >= 0.95)
    displaySiren("nuclearAlarm", 2, "Nuclear", "Alarm", "No_text_here");
    
  else if (policeSirenNNNormCorrelation >= .85)
    displaySiren("policeSiren_noNoise", 2, "Police", "Siren", "No_text_here");
  else if (policeSirenWNNormCorrelation >= 0.85)
    displaySiren("policeSiren_withNoise", 2, "Police", "Siren", "No_text_here");  
  else if (fireTruckSirenNNNormCorrelation >= activeSiren)
    displaySiren("fireTruckSiren_noNoise", 3, "Fire", "Truck", "Siren");
  else if (fireTruckSirenNNNormCorrelation >= activeSiren)
    displaySiren("fireTruckSiren_withNoise", 3, "Fire", "Truck", "Siren");
    
  /*else if (tornadoSirenNNNormCorrelation >= .85)
    displaySiren("tornadoSiren_noNoise", 2, "Tornado", "Siren", "No_text_here");
  else if (tornadoSirenNNNormCorrelation >= .85)
    displaySiren("tornadoSiren_withNoise", 2, "Tornado", "Siren", "No_text_here");
    
  else if (policeYelpNNNormCorrelation >= 0.9)
    displaySiren("policeYelp_noNoise", 2, "Police", "Siren", "No_text_here");
  else if (policeYelpWNNormCorrelation >= 0.9)
    displaySiren("policeYelp_withNoise", 2, "Police", "Siren", "No_text_here"); //Change name from yelp to siren
    
  else if (policeHornNNNormCorrelation >= activeSiren)
    displaySiren("policeHorn_noNoise", 2, "Police", "Horn", "No_text_here");
  else if (policeHornWNNormCorrelation >= activeSiren)
    displaySiren("policeHorn_withNoise", 2, "Police", "Horn", "No_text_here");

  else if (ambulanceSirenNNNormCorrelation >= activeSiren)
    displaySiren("ambulanceSiren_noNoise", 2, "Ambulance", "Siren", "No_text_here");
  else if (ambulanceSirenWNNormCorrelation >= activeSiren)
    displaySiren("ambulanceSiren_withNoise", 2, "Ambulance", "Siren", "No_text_here");*/

  //Reset
  micInputSumSquare = 0;
}

//Display a arrow that point to a direction
//The argument directionCoordinate defined where the arrow point
//on the screen
//1 - Northwest
//2 - North
//3 - Northeast
//4 - East
//5 - Southeast
//6 - South
//7 - Southwest
//8 - West
void displayDirection(String mic, int directionCoordinate)
{
  tft.fillCircle(63, 49, 40, ST7735_BLACK);

  switch (directionCoordinate)
  {
    case 1:
      //Northwest
      tft.fillTriangle(103, 49, 63, 89, 35, 21, ST7735_WHITE);
      tft.fillTriangle(103, 50, 64, 89, 70, 56, ST7735_BLACK);
      break;
    case 2:
      //North
      tft.fillTriangle(91, 77, 35, 77, 63, 9, ST7735_WHITE);
      tft.fillTriangle(90, 77, 36, 77, 63, 56, ST7735_BLACK);
      break;
    case 3:
      //Northeast
      tft.fillTriangle(63, 89, 23, 49, 91, 21, ST7735_WHITE);
      tft.fillTriangle(62, 89, 23, 50, 56, 56, ST7735_BLACK);
      break;
    case 4:
      //East
      tft.fillTriangle(35, 77, 35, 21, 103, 49, ST7735_WHITE);
      tft.fillTriangle(35, 76, 35, 22, 56, 49, ST7735_BLACK);
      break;
    case 5:
      //Southeast
      tft.fillTriangle(23, 49, 63, 9, 91, 77, ST7735_WHITE);
      tft.fillTriangle(23, 48, 62, 9, 56, 42, ST7735_BLACK);
      break;
    case 6:
      //South
      tft.fillTriangle(35, 21, 91, 21, 63, 89, ST7735_WHITE);
      tft.fillTriangle(36, 21, 90, 21, 63, 42, ST7735_BLACK);
      break;
    case 7:
      //Southwest
      tft.fillTriangle(63, 9, 103, 49, 35, 77, ST7735_WHITE);
      tft.fillTriangle(64, 9, 103, 48, 70, 42, ST7735_BLACK);
      break;
    case 8:
      //West
      tft.fillTriangle(91, 21, 91, 77, 23, 49, ST7735_WHITE);
      tft.fillTriangle(91, 22, 91, 76, 70, 49, ST7735_BLACK);
      break;
    default:
      //Serial.println("No input for the direction");
      //tft.fillCircle(63, 49, 40, ST7735_BLACK);
      break;
  }

  //For debugging
  Serial.print("Mic ");
  Serial.print(mic);
  Serial.println(" is the loudest");
}

//Display the siren name and print to the screen
//inputString is the name of the siren and display on the consol
//numwords is how many word need to be display on the screen
//str1 hole the first word, str2 hold the second word, and str3 hold the third word
void displaySiren(String inputString, int numWords, String str1, String str2, String str3)
{
  //tft.fillRect(0, 76, 128, 84, ST7735_BLACK);

  tft.setTextSize(2);

  for (int i = 0; i < 2; i++)
  {
    if (i == 0)
      tft.fillScreen(ST7735_BLUE);
    else
      tft.fillScreen(ST7735_RED);

    tft.fillCircle(63, 49, 44, ST7735_BLACK);
    tft.drawCircle(63, 49, 44, ST7735_WHITE);

    displayDirection("", storeDirectionCoor);
    
    switch (numWords)
    {  
      case 2:
        tft.setCursor(0, 100);
        tft.print(str1);
        tft.setCursor(0, 120);
        tft.print(str2);
        break;
      case 3:
        tft.setCursor(0, 100);
        tft.print(str1);
        tft.setCursor(0, 120);
        tft.print(str2);
        tft.setCursor(0, 140);
        tft.print(str3);
        break;
      default:
        //tft.fillRect(0, 99, 61, 127, ST7735_BLACK);
        break;
    }

    //For debugging
    Serial.println(inputString);

    vibrationMotor(1000);
  
    //delay(500);
  }

  tft.fillScreen(ST7735_BLACK);
  tft.drawCircle(63, 49, 44, ST7735_WHITE);
}

//Turn the viration motor on and off
//The time the vibration motor stay on is
//depend on the motorRunTime
void vibrationMotor(int motorRunTime)
{
  digitalWrite(vibrationMotorPin,HIGH);
  delay(motorRunTime);
  digitalWrite(vibrationMotorPin,LOW);
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

  delay(250);
  
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
