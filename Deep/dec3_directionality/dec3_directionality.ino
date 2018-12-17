
//#include <cmath>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
  directionality();
    
}

void directionality()
{
    int micScore[9] = {0};
    int sampsFromMics[9][100] = {0};
    int numOfSamps = 100;
    for(int x = 0; x < numOfSamps; x++)
    {
        sampsFromMics[1][x] = analogRead(A0);
        sampsFromMics[2][x] = analogRead(A1);
        sampsFromMics[3][x] = analogRead(A2);
        sampsFromMics[4][x] = analogRead(A3);
        sampsFromMics[5][x] = analogRead(A4);
        sampsFromMics[6][x] = analogRead(A5);
        sampsFromMics[7][x] = analogRead(A6);
        sampsFromMics[8][x] = analogRead(A7);
        Serial.print("*");
    }

    int avgMicVal[9];
    for(int y = 1; y < 9; y++)
    {
        for(int x = 0; x < numOfSamps; x++)
        {
            avgMicVal[y] = avgMicVal[y] + sampsFromMics[y][x];
        }
        avgMicVal[y] = avgMicVal[y]/numOfSamps;
    }

    for(int x = 0; x < numOfSamps; x++)
    {
        sampsFromMics[1][x] = sampsFromMics[1][x] - avgMicVal[1];
        sampsFromMics[2][x] = sampsFromMics[2][x] - avgMicVal[2];
        sampsFromMics[3][x] = sampsFromMics[3][x] - avgMicVal[3];
        sampsFromMics[4][x] = sampsFromMics[4][x] - avgMicVal[4];
        sampsFromMics[5][x] = sampsFromMics[5][x] - avgMicVal[5];
        sampsFromMics[6][x] = sampsFromMics[6][x] - avgMicVal[6];
        sampsFromMics[7][x] = sampsFromMics[7][x] - avgMicVal[7];
        sampsFromMics[8][x] = sampsFromMics[8][x] - avgMicVal[8];
    }


    for(int x = 0; x < numOfSamps; x++)
    {
        sampsFromMics[1][x] = sampsFromMics[1][x] - avgMicVal[1];
        sampsFromMics[2][x] = sampsFromMics[2][x] - avgMicVal[2];
        sampsFromMics[3][x] = sampsFromMics[3][x] - avgMicVal[3];
        sampsFromMics[4][x] = sampsFromMics[4][x] - avgMicVal[4];
        sampsFromMics[5][x] = sampsFromMics[5][x] - avgMicVal[5];
        sampsFromMics[6][x] = sampsFromMics[6][x] - avgMicVal[6];
        sampsFromMics[7][x] = sampsFromMics[7][x] - avgMicVal[7];
        sampsFromMics[8][x] = sampsFromMics[8][x] - avgMicVal[8];
    }

    for(int x = 0; x < numOfSamps; x++)
    {
        sampsFromMics[1][x] = pow(sampsFromMics[1][x], 2);
        sampsFromMics[2][x] = pow(sampsFromMics[2][x], 2);
        sampsFromMics[3][x] = pow(sampsFromMics[3][x], 2);
        sampsFromMics[4][x] = pow(sampsFromMics[4][x], 2);
        sampsFromMics[5][x] = pow(sampsFromMics[5][x], 2);
        sampsFromMics[6][x] = pow(sampsFromMics[6][x], 2);
        sampsFromMics[7][x] = pow(sampsFromMics[7][x], 2);
        sampsFromMics[8][x] = pow(sampsFromMics[8][x], 2);
    }
    
    int threshold = 4;
    threshold = pow(threshold, 2);

    for(int y = 1; y < 9; y++)
    {
        for(int x = 0; x < numOfSamps; x++)
        {
           if(sampsFromMics[y][x] < threshold)
           {
               sampsFromMics[y][x] = 0;
           }
        }
    }

    for(int y = 1; y < 9; y++)
    {
        for(int x = 0; x < numOfSamps; x++)
        {
           if(sampsFromMics[y][x] != 0)
           {
               sampsFromMics[y][x] = 0;
           }else
           {
               break;
           }  
        }
    }

    int micPosition[9];
    bool foundFirst = false;

    for(int x = 0; x < numOfSamps; x++)
    {
        for(int y = 1; y < 9; y++)
        {
            if(sampsFromMics[y][x] != 0)
            {
                micPosition[y] = x;
                foundFirst = true;
            }
        }
        if(foundFirst = true)
        {
            break;
        }
    }


    int lowestPos = numOfSamps;
    for(int y = 1; y < 9; y++)
    {
        if(micPosition[y] < lowestPos)
        {
            lowestPos = micPosition[y];
        }
    }

    
    //int micScore[9] = {0};
    for(int y = 1; y < 9; y++)
    {
        if(micPosition[y] == lowestPos)
        {
            micScore[y]++;
        }
    }

    for(int y = 1; y < 9; y++)
    {
        Serial.print(micScore[y]);
    }
    Serial.println("Done");




    



}
