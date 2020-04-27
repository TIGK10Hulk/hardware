#include "MeAuriga.h"
#include <SoftwareSerial.h>

MeBluetooth bluetooth(PORT_3);

unsigned char table[128] = {0};

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(115200);    //The factory default baud rate is 115200
  Serial.println("Bluetooth Start!");
}

void loop()
{ 
  int readdata = 0,i = 0,count = 0;
  char outDat;
  if (bluetooth.available())
  {
    while((readdata = bluetooth.read()) != (int)-1)
    {
      table[count] = readdata;
      count++;
      delay(1);
    }
    for(i = 0;i<count;i++)
    {
      Serial.write(table[i]);
    }
  }
  if(Serial.available() )
  {
    outDat = Serial.read();
    bluetooth.write(outDat);
  }
}
