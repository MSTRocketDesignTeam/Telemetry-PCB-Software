/* Missouri S&T Rocket Design Team XBee Telemetry System
 * By Thomas Francois, Updated 7/6/2021
 * Uses TinyGPS++ and AltSoftSerial Libraries
 * Interprets NMEA Strings that arrive via SoftSerial pins
 * and sends relevant data out hardware serial pins to
 * XBee Tranceiver. Xbees pairs must be configued to function
 * on the same channel and in transparent mode. BerryGPS units
 * should output 9600 baud, XBee serial configured for 19200
 * baud due to compatibility with AltSoftSerial. GPS receiver
 * should be connected to pins 5(TX) and 6(RX) of the arduino,
 * and the XBee should connect to pins 8(RX) and 9(TX) of
 * the arduino.
*/

#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

#include <avr/wdt.h>

#define RXPin 6
#define TXPin 5

#define GPSBaud 9600
#define XBeeBaud 19200
#define GPSBaudHighSpeed 115200

#define standardRefresh 4017
#define highSpeedRefresh 1003

#define S0 2
#define S1 3

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the XBee
//SoftwareSerial ss(RXPin, TXPin);
//AltSoftSerial defines RX(Pin 8) and TX(Pin 9)
AltSoftSerial ss;

// A sample NMEA stream for debug purposes
/*
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C,0r,0n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62,0r,0n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77,0r,0n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C,0r,0n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D,0r,0n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F,0r,0n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F,0r,0n";
*/
int lastReceivedChars = 0;
int timeLastReceived = 0;

int highPinState = 0;
int lowPinState = 0;

int systemStatus = 0;
bool gpsHighSpeed = false;
int refreshSpeed = standardRefresh;

byte gpsSleepMsg[16] =        {0xB5,0x62,0x06,0x57,0x08,0x00,0x01,0x00,0x00,0x00,0x50,0x4F,0x54,0x53,0xAC,0x85};
byte gpsAwakeMsg[16] =        {0xB5,0x62,0x06,0x57,0x08,0x00,0x01,0x00,0x00,0x00,0x20,0x4E,0x55,0x52,0x7B,0xC3};
byte gpsResetMsg[12] =        {0xb5,0x62,0x06,0x04,0x04,0x00,0xff,0xff,0x02,0x00,0x0e,0x61};

//Can't seem to get these to work quite yet.
byte gps10Hz[14] =            {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};
byte gps2Hz[14] =             {0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77};
byte gpsHighSpeedSerial[28] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};
byte gpsLowSpeedSerial[14] =  {0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77};

void setup()
{
  //MCUSR = 0;  //Clears reset flags
  ss.begin(XBeeBaud);
  Serial.begin(GPSBaud);
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  highPinState = digitalRead(S1);
  lowPinState = digitalRead(S0);

  ss.print(F("**** "));

  if(!highPinState && !lowPinState)
    ss.print(F("RKT "));
  else if(!highPinState && lowPinState)
   ss.print(F("PL1 "));
  else if(highPinState && !lowPinState)
   ss.print(F("PL2 "));
 else if(highPinState && lowPinState)
   ss.print(F("PL3 "));
 ss.print(F("STARTUP, ENTERING STANDBY MODE"));
 ss.println(F(": END STATUS MESSAGE*******"));
}

static void printFloat(float val, bool valid, int len, int prec);
static void printInt(unsigned long val, bool valid, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printStr(const char *str, int len);
static void smartDelay(unsigned long ms);

void changeGPSSpeed(bool newSpeed);

void reset();

void loop()
{
  //Loop for receiving commands from ground station 
  while(ss.available() > 0)
  {
    char incomingByte = ss.read();
    ss.print(F("\r******DEVICE "));
    if(!highPinState && !lowPinState)
      ss.print(F("RKT "));
    else if(!highPinState && lowPinState)
      ss.print(F("PL1 "));
    else if(highPinState && !lowPinState)
      ss.print(F("PL2 "));
    else if(highPinState && lowPinState)
      ss.print(F("PL3 "));
    switch(incomingByte) {
      case '1':
      {
        if(systemStatus == 1)
        {
          systemStatus = 1;
          ss.print(F("MODULE IS ALREADY RUNNING"));
          break;
        }
        else if(systemStatus >= -1)
        {
          systemStatus = 1;
          ss.print(F("MODULE IS RESUMING OPERATION"));
        }
      }
      case 'p':
      {
        ss.print(F("MODULE: WAKING GPS MODULE FROM SLEEP"));
        ss.println("");        
        Serial.write(gpsAwakeMsg, sizeof(gpsAwakeMsg));
        break;
      }
      case '9':
      {
        if(systemStatus == 1)
          ss.print(F("MODULE IS ENTERING STANDBY MODE"));
        else
          ss.print(F("MODULE IS ALREADY IN STANDBY MODE"));
        systemStatus = 0;
        break;
      }
      case '0':
      {
        if(systemStatus >= 0)
        {
          ss.print(F("MODULE IS ENTERING DEEP STANDBY MODE"));
          ss.println("");
        }
        systemStatus = -1;
      }
      case 'q':
      {
        ss.print(F("MODULE: PUTTING GPS MODULE TO SLEEP"));      
        Serial.write(gpsSleepMsg, sizeof(gpsSleepMsg));
        break;
      }
      case 'h':
      {
        ss.print(F("MODULE: ENABLING GPS HIGH SPEED DISPLAY"));      
        changeGPSSpeed(true);
        break;
      }
      case 'l':
      {
        ss.print(F("MODULE: ENABLING GPS LOW SPEED DISPLAY"));   
        changeGPSSpeed(false);   
        break;
      }
      case 'r':
      {
        ss.print(F("MODULE: RESETTING GPS MODULE"));
        Serial.write(gpsResetMsg, sizeof(gpsResetMsg));
        break;      
      }
      case '\\':
      {
        ss.println(F("MODULE: RESETTING MCU"));
        smartDelay(500);
        reset();
        break;
      }
      case '.':
      {
        ss.println("");
        ss.println(F("Dev,Sats,Latitude ,Longitude ,Date (UTC),Time    ,Alt   ,Course,Speed ,Chars,Sentences,     "));
        ss.println(F("Nme      (deg)     (deg)      MM/DD/YEAR HH:MM:SS (m)    (o)    (mps)  RX(k) RX(k)          "));
        ss.println(F("--------------------------------------------------------------------------------------------"));
        break;
      }
      default :
        ss.print(F("UNRECOGNIZED COMMAND"));
    }
    ss.println(F(": END STATUS MESSAGE*******"));
  }
  if(systemStatus == 1)
  {
    //Error detection: If no characters arrive from the GPS unit for 31 seconds,
    //something is wrong and should notify users via the XBee
    if(!((millis())%31700) && gps.charsProcessed()%1000==lastReceivedChars%1000)
    {
      ss.print(F("***WARNING MESSAGE: DEVICE "));
      if(!highPinState && !lowPinState)
       ss.print("RKT ");
      else if(!highPinState && lowPinState)
       ss.print("PL1 ");
      else if(highPinState && !lowPinState)
       ss.print("PL2 ");
     else if(highPinState && lowPinState)
       ss.print("PL3 ");
  
      ss.print(F("NOT RECEIVING GPS DATA FOR "));
      printInt((millis()-timeLastReceived)/1000, true, 4);
      ss.println(F("SECS, CHECK GPS CONNECTIONS!**"));
      smartDelay(500);
    }
    //Heartbeat Program, send received character message every 10 seconds to indicate system is active
    if(!(millis()%10000))
    {
      //ss.println(F("Sats,Latitude ,Longitude ,   Date   ,  Time  ,  Alt ,Course,Speed ,Chars,Sentences,Checksum "));
      //ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS   (m)   (o)   (mps)  RX    RX        Fails    "));
      
      ss.print(F("*DEVICE "));
      if(!highPinState && !lowPinState)
       ss.print("RKT ");
      else if(!highPinState && lowPinState)
       ss.print("PL1 ");
      else if(highPinState && !lowPinState)
       ss.print("PL2 ");
     else if(highPinState && lowPinState)
       ss.print("PL3 ");    
  
      ss.print(F("HEARTBEAT MESSAGE: GPS Characters Rx: "));
      printInt((gps.charsProcessed()), true, 20);
      ss.println(F(" So Far:END MESSAGE***"));
      smartDelay(500);
    }
  
  
    //Heartbeat Program, send data format message every 58 seconds to indicate system is active
    if(!(millis()%58000))
    {
      ss.println(F("***SYSTEM HEARTBEAT MESSAGE: 1 Min Passed, Sending Data Str Format :END HEARTBEAT MESSAGE***"));
      ss.println(F("Dev,Sats,Latitude ,Longitude ,Date (UTC),Time    ,Alt   ,Course,Speed ,Chars,Sentences,     "));
      ss.println(F("Nme      (deg)     (deg)      MM/DD/YEAR HH:MM:SS (m)    (o)    (mps)  RX(k) RX(k)          "));
      
      ss.println(F("--------------------------------------------------------------------------------------------"));
      smartDelay(500);
    }
    
    //"Feeding" the gps object with NMEA data as it arrives
    while (Serial.available() > 0)
    {
      gps.encode(Serial.read());
      timeLastReceived = millis();
  
    //Formatting:
    //Dev ,Sats,Latitude ,Longitude ,Date      ,Time    ,Alt   ,Course,Speed ,Chars,Sentences,    
    //Name      (deg)     (deg)      MM/DD/YEAR HH:MM:SS (m)    (o)    (mps)  RX    RX            
    
    //Device Name (3 Characters)
    if(!highPinState && !lowPinState)
      ss.print("RKT,");
    else if(!highPinState && lowPinState)
      ss.print("PL1,");
    else if(highPinState && !lowPinState)
      ss.print("PL2,");
    else if(highPinState && lowPinState)
      ss.print("PL3,");
    //Satellites (4 characters)
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    //Location (20 characters)
      printFloat(gps.location.lat(), gps.location.isValid(), 10, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 11, 6);
    //Date & TimeUTC (19 characters w/ internal comma)
      printDateTime(gps.date, gps.time);
    //Altitude in m (6 characters)
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    //Course (6 characters)
      printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    //Speed (6 characters)
      printFloat(gps.speed.mps(), gps.speed.isValid(), 7, 2);
    //Total Characters Received (12 characters)
      printFloat((gps.charsProcessed()/1000), true, 13, 1);
    //Total Sentences Received (7 characters)
      printFloat((gps.sentencesWithFix()/1000), true, 8, 1);
      ss.println();
      smartDelay(refreshSpeed);
    }
    lastReceivedChars = gps.charsProcessed();
  }
  else if(!(millis()%15000))
    //Heartbeat Program, send time message every 15 seconds to indicate system is active
  {
    //ss.println(F("Sats,Latitude ,Longitude ,   Date   ,  Time  ,  Alt ,Course,Speed ,Chars,Sentences,Checksum "));
    //ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS   (m)   (o)   (mps)  RX    RX        Fails    "));
    ss.print(F("*DEVICE "));
    if(!highPinState && !lowPinState)
     ss.print("RKT ");
    else if(!highPinState && lowPinState)
     ss.print("PL1 ");
    else if(highPinState && !lowPinState)
     ss.print("PL2 ");
   else if(highPinState && lowPinState)
     ss.print("PL3 ");

    ss.print(F("HEARTBEAT MESSAGE: System Time is "));
    printInt(millis()/1000, true, 9);
    ss.println(F(" Sec from boot:END HEARTBEAT MESSAGE*"));
    smartDelay(500);
  }
}
//Function used to change the speed GPS data is displayed on the screen
void changeGPSSpeed(bool newSpeed)
{
  if(newSpeed)
  {
    //Serial.write(gpsHighSpeedSerial, sizeof(gpsHighSpeedSerial));
    //Serial.end();
    //Serial.begin(GPSBaudHighSpeed);
    //Serial.write(gps10Hz, sizeof(gps10Hz));
    refreshSpeed = highSpeedRefresh;
  }
  else
  {
    //Serial.write(gpsLowSpeedSerial, sizeof(gpsLowSpeedSerial));
    //Serial.end();
    //Serial.begin(GPSBaud);
    //Serial.write(gps2Hz, sizeof(gps2Hz));
    refreshSpeed = standardRefresh;
  }
}
void reset()
{
  wdt_enable(WDTO_15MS);
  for(;;){
  }
}

//By Mikal Hart from TinyGPS++ FullExample Code
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}
//By Mikal Hart from TinyGPS++ FullExample Code
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      ss.print('*');
    ss.print(',');
  }
  else
  {
    ss.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      ss.print(',');
  }
  smartDelay(0);
}
//By Mikal Hart from TinyGPS++ FullExample Code
static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ',';
  if (len > 0) 
    sz[len-1] = ',';
  ss.print(sz);
  smartDelay(0);
}
//By Mikal Hart from TinyGPS++ FullExample Code
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    ss.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d,", d.month(), d.day(), d.year());
    ss.print(sz);
  }
  
  if (!t.isValid())
  {
    ss.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d,", t.hour(), t.minute(), t.second());
    ss.print(sz);
  }

  //printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
//By Mikal Hart from TinyGPS++ FullExample Code
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    ss.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
