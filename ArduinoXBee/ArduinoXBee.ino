/* Missouri S&T Rocket Design Team XBee Telemetry System
 * By Thomas Francois, Updated 7/5.y/2021
 * Uses TinyGPS++ and AltSoftSerial Libraries
 * Interprets NMEA Strings that arrive via SoftSerial pins
 * and sends relevant data out hardware serial pins to
 * XBee Tranceiver. Xbees pairs must be configued to function
 * on the same channel and in transparent mode. GPS units
 * always output 4800 baud, XBee serial configured for 19200
 * baud due to compatibility with AltSoftSerial. GPS receiver
 * should be connected to pins 3(TX) and 4(RX) of the arduino,
 * and the XBee should connect to pins 8(RX) and 9(TX) of
 * the arduino.
*/

#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

#define RXPin 4
#define TXPin 3
#define GPSBaud 9600
#define XBeeBaud 19200

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the XBee
//SoftwareSerial ss(RXPin, TXPin);
//AltSoftSerial defines RX(Pin 8) and TX(Pin 9)
AltSoftSerial ss;

// A sample NMEA stream for debug purposes
/*
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";
*/
int lastReceivedChars = 0;
int timeLastReceived = 0;
char systemMessageType = 0;

void setup()
{
  ss.begin(XBeeBaud);
  Serial.begin(GPSBaud);
  ss.println(F("****SYSTEM STARTUP****"));
  ss.println(F("Missouri S&T Rocket XBee Telemetry using TinyGPS++"));
  ss.println(F("by Thomas Francois with modifications of TinyGPS++ Code Examples"));
  ss.println();
  ss.println(F("Sats,Latitude ,Longitude ,Date      ,Time    ,Alt   ,Course,Speed ,Chars,Sentences,Checksum "));
  ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS (m)    (o)    (mps)  RX    RX        Fails    "));
  ss.println(F("--------------------------------------------------------------------------------------------"));
  
}
static void printFloat(float val, bool valid, int len, int prec);
static void printInt(unsigned long val, bool valid, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printStr(const char *str, int len);
static void smartDelay(unsigned long ms);
void loop()
{
  //Error detection: If no characters arrive from the GPS unit for 31 seconds,
  //something is wrong and should notify users via the XBee
  if(!((millis())%31000) && gps.charsProcessed()==lastReceivedChars)
  {
    ss.print(F("***SYSTEM STATUS MESSAGE: NO GPS SERIAL DATA RECEIVED IN "));
    printInt((millis()-timeLastReceived)/1000, true, 4);
    ss.println(F("SECS, CHECK GPS CONNECTIONS!***"));
    smartDelay(500);
  }
  //Heartbeat Program, send time message every 5 seconds to indicate system is active
  if(!(millis()%5000) && systemMessageType==0)
  {
    //ss.println(F("Sats,Latitude ,Longitude ,   Date   ,  Time  ,  Alt ,Course,Speed ,Chars,Sentences,Checksum "));
    //ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS   (m)   (o)   (mps)  RX    RX        Fails    "));
    ss.print(F("***SYSTEM HEARTBEAT MESSAGE: System Time is "));
    printInt(millis()/1000, true, 5);
    ss.println(F(" Seconds from boot:END HEARTBEAT MESSAGE***"));
    smartDelay(500);
    systemMessageType = 1;
  }

  //Heartbeat Program, send received character message every 5 seconds to indicate system is active
  if(!(millis()%5000) && systemMessageType==1)
  {
    //ss.println(F("Sats,Latitude ,Longitude ,   Date   ,  Time  ,  Alt ,Course,Speed ,Chars,Sentences,Checksum "));
    //ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS   (m)   (o)   (mps)  RX    RX        Fails    "));
    ss.print(F("***SYSTEM HEARTBEAT MESSAGE: GPS Characters Received: "));
    printInt(gps.charsProcessed(), true, 6);
    ss.println(F(" So Far:END HEARTBEAT MESSAGE***"));
    smartDelay(500);
    systemMessageType = 0;
  }


  //Heartbeat Program, send data format message every 58 seconds to indicate system is active
  if(!(millis()%58000))
  {
    ss.println(F("***SYSTEM HEARTBEAT MESSAGE: 1 Min Passed, Sending Data Str Format :END HEARTBEAT MESSAGE***"));
    ss.println(F("Sats,Latitude ,Longitude ,Date      ,Time    ,Alt   ,Course,Speed ,Chars,Sentences,Checksum "));
    ss.println(F("     (deg)     (deg)      MM/DD/YEAR HH:MM:SS (m)    (o)    (mps)  RX    RX        Fails    "));
    ss.println(F("--------------------------------------------------------------------------------------------"));
    smartDelay(500);
  }
  
  //"Feeding" the gps object with NMEA data as it arrives
  while (Serial.available() > 0)
  {
    gps.encode(Serial.read());
    timeLastReceived = millis();

  //Formatting:
  //Sats,Latitude,  Longitude,Date & Time in UTC ,Alt   ,Course,Speed ,Chars,Sentences,Checksum
  //     (deg)      (deg)                        (m)    (o)     (mps)  RX    RX        Fails
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
  //Total Characters Received (5 characters)
    printInt(gps.charsProcessed(), true, 6);
  //Total Sentences Received (9 characters)
    printInt(gps.sentencesWithFix(), true, 10);
  //Total Failed GPS Checksums (8 characters)
    printInt(gps.failedChecksum(), true, 9);
    ss.println();
    smartDelay(217);
  }

  lastReceivedChars = gps.charsProcessed();
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
