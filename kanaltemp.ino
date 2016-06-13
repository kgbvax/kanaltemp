#include <RTCZero.h>

RTCZero rtc;

#define DEBUG
#include <Sodaq_RN2483.h>

#include <OneWire.h>                              
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 1

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define debugSerial SerialUSB
#define loraSerial Serial1

#define DEB(x) debugSerial.println(x);
//#define DEB(x)


const int theUpdateRate=30000;

/* use your own keys! */
const uint8_t devAddr[4] ={ 0x35, 0xAF, 0x16, 0xBE };
const uint8_t nwkSKey[16] = { 0x02, 0xB0, 0x7E, 0x01, 0x50, 0x16, 0x02, 0x80, 0xAE, 0x0D, 0x20, 0xA6, 0x0A, 0xB0, 0xF7, 0x01 };
const uint8_t appSKey[16] = { 0x00, 0x20, 0xCD, 0x02, 0x70, 0xB1, 0x06, 0x40, 0xB0, 0x09, 0x80, 0x56, 0x0E, 0x40, 0x68, 0x06 };
 
 

void setup()
{
  // Turn the LoRaBee on
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
 
  delay(5000);
  
  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  
   
  //LoRaBee.setDiag(debugSerial);
 
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true)) {
    DEB("Connection to the network was successful.");
   } else {
    DEB("Connection to the network failed!"); 
  }

  sensors.begin(); 
  rtc.begin();
  
}

void alarmMatch()
{
  DEB("Alarm Match!");
}

void lowPowerSleep(uint16_t sec) {
   
 /*
  DEB("putting 2483 to sleep");
  LoRaBee.sleep(59000); //have the LoRa Module wakt up a bit early */
  rtc.setTime(0, 0, 0);
  rtc.setAlarmTime(0,sec/60,sec%60);
  rtc.attachInterrupt(&alarmMatch);
  rtc.enableAlarm(rtc.MATCH_HHMMSS); 
  DEB("standby");
  rtc.standbyMode();  //sleep....
  DEB("return from standby"); 
}

void loop()
{
   
    sensors.requestTemperatures();  
    float temp=sensors.getTempCByIndex(0);
     
    int16_t transmit=(int16_t) 10*temp; 
    DEB(transmit);
  
    uint8_t payload[1];
    //highbyte lowbyte
    payload[0]=(uint8_t) (transmit >>8);
    payload[1]=(uint8_t)  0xff & transmit;
    
    switch (LoRaBee.send(1, payload, 2))
    {
      case NoError:
        DEB("/");
        break;
      case NoResponse:
        DEB("There was no response from the device.");
        break;
      case Timeout:
        DEB("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
        delay(20000);
        break;
      case PayloadSizeError:
        DEB("The size of the payload is greater than allowed. Transmission failed!");
        break;
      case InternalError:
        DEB("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe program will now halt.");
        while (1) {};
        break;
      case Busy:
        DEB("The device is busy. Sleeping for 10 extra seconds.");
        delay(10000);
        break;
      case NetworkFatalError:
        DEB("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
        while (1) {};
        break;
      case NotConnected:
        DEB("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
        while (1) {};
        break;
      case NoAcknowledgment:
        DEB("There was no acknowledgment sent back!");
        break;
      default:
        break;
    }
    lowPowerSleep(10);
 }


