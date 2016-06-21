#include <Arduino.h>

#include <Sodaq_RN2483.h>
#define DEBUG
//#define DUMMY_TEMP 7

static int theUpdateRate =   15; //sec

#define debugSerial SerialUSB
#define loraSerial Serial1

 
#ifdef DEBUG
#define DEB(x) debugSerial.println(x);
#else
#define DEB(x)
#endif

  
 
 
/* use your own keys! */
const uint8_t devAddr[4] = { 0x72, 0xB4, 0x14, 0x87 };
const uint8_t nwkSKey[16] = { 0x53, 0x82, 0xF8, 0x51, 0xCD, 0x26, 0xA8, 0xC6, 0xD2, 0x31, 0xF0, 0x49, 0xF9, 0x07, 0xA7, 0xBB };
const uint8_t appSKey[16] = { 0x75, 0x6B, 0xD6, 0x59, 0xA9, 0x74, 0x25, 0xF0, 0xB3, 0x97, 0xA0, 0xE8, 0x78, 0x7C, 0x99, 0xF0 };

void setup()
{
  // Turn the LoRaBee on
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
 
  

#ifdef DEBUG
  debugSerial.begin(57600);
#else
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  USBDevice.detach();
#endif

  delay(10000);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

#ifdef DEBUG
  LoRaBee.setDiag(debugSerial);
#endif
  
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true)) {
    DEB("Connection to the network was successful.");
  } else {
    DEB("Connection to the network failed!");
  }
 
}
 
 

void loop()
{
  
  uint8_t payload[1];
   
   uint8_t res = LoRaBee.send(1, payload, sizeof(payload));
 
  switch (res)
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
      delay(30000);
      while(true);  //goodbye
      break;
    case Busy:
      DEB("The device is busy. Sleeping for 10 extra seconds.");
      delay(60000);
      break;
    case NetworkFatalError:
      DEB("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      delay(60000);
      //autonomoReset();
       while(true);  //goodbye

      break;
    case NotConnected:
      DEB("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      delay(60000);
     // autonomoReset();
           while(true);  //goodbye

      break;
    case NoAcknowledgment:
      DEB("There was no acknowledgment sent back!");
      break;
    default:
      break;
  }
  delay(theUpdateRate*1000);
   
}

 
