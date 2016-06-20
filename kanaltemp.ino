#include <Arduino.h>
#include <RTCZero.h>
#include <Sodaq_RN2483.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <SPI.h>


RTCZero rtc;

//#define DEBUG
#define DUMMY_TEMP 7


//Coloumb counter defines
#define INT_LINE 4
#define POL_LINE 5

static int theUpdateRate =  10 * 60; //sec

#define debugSerial SerialUSB
#define loraSerial Serial1
#define ONE_WIRE_BUS 3

const static uint16_t recv_buffer_sz = 32;

#ifdef DEBUG
#define DEB(x) debugSerial.println(x);
#else
#define DEB(x)
#endif

#ifndef DEBUG
#define SEND_LED(x) digitalWrite(LED_BUILTIN,x)
#else
#endif  SEND_LED(x)

enum Commands {
  CMD_NOP =0x00,             // payload:none; no-operation
  CMD_SET_UPDATE_RATE =0x01, //payload: uint8 - new update rate in minutes
  CMD_GET_STATS = 0x02,      //payload: none; replies with some statistics (not implemented)
  CMD_RESET=0xff             //payload: none; board performs reset (not implemented)
};

static int32_t ccsum = 0;
//interrupt flags
static bool intCc = false; //coulomb counter interrupt
static bool intAlarm = false; // RTC alarm interrupt

static struct  {
   uint16_t messagesTransmitted = 0;
} kanalStats;

static unsigned long last_cc_tick = 0;
static float last_pwr_consumption = 0;
static uint16_t numTicks = 0;
static uint32_t totalTickTime = 0;
static bool direction; //true=charge

static uint8_t recieveBuffer[recv_buffer_sz]; //for incoming LoRa messages

/* use your own keys! */
const uint8_t devAddr[4] = { 0x35, 0xAF, 0x16, 0xBE };
const uint8_t nwkSKey[16] = { 0x02, 0xB0, 0x7E, 0x01, 0x50, 0x16, 0x02, 0x80, 0xAE, 0x0D, 0x20, 0xA6, 0x0A, 0xB0, 0xF7, 0x01 };
const uint8_t appSKey[16] = { 0x00, 0x20, 0xCD, 0x02, 0x70, 0xB1, 0x06, 0x40, 0xB0, 0x09, 0x80, 0x56, 0x0E, 0x40, 0x68, 0x06 };


void setup()
{
  // Turn the LoRaBee on
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
 
  //coulomb counter
  pinMode(INT_LINE, INPUT);
  pinMode(POL_LINE, INPUT);
  attachInterrupt(INT_LINE, ccisr, LOW);

#ifdef DEBUG
  debugSerial.begin(57600);
#else
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  USBDevice.detach();
#endif

  delay(10000);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  //LoRaBee.setDiag(debugSerial);
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true)) {
    DEB("Connection to the network was successful.");
  } else {
    DEB("Connection to the network failed!");
  }

  rtc.begin();
  last_cc_tick = millis();

  DFlashUltraDeepSleep();  //send SPI flash to ultra deep sleep
}

void alarmMatch()
{
  intAlarm = true;
}

/* put the RN2483 and the board to sleep
   min sleep time is 1 sec
   max sleep time is 00:59:59
*/
void lowPowerSleep(uint16_t sec) {
  if (sec < 1)
    sec = 1;

  DEB("LoRa to sleep");
  LoRaBee.sleep(1000 * sec - 500); //have the LoRa Module wake up 500msec  early */

  rtc.setTime(0, 0, 0);
  rtc.setAlarmTime(0, sec / 60, sec % 60);
  rtc.attachInterrupt(&alarmMatch);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  DEB("standby");
  do {
    intCc = false;
    rtc.standbyMode();  //sleep....
  } while (intCc == true || !intAlarm);  // sleep again until source is rtc alarm

  DEB("return from standby");
}

/*
     Coulumb Counter interrupt.
    Reads direction (POL) and increments / decrements global ccsum accordingly.
*/
void ccisr()
{
  intCc = true;
  direction = digitalRead(POL_LINE);
  if (direction) {
    ccsum++;  //charge
  } else {
    ccsum--; //discharge
  }
  numTicks++;

  unsigned long now = millis();
  totalTickTime += now - last_cc_tick;
  last_cc_tick = now;
}

//process incoming LoRa command
void processCommand(uint16_t numBytesRecieved)
{
  DEB("recieved command");
  switch (recieveBuffer[0]) {
    case CMD_SET_UPDATE_RATE:
      if (numBytesRecieved == 2 ) {
        uint8_t newRate = recieveBuffer[1];
        theUpdateRate = newRate * 60;
        if(theUpdateRate <60) {  //dont permit an update rate < 1 minute
          theUpdateRate = 60; 
        }
        DEB("CMD: new update rate");
        DEB(theUpdateRate);
      } else {
        DEB("incorret payload for command");
      }
      break;
    case CMD_NOP: //nop
       break;
    default:
      DEB("unkown command");
      break;
  }
}

void loop()
{
  int16_t transmitTemp;

#ifndef DUMMY_TEMP
  //power up switch GROVE rail
  pinMode(VCC_SW, OUTPUT);
  digitalWrite(VCC_SW, HIGH);
  delay(100);
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  sensors.begin();
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);

  oneWire.depower();  //switch off parasitic power (if used)
  digitalWrite(VCC_SW, LOW); //power down GROVE rail

  transmitTemp = (int16_t) 10 * temp;
#else
  transmitTemp = DUMMY_TEMP * 10;
#endif
  DEB(transmitTemp);

  // calculate power avg power consumption
  if ( numTicks > 0) {
    unsigned long avgTickTime = totalTickTime / numTicks;
    double avgTickSeconds = avgTickTime / 1000.0;
    DEB("avg tick time");
    DEB(avgTickSeconds);
    last_pwr_consumption = 614.4 /  avgTickSeconds;
    if (direction == false) {
      last_pwr_consumption =  last_pwr_consumption * -1;
    }
    totalTickTime = 0; //reset counters
    numTicks = 0;
  } else {
    last_pwr_consumption = 0; //no data available
  }


  uint8_t payload[8];
  // 2  temp
  // 4  Columb Counter
  // 2  consumption  (1000*mA)
  payload[0] = (uint8_t) (transmitTemp >> 8);
  payload[1] = (uint8_t)  0xff & transmitTemp;

  *(int32_t*) (&payload[2]) = ccsum;  //byte order??
  int16_t consumption = round(last_pwr_consumption * 1000);
  *(int16_t*) (&payload[6]) = consumption;

  DEB("coulomb ticks=");
  DEB(ccsum);


  unsigned long  startmillis = millis();
  uint8_t res = LoRaBee.send(1, payload, sizeof(payload));
  unsigned long send_duration = millis() - startmillis;

  switch (res)
  {
    case NoError:
      DEB("/");
      kanalStats.messagesTransmitted++;
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
      autonomoReset();
      break;
    case Busy:
      DEB("The device is busy. Sleeping for 10 extra seconds.");
      delay(60000);
      break;
    case NetworkFatalError:
      DEB("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      delay(60000);
      autonomoReset();
      break;
    case NotConnected:
      DEB("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      delay(60000);
      autonomoReset();
      break;
    case NoAcknowledgment:
      DEB("There was no acknowledgment sent back!");
      break;
    default:
      break;
  }

  delay(send_duration * 2); // wait 2*send duration for possible replies. TODO check with RN2483 lib whether this is not already included in the send time
  uint16_t numBytesRecieved = LoRaBee.receive(recieveBuffer,  recv_buffer_sz);
  if (numBytesRecieved > 0) {
    processCommand(numBytesRecieved);
  }

  lowPowerSleep(theUpdateRate);
}


void DFlashUltraDeepSleep()
{
  // SPI initialisation
  SPI.begin();

  // Initialise the CS pin for the data flash
  pinMode(SS_DFLASH, OUTPUT);
  digitalWrite(SS_DFLASH, HIGH);

  transmit(0x00); // In case already in sleep, wake
  transmit(0x79); // Now enter sleep

  // SPI end
  SPI.end();

  // Resets the pins used
  resetSPIPins();
}

void transmit(uint8_t val)
{
  SPISettings settings;
  SPI.beginTransaction(settings);
  digitalWrite(SS_DFLASH, LOW);

  SPI.transfer(val);

  digitalWrite(SS_DFLASH, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1000);
}

void resetSPIPins()
{
  resetPin(MISO);
  resetPin(MOSI);
  resetPin(SCK);
  resetPin(SS);
}

void resetPin(uint8_t pin)
{
  PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg = (uint8_t)(0);
  PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
  PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (uint32_t) (1 << g_APinDescription[pin].ulPin);
}

void autonomoReset() {
  __DSB();
  SCB->AIRCR  = 0x05FA0004;
  __DSB();
  while (1);
}

