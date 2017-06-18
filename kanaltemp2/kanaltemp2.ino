

/**
   wiekaltistderkanal.de sensor  V2

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Note: This uses a modfied version of sodaq's RN2483 lib */
 #include <Arduino.h>
#include <RTCZero.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BME280.h"
#include "Sodaq_RN2483.h"
#include "Sodaq_wdt.h"

#undef DEBUG 
#define USE_WDT 1
 
#ifdef USE_WDT
#define DELAY(x) sodaq_wdt_safe_delay(x)
#else
#define DELAY(x) delay(x)
#endif

//constants for reading battery voltage
#define BATVOLTPIN  BAT_VOLT
static const double ADC_AREF = 3.3;
static const double BATVOLT_R1 = 4.7;
static const double BATVOLT_R2 = 10.0;

//IO port definitions
#define debugSerial SerialUSB
#define loraSerial  Serial1
 

//temp sensors
#define ONE_WIRE_BUS   3
#define ONE_WIRE_BUS_B 2
#define ONE_WIRE_BUS_CASE  7


//used for scaling float into uint16
//tick = 0.000534
static const double water_min_temp = -5.0;
static const double water_max_temp = 30;

//tick = 0.000092
static const double battery_min_volt = 0.0;
static const double battery_max_volt = 6.0;

//tick = 0.002747
static const double case_min_temp = -30;
static const double case_max_temp = 150; //on fire!


RTCZero rtc;

static int theUpdateRate = 20 * 60;
const static uint16_t recv_buffer_sz = 32;

#ifdef DEBUG
#define DEB(x) debugSerial.println(x);
#else
#define DEB(x)
#endif

enum Commands {
  CMD_NOP = 0x00,            // payload:none; no-operation
  CMD_SET_UPDATE_RATE = 0x01, //payload: uint8 - new update rate in minutes
  CMD_GET_STATS = 0x02,      //payload: none; replies with some statistics (not implemented)
  CMD_RESET = 0xff           //payload: none; board performs reset (not implemented)
};


//aggregate sum of coulomb counter
static int32_t ccsum = 0;

//interrupt flags
static volatile bool intAlarm = false; // RTC alarm interrupt

//statistics, mostly unused
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
const uint8_t devEUI[] = { 0x0B, 0x30, 0xB3, 0x0E, 0x30, 0x12, 0x04, 0x50 };
const uint8_t appEUI[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x0B, 0xCF };
const uint8_t appKey[] = { 0x45, 0x0C, 0xE2, 0x6B, 0x24, 0x43, 0xE1, 0x83, 0xC6, 0x57, 0xF8, 0x34, 0xE7, 0xF1, 0x02, 0x76 };

static bool connected=false;

/* TTN app name: kanaltemp_v2 */
Adafruit_BME280 bme; // I2C


void setup()
{
  DELAY(3142 * 2);

#ifdef DEBUG
  debugSerial.begin(57600);
  LoRaBee.setDiag(debugSerial);
#else
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  USBDevice.detach();
#endif

  DEB("Hi there.")

  // Turn the LoRaBee on
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
   DEB("LoRa Bee ON");
  DFlashUltraDeepSleep();  //send SPI flash to ultra deep sleep
 

 
  
  DELAY(1500);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  DELAY(100);
  
  connected = connectTTN();

 
  //TTN channel plan. 
  LoRaBee.configChFreq(0, 868100000L,0,5,1);
  LoRaBee.configChFreq(1, 868300000L,0,5,1); 
  LoRaBee.configChFreq(2, 868500000L,0,5,1);
  LoRaBee.configChFreq(3, 867100000L,0,5,1);
  LoRaBee.configChFreq(4, 867300000L,0,5,1);
  LoRaBee.configChFreq(5, 867500000L,0,5,1);
  LoRaBee.configChFreq(6, 867700000L,0,5,1);
  LoRaBee.configChFreq(7, 867900000L,0,5,1);

  if (connected == true) {
    DEB("Connection to the network was successful.");
  } else {
    DEB("Connection to the network failed!");
  }
  rtc.begin();
  last_cc_tick = millis();

   SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Enable deepsleep */
 
  bme.begin();
 //  XXXXX
#ifdef USE_WDT  
  sodaq_wdt_enable(WDT_PERIOD_8X);
#endif
 
}

boolean connectTTN(){
  return LoRaBee.initOTA(loraSerial, devEUI,appEUI,appKey);
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

  uint32_t beesleep = 1000L * sec - 500L;
  LoRaBee.sleep(beesleep); //have the LoRa Module wake up 500msec  early  
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Enable deepsleep */

  rtc.setTime(0, 0, 0);
  int w_min = sec / 60;
  int w_sec = sec % 60;

  rtc.setAlarmTime(0, w_min, w_sec);
  rtc.attachInterrupt(&alarmMatch);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  intAlarm = false;

  DEB("sleep until event");
  do {
     __WFI();
     sodaq_wdt_reset();
     DELAY(10);
  } while (!intAlarm);  // sleep again until source is rtc alarm
  rtc.disableAlarm();
  DEB("return from sleep");
}

 
//process incoming LoRa command
void processCommand(uint16_t numBytesRecieved)
{
  DEB("recieved command");
  if (numBytesRecieved < 1) { // abort if no payload
    return;
  }

  switch (recieveBuffer[0]) {
    case CMD_SET_UPDATE_RATE:
      if (numBytesRecieved == 2 ) {
        uint8_t newRate = recieveBuffer[1];
        theUpdateRate = newRate * 60;
        if (theUpdateRate < 60) { //dont permit an update rate < 1 minute
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

float readTemp(int oneWireBusPin) {
  OneWire oneWire(oneWireBusPin);
  DallasTemperature sensors(&oneWire);
  sensors.begin();
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  oneWire.depower();
  return temp;
}

void loop()
{
  uint16_t transmitTemp1, transmitTemp2, caseTemp;
  //power up switch GROVE rail
  pinMode(VCC_SW, OUTPUT);
  digitalWrite(VCC_SW, HIGH);
  DELAY(100);
  sodaq_wdt_reset();
  
  // default (and max) resolution of sensor is 12 bit / 0.0625°C
  // beware: absolute accuracy is ±0.5°C  from -10°C to +85°C for the uncalibrated DS18b20
  float temp1 = readTemp(ONE_WIRE_BUS);
  transmitTemp1 = fitUInt16(temp1, water_min_temp, water_max_temp); //  lower sensor
  float temp2 = readTemp(ONE_WIRE_BUS_B);
  transmitTemp2 = fitUInt16(temp2, water_min_temp, water_max_temp); // upper sensor
  caseTemp = fitUInt16(readTemp(ONE_WIRE_BUS_CASE), case_min_temp, case_max_temp); // case (battery) temp

  DEB("battery");
  float vbatt = getRealBatteryVoltageMV() / 1000.0;
  DEB(vbatt);
  uint16_t vbattui = fitUInt16(vbatt, battery_min_volt, battery_max_volt);
  DEB(vbattui);
  
  digitalWrite(VCC_SW, LOW); //power down GROVE rail

  DEB("temps")
  DEB(temp1);
  DEB(temp2);

  resetPin(VCC_SW); //conserve 1 uA?
  resetPin(ONE_WIRE_BUS);
  resetPin(ONE_WIRE_BUS_B);
  resetPin(ONE_WIRE_BUS_CASE);

  //now get TPH data
  uint16_t pressureHPa;
  uint16_t humidity;
  {
    pressureHPa = round(bme.readPressure() / 100.0F*10.0);
    DEB("pressure");
    DEB(pressureHPa);

    humidity = round(bme.readHumidity() * 10.0);
    DEB("humidity*10");
    DEB(humidity);
   
  }

  uint8_t payload[18];
  // 2  temp (lower sensor) (scaled -5 .. 30)
  // 2  temp (upper sensor) (scaled -5 .. 30)
  // 4  Columb Counter sum
  // 2  consumption  (1000*mA)
  // 2  batt voltage (scaled 0..5)
  // 2  case temp (scaled case_min_temp .. case_max_temp // -30 ...150)
  // 2  humidity *10
  // 2  pressure in HPa *10
  payload[0] = (uint8_t) (transmitTemp1 >> 8);
  payload[1] = (uint8_t)  0xff & transmitTemp1;

  payload[2] = (uint8_t) (transmitTemp2 >> 8);
  payload[3] = (uint8_t)  0xff & transmitTemp2;

  *(int32_t*) (&payload[4]) = ccsum;  //byte order??
  int16_t consumption = round(last_pwr_consumption * 1000);
  *(int16_t*) (&payload[8]) = consumption;

  payload[10] = (uint8_t) (vbattui >> 8);
  payload[11] = (uint8_t) (vbattui & 0xff);
  payload[12] = (uint8_t) (caseTemp >> 8);
  payload[13] = (uint8_t) (caseTemp & 0xff);

  payload[14] = (uint8_t) (humidity >> 8);
  payload[15] = (uint8_t) (humidity & 0xff);

  payload[16] = (uint8_t) (pressureHPa >> 8);
  payload[17] = (uint8_t) (pressureHPa & 0xff);


  

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
      DEB("Connection timed-out. Check your serial connection to the device! Sleeping for 2sec.");
      DELAY(20000);
      break;
    case PayloadSizeError:
      DEB("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      DEB("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe program will now halt.");
      DELAY(20000);
      //autonomoReset();
      break;
    case Busy:
      DEB("The device is busy. Sleeping for 1 extra seconds.");
      DELAY(20000);
      break;
    case NetworkFatalError:
      DEB("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      DELAY(20000);
      //autonomoReset();
      break;
    case NotConnected:
      DEB("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      DELAY(20000);
      connectTTN();
      //autonomoReset();
      break;
    case NoAcknowledgment:
      DEB("There was no acknowledgment sent back!");
      DELAY(2000);
      break;
    default:
      break;
  }

// TODO check for err condition 
  uint16_t numBytesRecieved = LoRaBee.receive(recieveBuffer,  recv_buffer_sz);
  if (numBytesRecieved > 0) {
    processCommand(numBytesRecieved);
  }

  lowPowerSleep(theUpdateRate);
}


double getRealBatteryVoltageMV()
{
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);

  return (ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage;
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

//scale a float between min..max into a UInt16
uint16_t fitUInt16(double value, double min, double max) {
  double tick =  UINT16_MAX / abs(min - max);
  double prec = 1 / tick;
  //  printf("prec %f \n",prec);
  double zeroBaseVal = (min < 0) ? value - min : value + min;
  //  printf("zbval %f \n",zeroBaseVal);
  return zeroBaseVal * tick;
}


double  expandUInt16(uint16_t val, double min, double max ) {
  double tick =  UINT16_MAX / abs(min - max);
  double offset = (min < 0) ? min : -min;
  return (val / tick) + offset;
}

