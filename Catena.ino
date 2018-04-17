
#include "constant.h"

#include "wiring_private.h"
#include <RTCZero.h>
#include <arduinoUART.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// LoRaWAN library
#include <arduinoLoRaWAN.h>
#include <configLoRaWAN.h>

RTCZero rtc;

/* Change these values to set the current initial time */
const uint8_t seconds = 0;
const uint8_t minutes = 00;
const uint8_t hours = 10;

/* Change these values to set the current initial date */
const uint8_t day = 20;
const uint8_t month = 2;
const uint8_t year = 17;

//////////////////////////////////////////////
uint8_t uart = 1;
//////////////////////////////////////////////

// Device parameters for Back-End registration
////////////////////////////////////////////////////////////
char DEVICE_EUI[]  = "0004A30B001BBB91";
char APP_EUI[] = "0102030405060708";
char APP_KEY[] = "27C6363C4B58C2FAA796DA60F762841B";
////////////////////////////////////////////////////////////

// Define port to use in Back-End: from 1 to 223
uint8_t port = 1;

// Define data payload to send (maximum is up to data rate)
char data[] = "010203";

// variable
uint8_t error = 0;
uint8_t _deviceState = DEVICE_STATE_INIT;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(DS18B20_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#define DS_NUM 11

typedef struct ds
  {
	DeviceAddress addr;
	int16_t t_16;
	bool check;
  };

ds _ds[DS_NUM];

uint8_t dsaddr[DS_NUM][8] = {
		{ 0x28, 0xFF, 0x35, 0xE7, 0xA1, 0x16, 0x05, 0xF5 },
		{ 0x28, 0xFF, 0x76, 0x94, 0xB0, 0x16, 0x04, 0xCD },
		{ 0x28, 0xFF, 0xB5, 0xA6, 0xB4, 0x16, 0x03, 0xF3 },
		{ 0x28, 0x56, 0xFB, 0xA2, 0x08, 0x00, 0x00, 0xDF },
		{ 0x28, 0xFF, 0xF5, 0x04, 0xA7, 0x16, 0x05, 0xC7 },
		{ 0x28, 0xFF, 0xC4, 0x9C, 0xA1, 0x16, 0x03, 0x7F },
		{ 0x28, 0xFF, 0x8C, 0xAB, 0xA7, 0x16, 0x03, 0xB7 },
		{ 0x28, 0x8C, 0x26, 0xA1, 0x08, 0x00, 0x00, 0xEE },
		{ 0x28, 0xFF, 0x17, 0xD0, 0xB4, 0x16, 0x03, 0xE2 },
		{ 0x28, 0xFF, 0x64, 0x8C, 0xA7, 0x16, 0x03, 0xA5 },
		{ 0x28, 0xFF, 0xB0, 0x0F, 0xB0, 0x16, 0x05, 0x58 }
};
uint32_t cont = 0;

void alarmMatch()
{
	//Serial2.println("Wakeup");
}

void printAddress(DeviceAddress deviceAddress);

// the setup function runs once when you press reset or power the board
void setup() {

	 // Switch unused pins as input and enabled built-in pullup
	 for (unsigned char pinNumber = 0; pinNumber < 23; pinNumber++)
	 {
		 pinMode(pinNumber, INPUT_PULLUP);
	 }

	 for (unsigned char pinNumber = 32; pinNumber < 42; pinNumber++)
	 {
		 pinMode(pinNumber, INPUT_PULLUP);
	 }

	 pinMode(25, INPUT_PULLUP);
	 pinMode(26, INPUT_PULLUP);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);
	pinMode(RN_RESET, INPUT);
	pinMode(RN_RESET, OUTPUT);
	digitalWrite(RN_RESET, HIGH);
	pinMode(BAT_ADC_EN, OUTPUT);
	digitalWrite(BAT_ADC_EN, LOW);
	pinMode(BAT_ADC, INPUT);
	analogReadResolution(12);

	//Serial.begin(9600);
	Serial.begin(9600);
//	pinPeripheral(6, PIO_SERCOM);
//	pinPeripheral(7, PIO_SERCOM);
//	Serial1.begin(57600);

	digitalWrite(LED, HIGH);
	delay(10000);
	digitalWrite(LED, LOW);
	Serial.println("START");

	sensors.begin();
	Serial.print("Found ");
//	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(" devices.");

	for(int i=0; i< DS_NUM; i++)
	{
		memcpy(&_ds[i].addr, dsaddr[i], 8);
		_ds[i].t_16 = 0;
		_ds[i].check = false;

		Serial.print("Check ");
		Serial.print(i+1);
		Serial.print(": ");
		printAddress(_ds[i].addr);
		Serial.print(" ");
		DeviceAddress taddr;
		memcpy(taddr, _ds[i].addr, 8);
		if (sensors.getAddress(taddr, i))
		{
			Serial.println("ON");
			bool err = sensors.setResolution(_ds[i].addr, 12);
			if(err)
				Serial.println("true");
			else
				Serial.println("false");
			_ds[i].check = true;
		}
		else
		{
			Serial.println("OFF");
			_ds[i].check = false;
		}

	}

	for(int i=0; i< DS_NUM; i++)
	{
		Serial.print("addr: ");
		printAddress(_ds[i].addr);
		Serial.println("");
	}

//	USBDevice.detach();

	// RTC initialization
	rtc.begin();
	rtc.setTime(hours, minutes, seconds);
	rtc.setDate(day, month, year);

//	 error = LoRaWAN.ON(uart);
//	 arduinoLoRaWAN::printAnswer(error);

}

// the loop function runs over and over again forever
void loop() {

	switch( _deviceState )
	{

		case DEVICE_STATE_INIT:
		{
		    //////////////////////////////////////////////
		    // 1. Switch on
		    //////////////////////////////////////////////
		    error = LoRaWAN.ON(uart);

		    //Serial2.print("1. Switch on: ");
		    arduinoLoRaWAN::printAnswer(error);



		    error = LoRaWAN.setDataRate(5);
		    arduinoLoRaWAN::printAnswer(error);
		    //Serial2.print("LoRaWAN._dataRate: ");
		    //Serial2.println(LoRaWAN._dataRate);

		    error = LoRaWAN.setRetries(0);
		    arduinoLoRaWAN::printAnswer(error);
		    //Serial2.print("LoRaWAN._dataRate: ");
		    //Serial2.println(LoRaWAN._dataRate);

		    error = LoRaWAN.setADR("on");
		    arduinoLoRaWAN::printAnswer(error);

			_deviceState = DEVICE_STATE_JOIN;
			break;
		}

		case DEVICE_STATE_JOIN:
		{
			//////////////////////////////////////////////
			// 6. Join network
			//////////////////////////////////////////////
			error = LoRaWAN.joinOTAA();
			arduinoLoRaWAN::printAnswer(error);

			// Check status
			 if( error == 0 )
			 {
			   //2. Join network OK
//			   Serial.println("Join network OK");

			   _deviceState = DEVICE_STATE_SEND;
			 }
			 else
			 {
			   //2. Join network error
//				 Serial.println("Join network error");
				 delay(10000);
				 _deviceState = DEVICE_STATE_JOIN;
			 }

			break;
		}

		case DEVICE_STATE_SEND:
		{
			sensors.requestTemperatures(); // Send the command to get temperatures

			uint8_t datab[4];
			char datas[9];
			data[0]='\0';

			int sensorValue = 0;
			digitalWrite(BAT_ADC_EN, HIGH);
			delay(500);
			sensorValue = analogRead(BAT_ADC);
			digitalWrite(BAT_ADC_EN, LOW);
//			Serial.print("ADC: ");
//			Serial.print(sensorValue);
//			Serial.print(" V: ");
			float v = ((float)sensorValue)*(0.0013427734375);
//			Serial.print(v);
//			Serial.print(" A: ");
			float a = ( ( (v-3) / 1.2) * 254 ) + 1;
			uint8_t level = 0;
			if(a<=0)
				level = 1;
			else if(a>=254)
				level = 254;
			else
				level = (uint8_t) a;
//			Serial.println(level);

			error = LoRaWAN.setBatteryLevel(level);
			arduinoLoRaWAN::printAnswer(error);

			float a_10 = ( ( (v) / 10) * 255 );
			uint8_t level_10 = 0;
			if(a_10<=0)
				level_10 = 0;
			else if(a_10>=255)
				level_10 = 255;
			else
				level_10 = (uint8_t) a_10;

			memcpy(&datab[0], &level_10, 1);
			sprintf(datas,"%02X", datab[0] & 0xff);
			strcat(data, datas);

			for(int i=0; i< DS_NUM; i++)
			{
				if(_ds[i].check == true)
				{
//					Serial.print("Addr: ");
//					printAddress(_ds[i].addr);
//					Serial.print(" TempC ");
//					Serial.print(i+1);
//					Serial.print(": ");
					_ds[i].t_16 = sensors.getTemp(_ds[i].addr);
//					Serial.println(DallasTemperature::rawToCelsius(_ds[i].t_16));
					memcpy(&datab[0], &_ds[i].t_16, 2);
					sprintf(datas,"%02X%02X", datab[0] & 0xff, datab[1] & 0xff);
					strcat(data, datas);
				}
			}
			Serial.println("");

			//////////////////////////////////////////////
			// 3. Send unconfirmed packet
			//////////////////////////////////////////////

			error = LoRaWAN.sendConfirmed(port, data);
			arduinoLoRaWAN::printAnswer(error);

			// Error messages:
			/*
			* '6' : Module hasn't joined a network
			* '5' : Sending error
			* '4' : Error with data length
			* '2' : Module didn't response
			* '1' : Module communication error
			*/
			// Check status
			if( error == 0 )
			{
			 //3. Send Confirmed packet OK
			 Serial.println("Send Confirmed packet OK");
			 if (LoRaWAN._dataReceived == true)
			 {
			   //There's data on
			   //port number: LoRaWAN._port
			   //and Data in: LoRaWAN._data
			   Serial.println("Downlink data");
			   Serial.print("LoRaWAN._port: ");
			   Serial.println(LoRaWAN._port);
			   Serial.print("LoRaWAN._data: ");
			   Serial.println(LoRaWAN._data);
			 }
			}
			else if( error == 6 )
			{
			 //3. Send Confirmed packet error
			   Serial.println("Module hasn't joined a network");

			   _deviceState = DEVICE_STATE_JOIN;
			}
			else
			{
			 //3. Send Confirmed packet error
			   Serial.println("Send Confirmed packet ERROR");
			}
			digitalWrite(LED, HIGH);
			delay(300);
			digitalWrite(LED, LOW);

			error = LoRaWAN.sleep(2000000);
			arduinoLoRaWAN::printAnswer(error);

//			Serial.print("Start sleep: ");
//			Serial.println(++cont);
//			//Serial2.flush();
//			//Serial2.end();

			//rtc.setAlarmSeconds((rtc.getAlarmSeconds() + 30) % 60);
			rtc.setAlarmMinutes((rtc.getAlarmMinutes() + 2) % 60);
			rtc.enableAlarm(rtc.MATCH_MMSS);
			rtc.attachInterrupt(alarmMatch);

			digitalWrite(LED, LOW);
//			rtc.standbyMode();
			delay(120000);

//			Serial.println("Exit sleep");

			error = LoRaWAN.wakeUP();
			arduinoLoRaWAN::printAnswer(error);

			error = LoRaWAN.check();
			arduinoLoRaWAN::printAnswer(error);
		}
	}


//	sensors.requestTemperatures(); // Send the command to get temperatures
//	for(int i=0; i< DS_NUM; i++)
//	{
//		if(_ds[i].check == true)
//		{
//
////			Serial.print("Resolution: ");
////			Serial.print(sensors.getResolution(_ds[i].addr), DEC);
////			Serial.println();
//
//			Serial.print("Addr: ");
//			printAddress(_ds[i].addr);
//			Serial.print(" TempC ");
//			Serial.print(i+1);
//			Serial.print(": ");
//			//float tempC = sensors.getTempC(_ds[i].addr);
//			_ds[i].t_16 = sensors.getTemp(_ds[i].addr);
//			Serial.println(DallasTemperature::rawToCelsius(_ds[i].t_16));
//
//		}
//	}
//	Serial.println("");


//	int sensorValue = 0;
//	digitalWrite(BAT_ADC_EN, HIGH);
//	delay(500);
//	sensorValue = analogRead(BAT_ADC);
//	digitalWrite(BAT_ADC_EN, LOW);
//	Serial.print("ADC: ");
//	Serial.print(sensorValue);
//	Serial.print(" V: ");
//	float v = ((float)sensorValue)*(0.0013427734375);
//	Serial.print(v);
//	Serial.print(" A: ");
//	float a = ( ( (v-3) / 1.2) * 254 ) + 1;
//	uint8_t level = 0;
//	if(a<=0)
//		level = 1;
//	else if(a>=254)
//		level = 254;
//	else
//		level = (uint8_t) a;
//	Serial.println(level);
//
//	error = LoRaWAN.setBatteryLevel(level);
//	arduinoLoRaWAN::printAnswer(error);
//
//	error = LoRaWAN.check();
//	arduinoLoRaWAN::printAnswer(error);
//
//  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);              // wait for a second
//  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
//  delay(5000);              // wait for a second
}


// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

