
#include "constant.h"

#include "wiring_private.h"
#include <Adafruit_SleepyDog.h>
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

#define DS_NUM_CATENE 3
#define A 0
#define B 1
#define C 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire_A(DS18B20_PIN_11);
OneWire oneWire_B(DS18B20_PIN_18);
OneWire oneWire_C(DS18B20_PIN_3);

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature sensors_A(&oneWire_A);
//DallasTemperature sensors_B(&oneWire_B);
//DallasTemperature sensors_C(&oneWire_C);
DallasTemperature sensors[DS_NUM_CATENE] = { DallasTemperature(&oneWire_A), DallasTemperature(&oneWire_B), DallasTemperature(&oneWire_C) };

#define DS_NUM 5
//#define DS_NUM 11
//#define DS_NUM 1

typedef struct ds
  {
	DeviceAddress addr;
	int16_t t_16;
	bool check;
  };

ds _ds[3][DS_NUM];

//Catena5 installata
//uint8_t dsaddr[DS_NUM][8] = {
//		{ 0x28, 0xFF, 0xDE, 0x9E, 0xA7, 0x16, 0x05, 0xB5 },
//		{ 0x28, 0xFF, 0x37, 0x91, 0xB0, 0x16, 0x04, 0x68 },
//		{ 0x28, 0xFF, 0xBD, 0x0C, 0xB0, 0x16, 0x05, 0x3C },
//		{ 0x28, 0xFF, 0xCC, 0x48, 0xA7, 0x16, 0x05, 0x42 },
//		{ 0x28, 0xFF, 0x8E, 0x2B, 0xB0, 0x16, 0x05, 0x00 }
//};

//Catena11
//uint8_t dsaddr[DS_NUM][8] = {
//		{ 0x28, 0xFF, 0x35, 0xE7, 0xA1, 0x16, 0x05, 0xF5 },
//		{ 0x28, 0xFF, 0x76, 0x94, 0xB0, 0x16, 0x04, 0xCD },
//		{ 0x28, 0xFF, 0xB5, 0xA6, 0xB4, 0x16, 0x03, 0xF3 },
//		{ 0x28, 0x56, 0xFB, 0xA2, 0x08, 0x00, 0x00, 0xDF },
//		{ 0x28, 0xFF, 0xF5, 0x04, 0xA7, 0x16, 0x05, 0xC7 },
//		{ 0x28, 0xFF, 0xC4, 0x9C, 0xA1, 0x16, 0x03, 0x7F },
//		{ 0x28, 0xFF, 0x8C, 0xAB, 0xA7, 0x16, 0x03, 0xB7 },
//		{ 0x28, 0x8C, 0x26, 0xA1, 0x08, 0x00, 0x00, 0xEE },
//		{ 0x28, 0xFF, 0x17, 0xD0, 0xB4, 0x16, 0x03, 0xE2 },
//		{ 0x28, 0xFF, 0x64, 0x8C, 0xA7, 0x16, 0x03, 0xA5 },
//		{ 0x28, 0xFF, 0xB0, 0x0F, 0xB0, 0x16, 0x05, 0x58 }
//};

uint8_t dsaddr[DS_NUM_CATENE][DS_NUM][8] = {
		//Catena A
		{
			{ 0x28, 0x61, 0x64, 0x11, 0x90, 0x71, 0xB3, 0xD9 },
			{ 0x628, 0x61, 0x64, 0x11, 0x8D, 0xB0, 0xD1, 0x2A },
			{ 0x628, 0x61, 0x64, 0x11, 0x83, 0xA3, 0x05, 0x51 },
			{ 0x628, 0x61, 0x64, 0x11, 0x90, 0x4A, 0xFC, 0xD0 },
			{ 0x628, 0x61, 0x64, 0x11, 0x83, 0xC4, 0xFB, 0x0E }
		},
		//Catena B
		{
			{ 0x28, 0x61, 0x64, 0x11, 0x90, 0x78, 0xC1, 0x2F },
			{ 0x28, 0x61, 0x64, 0x11, 0x83, 0xC7, 0xDF, 0x19 },
			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0x9E, 0x27, 0x9E },
			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0x9F, 0xA1, 0x0B },
			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0xF5, 0x64, 0x43 }
		},
		//Catena C
		{
			{ 0x28, 0xFF, 0x26, 0x92, 0xA1, 0x16, 0x03, 0x33 },
			{ 0x28, 0xFF, 0x3E, 0x8A, 0xA1, 0x16, 0x03, 0x55 },
			{ 0x28, 0xFF, 0x80, 0x58, 0xA7, 0x16, 0x03, 0x6F },
			{ 0x28, 0xFF, 0xA6, 0x80, 0xB0, 0x16, 0x04, 0xA5 },
			{ 0x28, 0xFF, 0x7A, 0x85, 0xA7, 0x16, 0x03, 0xE8 }
		}
};


//Catena D
//uint8_t dsaddr_c[DS_NUM][8] = {
//		{ 0x28, 0xFF, 0x04, 0xD3, 0xA6, 0x16, 0x04, 0xDE },
//		{ 0x28, 0xFF, 0xAF, 0xCC, 0xA7, 0x16, 0x03, 0x17 },
//		{ 0x28, 0x55, 0x99, 0xA1, 0x08, 0x00, 0x00, 0x9C },
//		{ 0x28, 0xFF, 0x0E, 0x57, 0xB0, 0x16, 0x04, 0x2F },
//		{ 0x28, 0xFF, 0xBB, 0xDE, 0xA1, 0x16, 0x03, 0x9A }
//};


//uint8_t dsaddr[DS_NUM][8] = {
//		{ 0x28, 0xFF, 0xF6, 0xF1, 0xB4, 0x16, 0x03 , 0x0A }
//};

uint32_t cont = 0;

void alarmMatch()
{
	//Serial2.println("Wakeup");
}

void printAddress(DeviceAddress deviceAddress);

// the setup function runs once when you press reset or power the board
void setup() {

	 // Switch unused pins as input and enabled built-in pullup
//	 for (unsigned char pinNumber = 0; pinNumber < 23; pinNumber++)
//	 {
//		 pinMode(pinNumber, INPUT_PULLUP);
//	 }
//
//	 for (unsigned char pinNumber = 32; pinNumber < 42; pinNumber++)
//	 {
//		 pinMode(pinNumber, INPUT_PULLUP);
//	 }
//
//	 pinMode(25, INPUT_PULLUP);
//	 pinMode(26, INPUT_PULLUP);

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

//	sensors[A] = sensors_A;
//	sensors[B] = sensors_B;
//	sensors[C] = sensors_C;

	//sensors[A].begin();
	sensors[B].begin();
	sensors[C].begin();
	Serial.print("Found ");
//	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(" devices.");

	for(int k=0; k< DS_NUM_CATENE; k++)
	{
		Serial.print("--> ");
		Serial.println((char)('A' + k));

		for(int i=0; i< DS_NUM; i++)
		{
			memcpy(&_ds[k][i].addr, dsaddr[k][i], 8);
			_ds[k][i].t_16 = 0;
			_ds[k][i].check = false;

			Serial.print("Check ");
			Serial.print(i+1);
			Serial.print(": ");
			printAddress(_ds[k][i].addr);
			Serial.print(" ");
			DeviceAddress taddr;
			memcpy(taddr, _ds[k][i].addr, 8);
			if (sensors[k].getAddress(taddr, i))
			{
				Serial.println("ON");
				bool err = sensors[k].setResolution(_ds[k][i].addr, 12);
				if(err)
					Serial.println("true");
				else
					Serial.println("false");
				_ds[k][i].check = true;
			}
			else
			{
				Serial.println("OFF");
				_ds[k][i].check = false;
			}

		}

		for(int i=0; i< DS_NUM; i++)
		{
			Serial.print("addr: ");
			printAddress(_ds[k][i].addr);
			Serial.println("");
		}

		Serial.println("");

	}


	Serial.println("Fine setup");

	Serial.end();
	USBDevice.detach();

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
			Watchdog.disable();
		    //////////////////////////////////////////////
		    // 1. Switch on
		    //////////////////////////////////////////////
		    error = LoRaWAN.ON(uart);

		    Serial.print("1. Switch on: ");
		    arduinoLoRaWAN::printAnswer(error);



		    error = LoRaWAN.setDataRate(0);
		    arduinoLoRaWAN::printAnswer(error);
//		    Serial.print("LoRaWAN._dataRate: ");
//		    Serial.println(LoRaWAN._dataRate);

		    error = LoRaWAN.setRetries(0);
		    arduinoLoRaWAN::printAnswer(error);
//		    Serial.print("LoRaWAN._retries: ");
//		    Serial.println(LoRaWAN._retries);

		    error = LoRaWAN.setADR("on");
		    arduinoLoRaWAN::printAnswer(error);
//		    Serial.print("LoRaWAN._adr: ");
//		    Serial.println(LoRaWAN._adr);

			_deviceState = DEVICE_STATE_JOIN;
			break;
		}

		case DEVICE_STATE_JOIN:
		{
			Watchdog.disable();
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
			sensors[A].requestTemperatures(); // Send the command to get temperatures
			sensors[B].requestTemperatures();
			sensors[C].requestTemperatures();
			delay(500);

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

			for(int k=0; k< DS_NUM_CATENE; k++)
			{
				for(int i=0; i< DS_NUM; i++)
				{
					if(_ds[k][i].check == true)
					{
//						Serial.print("Addr: ");
//						printAddress(_ds[k][i].addr);
//						Serial.print(" Temp");
//						Serial.print((char)('A' + k));
//						Serial.print(" ");
//						Serial.print(i+1);
//						Serial.print(": ");
						_ds[k][i].t_16 = sensors[k].getTemp(_ds[k][i].addr);
//						Serial.println(DallasTemperature::rawToCelsius(_ds[k][i].t_16));
						memcpy(&datab[0], &_ds[k][i].t_16, 2);
						sprintf(datas,"%02X%02X", datab[0] & 0xff, datab[1] & 0xff);
						strcat(data, datas);
					}
				}
			}
//			Serial.println("");

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
//			 Serial.println("Send Confirmed packet OK");
			 if (LoRaWAN._dataReceived == true)
			 {
			   //There's data on
			   //port number: LoRaWAN._port
			   //and Data in: LoRaWAN._data
//			   Serial.println("Downlink data");
//			   Serial.print("LoRaWAN._port: ");
//			   Serial.println(LoRaWAN._port);
//			   Serial.print("LoRaWAN._data: ");
//			   Serial.println(LoRaWAN._data);
			 }
			}
			else if( error == 6 )
			{
			   _deviceState = DEVICE_STATE_JOIN;
			}
			else if( error == 2 )
			{
			   _deviceState = DEVICE_STATE_INIT;
			}
			else
			{
			 //3. Send Confirmed packet error
//			   Serial.println("Send Confirmed packet ERROR");
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
			rtc.setAlarmMinutes((rtc.getAlarmMinutes() + 3) % 60);
			rtc.enableAlarm(rtc.MATCH_MMSS);
			rtc.attachInterrupt(alarmMatch);

			digitalWrite(LED, LOW);
			// Disable the watchdog entirely by calling Watchdog.disable();
			Watchdog.disable();
			rtc.standbyMode();
//			delay(30000);
			Watchdog.enable(16000);

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

