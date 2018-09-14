
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
#define SLEEP_TIME 10
#define DUTY_CYCLE_TIME 180

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

#define DS_NUM_CATENE 2
//#define A 0
//#define B 1
//#define C 2
#define D 0
#define E 1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire_A(DS18B20_PIN_11);
//OneWire oneWire_B(DS18B20_PIN_18);
//OneWire oneWire_C(DS18B20_PIN_3);
OneWire oneWire_D(DS18B20_PIN_11);
OneWire oneWire_E(DS18B20_PIN_18);

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature sensors_A(&oneWire_A);
//DallasTemperature sensors_B(&oneWire_B);
//DallasTemperature sensors_C(&oneWire_C);
//DallasTemperature sensors[DS_NUM_CATENE] = { DallasTemperature(&oneWire_A), DallasTemperature(&oneWire_B), DallasTemperature(&oneWire_C) };
DallasTemperature sensors[DS_NUM_CATENE] = { DallasTemperature(&oneWire_D), DallasTemperature(&oneWire_E) };

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

//uint8_t dsaddr[DS_NUM_CATENE][DS_NUM][8] = {
//		//Catena A
//		{
//			{ 0x28, 0x61, 0x64, 0x11, 0x90, 0x71, 0xB3, 0xD9 },
//			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0xB0, 0xD1, 0x2A },
//			{ 0x28, 0x61, 0x64, 0x11, 0x83, 0xA3, 0x05, 0x51 },
//			{ 0x28, 0x61, 0x64, 0x11, 0x90, 0x4A, 0xFC, 0xD0 },
//			{ 0x28, 0x61, 0x64, 0x11, 0x83, 0xC4, 0xFB, 0x0E }
//		},
//		//Catena B
//		{
//			{ 0x28, 0x61, 0x64, 0x11, 0x90, 0x78, 0xC1, 0x2F },
//			{ 0x28, 0x61, 0x64, 0x11, 0x83, 0xC7, 0xDF, 0x19 },
//			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0x9E, 0x27, 0x9E },
//			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0x9F, 0xA1, 0x0B },
//			{ 0x28, 0x61, 0x64, 0x11, 0x8D, 0xF5, 0x64, 0x43 }
//		},
//		//Catena C
//		{
//			{ 0x28, 0xFF, 0x26, 0x92, 0xA1, 0x16, 0x03, 0x33 },
//			{ 0x28, 0xFF, 0x3E, 0x8A, 0xA1, 0x16, 0x03, 0x55 },
//			{ 0x28, 0xFF, 0x80, 0x58, 0xA7, 0x16, 0x03, 0x6F },
//			{ 0x28, 0xFF, 0xA6, 0x80, 0xB0, 0x16, 0x04, 0xA5 },
//			{ 0x28, 0xFF, 0x7A, 0x85, 0xA7, 0x16, 0x03, 0xE8 }
//		}
//};

uint8_t dsaddr[DS_NUM_CATENE][DS_NUM][8] = {
		//Catena D
		{
			{ 0x28, 0xFF, 0x04, 0xD3, 0xA6, 0x16, 0x04, 0xDE },
			{ 0x28, 0xFF, 0xAF, 0xCC, 0xA7, 0x16, 0x03, 0x17 },
			{ 0x28, 0x55, 0x99, 0xA1, 0x08, 0x00, 0x00, 0x9C },
			{ 0x28, 0xFF, 0x0E, 0x57, 0xB0, 0x16, 0x04, 0x2F },
			{ 0x28, 0xFF, 0xBB, 0xDE, 0xA1, 0x16, 0x03, 0x9A }
		},
		//Catena E (ex 5)
		{
			{ 0x28, 0xFF, 0x03, 0x74, 0xA7, 0x16, 0x05, 0x55 },
			{ 0x28, 0xFF, 0x37, 0x91, 0xB0, 0x16, 0x04, 0x68 },
			{ 0x28, 0xFF, 0xBD, 0x0C, 0xB0, 0x16, 0x05, 0x3C },
			{ 0x28, 0xFF, 0xCC, 0x48, 0xA7, 0x16, 0x05, 0x42 },
			{ 0x28, 0xFF, 0x8E, 0x2B, 0xB0, 0x16, 0x05, 0x00 }
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

uint32_t sleep_cont = 0;

uint32_t appWatchdog = 0;

int err_tx = 0;

void alarmMatch()
{
	//Serial2.println("Wakeup");
}

void printAddress(DeviceAddress deviceAddress);

// the setup function runs once when you press reset or power the board
void setup() {

	 // Switch unused pins as input and enabled built-in pullup
	 for (unsigned char pinNumber = 0; pinNumber < 11; pinNumber++)
	 {
		 pinMode(pinNumber, INPUT_PULLUP);
	 }

	 for (unsigned char pinNumber = 32; pinNumber < 34; pinNumber++)
	 {
		 pinMode(pinNumber, INPUT_PULLUP);
	 }

	 // NO 34 = PA19 = D12 reset pin

	 for (unsigned char pinNumber = 35; pinNumber < 42; pinNumber++)
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

	// pin reset host
	digitalWrite(PIN_RESET, LOW);
	pinMode(PIN_RESET, OUTPUT);

	//Serial.begin(9600);
	Serial.begin(9600);
	Serial5.begin(9600);
//	pinPeripheral(6, PIO_SERCOM);
//	pinPeripheral(7, PIO_SERCOM);
//	Serial1.begin(57600);

	digitalWrite(LED, HIGH);
	delay(10000);
	digitalWrite(LED, LOW);
	Serial5.println("START");

//	sensors[A].begin();
//	sensors[B].begin();
//	sensors[C].begin();
	sensors[D].begin();
	sensors[E].begin();
//	Serial5.print("Found A ");
//	Serial5.print(sensors[A].getDeviceCount(), DEC);
//	Serial5.println(" devices.");
//	Serial5.print("Found B ");
//	Serial5.print(sensors[B].getDeviceCount(), DEC);
//	Serial5.println(" devices.");
//	Serial5.print("Found C ");
//	Serial5.print(sensors[C].getDeviceCount(), DEC);
//	Serial5.println(" devices.");
	Serial5.print("Found D ");
	Serial5.print(sensors[D].getDeviceCount(), DEC);
	Serial5.println(" devices.");
	Serial5.print("Found E ");
	Serial5.print(sensors[E].getDeviceCount(), DEC);
	Serial5.println(" devices.");

	for(int k=0; k< DS_NUM_CATENE; k++)
	{
		Serial5.print("--> ");
//		Serial5.println((char)('A' + k));
		Serial5.println((char)('D' + k));

		for(int i=0; i< DS_NUM; i++)
		{
			memcpy(&_ds[k][i].addr, dsaddr[k][i], 8);
			_ds[k][i].t_16 = 0;
			_ds[k][i].check = false;

			Serial5.print("Check ");
			Serial5.print(i+1);
			Serial5.print(": ");
			printAddress(_ds[k][i].addr);
			Serial5.print(" ");
			DeviceAddress taddr;
			memcpy(taddr, _ds[k][i].addr, 8);
			if (sensors[k].getAddress(taddr, i))
			{
				Serial5.println("ON");
				bool err = sensors[k].setResolution(_ds[k][i].addr, 12);
				if(err)
					Serial5.println("true");
				else
					Serial5.println("false");
				_ds[k][i].check = true;
			}
			else
			{
				Serial5.println("OFF");
				_ds[k][i].check = false;
			}

		}

		for(int i=0; i< DS_NUM; i++)
		{
			Serial5.print("addr: ");
			printAddress(_ds[k][i].addr);
			Serial5.println("");
		}

		Serial5.println("");

	}


	Serial5.println("Fine setup");

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
			Watchdog.enable(16000);
		    //////////////////////////////////////////////
		    // 1. Switch on
		    //////////////////////////////////////////////

			error = LoRaWAN.OFF(uart);
			digitalWrite(RN_RESET, LOW);
			delay(500);
			digitalWrite(RN_RESET, HIGH);
		    error = LoRaWAN.ON(uart);

		    Serial5.print("1. Switch on: ");
		    arduinoLoRaWAN::printAnswer(error);



		    error = LoRaWAN.setDataRate(0);
		    arduinoLoRaWAN::printAnswer(error);
		    Serial5.print("LoRaWAN._dataRate: ");
		    Serial5.println(LoRaWAN._dataRate);

		    error = LoRaWAN.setRetries(0);
		    arduinoLoRaWAN::printAnswer(error);
		    Serial5.print("LoRaWAN._retries: ");
		    Serial5.println(LoRaWAN._retries);

		    error = LoRaWAN.setADR("on");
		    arduinoLoRaWAN::printAnswer(error);
		    Serial5.print("LoRaWAN._adr: ");
		    Serial5.println(LoRaWAN._adr);

			_deviceState = DEVICE_STATE_JOIN;
			break;
		}

		case DEVICE_STATE_JOIN:
		{
			Watchdog.disable();
			Watchdog.enable(16000);
			//////////////////////////////////////////////
			// 6. Join network
			//////////////////////////////////////////////
			error = LoRaWAN.joinOTAA();
			arduinoLoRaWAN::printAnswer(error);

			// Check status
			 if( error == 0 )
			 {
			   //2. Join network OK
			   Serial5.println("Join network OK");

			   _deviceState = DEVICE_STATE_SEND;
			 }
			 else
			 {
			   //2. Join network error
				 Serial5.println("Join network error");
				 if(appWatchdog++ >= 10)
				 {
					 Serial5.println("Auto reset appWatchdog");
					 appWatchdog = 0;
					 Watchdog.reset();
					 Watchdog.enable(1000);
					 delay(2000);
				 }
				 Watchdog.reset();
				 delay(10000);
				 _deviceState = DEVICE_STATE_JOIN;
			 }

			break;
		}

		case DEVICE_STATE_SEND:
		{
//			sensors[A].requestTemperatures(); // Send the command to get temperatures
//			sensors[B].requestTemperatures();
//			sensors[C].requestTemperatures();
			sensors[D].requestTemperatures(); // Send the command to get temperatures
			sensors[E].requestTemperatures();
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
						Serial5.print("Addr: ");
						printAddress(_ds[k][i].addr);
						Serial5.print(" Temp");
//						Serial5.print((char)('A' + k));
						Serial5.print((char)('D' + k));
						Serial5.print(" ");
						Serial5.print(i+1);
						Serial5.print(": ");
						_ds[k][i].t_16 = sensors[k].getTemp(_ds[k][i].addr);
						Serial5.println(DallasTemperature::rawToCelsius(_ds[k][i].t_16));
						memcpy(&datab[0], &_ds[k][i].t_16, 2);
						sprintf(datas,"%02X%02X", datab[0] & 0xff, datab[1] & 0xff);
						strcat(data, datas);
					}
					else
					{
						datab[0] = 0x00;
						datab[1] = 0x00;
						sprintf(datas,"%02X%02X", datab[0] & 0xff, datab[1] & 0xff);
						strcat(data, datas);
					}
				}
			}
			Serial5.println("");

			//////////////////////////////////////////////
			// 3. Send unconfirmed packet
			//////////////////////////////////////////////

			int tx_cont = 0;
			do
			{
				Watchdog.reset();
				if(tx_cont>0)
					delay(5000);
				Serial5.print("Send unconfirmed packet: ");
				Serial5.println(tx_cont);
				error = LoRaWAN.sendConfirmed(port, data);
				arduinoLoRaWAN::printAnswer(error);
			}while(error!=0 && tx_cont++<3);

			_deviceState = DEVICE_STATE_SLEEP;

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
				err_tx=0;
			 //3. Send Confirmed packet OK
			 Serial5.println("Send Confirmed packet OK");
			 if (LoRaWAN._dataReceived == true)
			 {
			   //There's data on
			   //port number: LoRaWAN._port
			   //and Data in: LoRaWAN._data
			   Serial5.println("Downlink data");
			   Serial5.print("LoRaWAN._port: ");
			   Serial5.println(LoRaWAN._port);
			   Serial5.print("LoRaWAN._data: ");
			   Serial5.println(LoRaWAN._data);

			   int number = (int)strtol(LoRaWAN._data, NULL, 16);

			   Serial5.println(number);

			   // r host reset
			   if(number == 114)
			   {
				    Serial5.println("Host reset");
					digitalWrite(PIN_RESET, HIGH);
					delay(2000);
					digitalWrite(PIN_RESET, LOW);
			   }
			   // auto-reset
			   if(number == 97)
			   {
				   Serial5.println("Auto-reset");
				   Watchdog.enable(1000);
				   delay(2000);
			   }

			 }

			}
			else if( error == 6 )
			{
			   _deviceState = DEVICE_STATE_JOIN;
			}
//			else if( error == 2 )
//			{
//			   _deviceState = DEVICE_STATE_INIT;
//			}
			else
			{
			   Serial5.println("Send Confirmed packet ERROR");
				if(err_tx++ >= 5)
				{
					Serial5.println("Auto-reset");
					   Watchdog.enable(1000);
					   delay(2000);
				}
			}

			break;
		}
		case DEVICE_STATE_SLEEP:
			{
				//////////////////////////////////////////////
				// 7. Sleep
				//////////////////////////////////////////////

				Serial5.println("SLEEP");
				Serial5.print("sleep_cont: ");
				Serial5.println(sleep_cont);

				if(sleep_cont == 0)
				{
					sleep_cont++;

					Serial5.println("LoRaWAN.sleep");
					error = LoRaWAN.sleep(2000000);
					arduinoLoRaWAN::printAnswer(error);

				}else if( ( sleep_cont > 0 ) && ( sleep_cont < ( DUTY_CYCLE_TIME / SLEEP_TIME ) ) )
				{
					sleep_cont++;

					char buf[20];
					sprintf(buf, "%02d:%02d:%02d - %2d/%2d/%4d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getDay(), rtc.getMonth(), rtc.getYear());
					Serial5.println(buf);

					rtc.disableAlarm();
					rtc.setAlarmSeconds((rtc.getSeconds() + SLEEP_TIME) % 60);
					rtc.enableAlarm(rtc.MATCH_SS);
					rtc.attachInterrupt(alarmMatch);

					sprintf(buf, "%02d:%02d:%02d - %2d/%2d/%4d", rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds(), rtc.getAlarmDay(), rtc.getAlarmMonth(), rtc.getAlarmYear());
					Serial5.println(buf);

					Watchdog.disable();
					Watchdog.enable(16000);
					Serial5.println("standbyMode");
					delay(100);
					Serial5.end();

					rtc.standbyMode();

					Serial5.begin(9600);
					Serial5.println("exit sleep");
					sprintf(buf, "%02d:%02d:%02d - %2d/%2d/%4d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getDay(), rtc.getMonth(), rtc.getYear());
					Serial5.println(buf);

					Watchdog.disable();
					Watchdog.enable(16000);

					_deviceState = DEVICE_STATE_SLEEP;

				}else if( sleep_cont >= ( DUTY_CYCLE_TIME / SLEEP_TIME ) )
				{
					Serial5.println("WAKEUP radio");
					sleep_cont = 0;
					_deviceState = DEVICE_STATE_SEND;

					error = LoRaWAN.wakeUP();
					arduinoLoRaWAN::printAnswer(error);

					error = LoRaWAN.check();
					arduinoLoRaWAN::printAnswer(error);

					error = LoRaWAN.check();
					arduinoLoRaWAN::printAnswer(error);

					error = LoRaWAN.check();
					arduinoLoRaWAN::printAnswer(error);
					if(error > 0)
						_deviceState = DEVICE_STATE_INIT;
				}

				break;
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
    Serial5.print(deviceAddress[i], HEX);
  }
}

