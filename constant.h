/*
 * constant.h
 *
 *  Created on: 21 feb 2018
 *      Author: Emanuele
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_

#define HAVE_HWSERIAL1 1

#define LED 8
#define RN_RESET 9
#define BAT_ADC_EN 15
#define BAT_ADC 19
#define DS18B20_PIN_11 11  // on pin 11 (a 4.7K resistor is necessary)
#define DS18B20_PIN_3 3  // on pin 3 (a 4.7K resistor is necessary)
#define DS18B20_PIN_18 18  // on pin A4 (a 4.7K resistor is necessary)
#define PIN_RESET 12

/*! @enum DeviceState
 */
enum DeviceState
{
    DEVICE_STATE_INIT = 0,
    DEVICE_STATE_JOIN = 1,
    DEVICE_STATE_SEND = 2,
	DEVICE_STATE_SLEEP = 3
};

#endif /* CONSTANT_H_ */
