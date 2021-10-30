/*
 * ADC.h
 *
 *  Created on: Aug 6, 2021
 *      Author: hikot
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_


#include "stm32f4xx_hal.h"


/*
 * adc1 ch gotono wariate
 *
 *  ch |      role       |  array  |
 *
 *  2  | MotorCurrent 3  |    0    |
 *  3  | MotorCurrent 4  |    1    |
 *  4  | Potentiometer 1 |    2    |
 *  5  | Potentiometer 2 |    3    |
 *  6  | Potentiometer 3 |    4    |
 *  7  | Potentiometer 4 |    5    |
 *  8  | BatteryVoltage  |    6    |
 *  10 | MotorCurrent 1  |    7    |
 *  12 | MotorCurrent 2  |    8    |
 */
uint16_t adc_raw_data[9];

float flipper_angular_velocity[4];

static uint16_t potentiometer_before_data[4];


void getMotorCurrentValue(float *motor_current_value);

float getBatteryVoltage();

void getPotentiometerRawValue(uint16_t *potentiometer_raw_data);

void convertADCValueIntoFlipperAngle(float *flipper_angle, int32_t *potentiometer_adc_raw_data);

void calculateFlipperAngularVelocity(float *angular_velocity);  //timer interrupt  angular_velocity[rad/interrupt_time]


#endif /* INC_ADC_H_ */
