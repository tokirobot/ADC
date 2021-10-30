/*
 * ADC.c
 *
 *  Created on: Aug 6, 2021
 *      Author: hikot
 */

#include "ADC.h"


void getMotorCurrentValue(float *motor_current_value){
	float current_sensor_voltage[4]; //[V]

	//(RawData) -> (Voltage)
	current_sensor_voltage[0] = adc_raw_data[7] * 3.3 / 4096;
	current_sensor_voltage[1] = adc_raw_data[8] * 3.3 / 4096;
	current_sensor_voltage[2] = adc_raw_data[0] * 3.3 / 4096;
	current_sensor_voltage[3] = adc_raw_data[1] * 3.3 / 4096;


	//(Voltage) -> (Current) 10mV/A
	motor_current_value[0] = current_sensor_voltage[0] * 100;
	motor_current_value[1] = current_sensor_voltage[1] * 100;
	motor_current_value[2] = current_sensor_voltage[2] * 100;
	motor_current_value[3] = current_sensor_voltage[3] * 100;
}


float getBatteryVoltage(){
	float battery_voltage = adc_raw_data[0] * 3.3 / 4096; //[V]

	return battery_voltage;
}


void getPotentiometerRawValue(uint16_t *potentiometer_raw_data){
	potentiometer_raw_data[0] = adc_raw_data[2];
	potentiometer_raw_data[1] = adc_raw_data[3];
	potentiometer_raw_data[2] = adc_raw_data[4];
	potentiometer_raw_data[3] = adc_raw_data[5];
}


void convertADCValueIntoFlipperAngle(float *flipper_angle, int32_t *potentiometer_adc_raw_data){
	/*
	 * (adc_value) -> (flipper_potentiometer_radian)
	 *
	 *   4096      ->   2PI * 5 * 4
	 *
	 *    1        ->  20PI/4096
	 *
	 *
	 */

	flipper_angle[0] = potentiometer_adc_raw_data[0] * 0.01533980787f;  //PI*20/4096
	flipper_angle[1] = potentiometer_adc_raw_data[1] * 0.01533980787f;  //PI*20/4096
	flipper_angle[2] = potentiometer_adc_raw_data[2] * 0.01533980787f;  //PI*20/4096
	flipper_angle[3] = potentiometer_adc_raw_data[3] * 0.01533980787f;  //PI*20/4096
}


void calculateFlipperAngularVelocity(float *angular_velocity){
	uint16_t potentiometer_data[4];
	getPotentiometerRawValue(potentiometer_data);

	int16_t angular_velocity_adc_value[4];


	angular_velocity_adc_value[0] = potentiometer_data[0] - potentiometer_before_data[0];
	angular_velocity_adc_value[1] = potentiometer_data[1] - potentiometer_before_data[1];
	angular_velocity_adc_value[2] = potentiometer_data[2] - potentiometer_before_data[2];
	angular_velocity_adc_value[3] = potentiometer_data[3] - potentiometer_before_data[3];

	potentiometer_before_data[0] = potentiometer_data[0];
	potentiometer_before_data[1] = potentiometer_data[1];
	potentiometer_before_data[2] = potentiometer_data[2];
	potentiometer_before_data[3] = potentiometer_data[3];

	convertADCValueIntoFlipperAngle(angular_velocity, angular_velocity_adc_value);
}


