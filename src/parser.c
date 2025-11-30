/*
 * parser.c
 *
 *  Created on: 30.11.2025
 *      Author: stancecoke
 */

#include "main.h"
#include "CAN_Display.h"
void parse_para1(MotorParams_t* MP, MotorState_t* MS){
	MP->battery_current_max=Para1[1]*1000;
	MP->gear_ratio=Para1[19];
	MP->throttle_offset=(Para1[34]<<12)/50; //map 5V to 12 bit ADC resolution
	MP->throttle_max=(Para1[35]<<12)/50; //map 5V to 12 bit ADC resolution
}
