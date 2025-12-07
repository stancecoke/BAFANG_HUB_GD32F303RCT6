/*
 * parser.c
 *
 *  Created on: 30.11.2025
 *      Author: stancecoke
 */

#include "main.h"
#include "CAN_Display.h"
void parse_params(MotorParams_t* MP){
	MP->battery_current_max=Para1[1]*1000;
	MP->gear_ratio=Para1[19];
	MP->throttle_offset=(Para1[34]<<12)/33; //map 3.3V to 12 bit ADC resolution
	MP->throttle_max=(Para1[35]<<12)/33; //map 3.3V to 12 bit ADC resolution

	memcpy(&MP->assist_profile[0][0],&Para2[0],30);
	for (k=0; k < 4; k++){
		MP->assist_settings[k][0]=Para1[k*2+41];
		MP->assist_settings[k][1]=Para1[k*2+50];
		MP->assist_settings[k][2]=Para0[k*2+2];
	}
	MP->assist_settings[4][0]=Para1[48];
	MP->assist_settings[4][1]=Para1[57];
	MP->assist_settings[4][2]=Para0[9];
}
