/*
 * parser.c
 *
 *  Created on: 30.11.2025
 *      Author: stancecoke
 */

#include "main.h"
#include "CAN_Display.h"

uint16_t l=0;

void parse_DPparams(MotorParams_t* MP){
	MP->system_voltage = Para1[0];
	MP->battery_current_max=Para1[1]*1000;
	MP->max_voltage = Para1[2];
	MP->phase_current_max=Para1[9]*1000/CAL_I; //uses field Max Current on Low Charge
	MP->gear_ratio=Para1[19];
	MP->MagicNumber=Para1[24]+(Para1[25]<<8);
	MP->throttle_offset=(Para1[34]<<12)/33; //map 3.3V to 12 bit ADC resolution
	MP->throttle_max=(Para1[35]<<12)/33; //map 3.3V to 12 bit ADC resolution
	MP->voltage_min=(Para1[3]+(Para1[4]<<8))/CAL_BAT_V;
	MP->legalflag=Para1[14];
	if (!Para1[18])MP->reverse=-1;
	else MP->reverse=1;
	MP->pulses_per_revolution=Para1[20];

	MP->PAS_timeout= Para1[38]*400; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	MP->ramp_end = 11250/Para1[39]; //use field Current Shedding Time (Ramp Down), calculate timer tics from theshold cadence

	memcpy(&MP->assist_profile[0][0],&Para2[0],30);
	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=Para1[k*2+41]; //current limit (%)
		MP->assist_settings[k+1][1]=Para1[k*2+50]; //speed limit (%)
		MP->assist_settings[k+1][2]=Para0[k*2+2];  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=Para1[48];
	MP->assist_settings[5][1]=Para1[57];
	MP->assist_settings[5][2]=Para0[9];

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;

	//Torque override Threshold
	for (k=0; k < 4; k++){
		MP->TQO_threshold[k+1]=Para0[k*4+12]+(Para0[k*4+13]<<8);  // use field Assist ratio
	}
	MP->TQO_threshold[5]=Para0[26]+(Para0[27]<<8);
	MP->TQO_threshold[0]=3299;
}


void parse_MOparams(MotorParams_t* MP){
	Para1[0] = MP->system_voltage;
	Para1[1] = MP->battery_current_max/1000;
	Para1[2] = MP->max_voltage;
	Para1[3] = (MP->voltage_min*CAL_BAT_V)&0xFF;
	Para1[4] = ((MP->voltage_min*CAL_BAT_V)>>8)&0xFF;
	Para1[9]= MP->phase_current_max*CAL_I/1000;
	Para1[14]= MP->legalflag;
	if (MP->reverse==-1)Para1[18]=0;
	else Para1[18]=1;
	Para1[19]= MP->gear_ratio;
	Para1[20]= MP->pulses_per_revolution;
	Para1[24]= (MP->MagicNumber)&0xFF;
	Para1[25]= ((MP->MagicNumber)>>8)&0xFF;
	Para1[34]= (MP->throttle_offset*33)>>12; //map 3.3V to 12 bit ADC resolution
	Para1[35]= (MP->throttle_max*33)>>12; //map 3.3V to 12 bit ADC resolution
	Para1[38]= MP->PAS_timeout*10/4000; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	Para1[39]= 11250/MP->ramp_end; //use field Current Shedding Time (Ramp Down), calculate threshold cadence from timer tics
	memcpy(&Para2[0],&MP->assist_profile[0][0],30);
	for (k=0; k < 4; k++){
		Para1[k*2+41]= MP->assist_settings[k+1][0]; //current limit (%)
		Para1[k*2+50]= MP->assist_settings[k+1][1]; //speed limit (%)
		Para0[k*2+2]= MP->assist_settings[k+1][2];  //ride mode (Acceleraton in Canable Tool)
	}
	Para1[48]= MP->assist_settings[5][0];
	Para1[57]= MP->assist_settings[5][1];
	Para0[9]= MP->assist_settings[5][2];

	//Torque override Threshold
	for (k=0; k < 4; k++){
		Para0[k*4+12]=MP->TQO_threshold[k+1]&0xFF;
		Para0[k*4+13]=MP->TQO_threshold[k+1]>>8;  // use field Assist ratio
	}


	Para0[26]= MP->TQO_threshold[5]&0xFF;
	Para0[27]= MP->TQO_threshold[5]>>8;

	update_checksum();
}

void InitEEPROM(MotorParams_t* MP){
	MP->TS_coeff=TS_COEF;
	MP->MagicNumber=202;
	MP->battery_current_max=BATTERYCURRENT_MAX;
	MP->gear_ratio=GEAR_RATIO;
	MP->throttle_offset=THROTTLE_OFFSET; //map 3.3V to 12 bit ADC resolution
	MP->throttle_max=THROTTLE_MAX; //map 3.3V to 12 bit ADC resolution
	MP->reverse=REVERSE;
	MP->pulses_per_revolution=PULSES_PER_REVOLUTION;
	MP->phase_current_max = PH_CURRENT_MAX;
	MP->voltage_min=VOLTAGE_MIN;
	MP->legalflag = LEGALFLAG;
	MP->PAS_timeout = PAS_TIMEOUT;
	MP->ramp_end = RAMP_END;
	MP->system_voltage = SYSTEM_VOLTAGE;
	MP->max_voltage = MAX_VOLTAGE;
	for (k=0; k < 6; k++){
		for (l=0; l < 7; l++){
			MP->assist_profile[k][l]=(k+1)*20;
		}
	}

	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=100; //current limit (%)
		MP->assist_settings[k+1][1]=100; //speed limit (%)
		MP->assist_settings[k+1][2]=TQFILTER;  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=100;
	MP->assist_settings[5][1]=100;
	MP->assist_settings[5][2]=TQFILTER;

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;

	for (k=0; k < 5; k++){
		MP->TQO_threshold[k+1]=3299;
	}
	MP->TQO_threshold[0]=3299;
	write_virtual_eeprom();
}
