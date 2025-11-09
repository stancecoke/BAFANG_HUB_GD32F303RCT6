/*
 * CAN_Display.c
 *
 *  Created on: 19.10.2025
 *  Author: stancecoke
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


#include "main.h"
#include "CAN_Display.h"

Ext_ID_t Ext_ID;
void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS);


uint16_t distance =0;
uint16_t delay_counter =0;


void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS){

	Ext_ID.command = (receive_message.rx_efid)&0xFFFF;
	Ext_ID.operation = (receive_message.rx_efid>>16)&0x07; //only 3 bit width
	Ext_ID.target = (receive_message.rx_efid>>19)&0x1F; //only 5 bit width
	Ext_ID.source = (receive_message.rx_efid>>24)&0x1F;

	if(Ext_ID.target==2)sendCAN_Tx(MP,MS);
	if(Ext_ID.command==0x6300){
		switch (receive_message.rx_data[1]){
			case 0:
				MS->assist_level=0;
				break;
			case 1:
				MS->assist_level=1;
				break;
			case 0x0B:
				MS->assist_level=2;
				break;
			case 0x0C:
				MS->assist_level=3;
				break;
			case 0x0D:
				MS->assist_level=4;
				break;
			case 0x02:
				MS->assist_level=5;
				break;
			case 0x15:
				MS->assist_level=6;
				break;
			case 0x16:
				MS->assist_level=7;
				break;
			case 0x17:
				MS->assist_level=8;
				break;
			case 0x03:
				MS->assist_level=9;
				break;
		}
		if (receive_message.rx_data[1]==6)MS->pushassist_flag=SET;
		else MS->pushassist_flag=RESET;
		if (receive_message.rx_data[2]&0b1)MS->light_flag=SET;
		else MS->light_flag=RESET;
		if (receive_message.rx_data[2]&0b10)MS->button_up_flag=SET;
		else MS->button_up_flag=RESET;
		if (receive_message.rx_data[2]&0b100000)MS->button_down_flag=SET;
		else MS->button_down_flag=RESET;

	}

}


void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){

	switch (Ext_ID.command){

		case 0x6300: //speed and power
			/* initialize transmit message */
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83201;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			transmit_message.tx_data[0] = (MS->Speed)&0xFF;
			transmit_message.tx_data[1] = ((MS->Speed)>>8)&0xFF;
			transmit_message.tx_data[2] = (MS->Battery_Current/10)&0xFF;
			transmit_message.tx_data[3] = ((MS->Battery_Current/10)>>8)&0xFF;
			transmit_message.tx_data[4] = (MS->Voltage/10)&0xFF;
			transmit_message.tx_data[5] = ((MS->Voltage/10)>>8)&0xFF;
			transmit_message.tx_data[6] = 0x32; //internal temperature
			transmit_message.tx_data[7] = 0x3C; //motor temperature

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x6301: //battery and distance
			/* initialize transmit message */
			if(delay_counter<10)delay_counter++;
			else
				if(distance<100)distance++;
				else distance = 0;
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83200;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			transmit_message.tx_data[0] = 50;//battery percentage
			transmit_message.tx_data[1] = distance;
			transmit_message.tx_data[2] = 0x06;
			transmit_message.tx_data[3] = MS->cadence; //cadence
			transmit_message.tx_data[4] = MS->torque_on_crank&0xFF; //torque mV LSB
			transmit_message.tx_data[5] = (MS->torque_on_crank>>8)&0xFF; //torque mv MSB
			transmit_message.tx_data[6] = 0x0F;//range LSB
			transmit_message.tx_data[7] = 0x0F;//range MSB

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x6302: //to do
			/* initialize transmit message */
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83205;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 2;
			transmit_message.tx_data[0] = MS->calories&0xFF; //calories
			transmit_message.tx_data[1] = (MS->calories>>8)&0xFF;


			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;
	}//end case
}
