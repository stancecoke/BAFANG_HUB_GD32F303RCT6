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




void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS){
	Ext_ID.subcommand = (receive_message.rx_efid)&0xFF;
	Ext_ID.command = (receive_message.rx_efid>>8)&0xFF;
	Ext_ID.operation = (receive_message.rx_efid>>16)&0x07; //only 3 bit width
	Ext_ID.target = (receive_message.rx_efid>>19)&0x1F; //only 5 bit width
	Ext_ID.source = (receive_message.rx_efid>>24)&0x1F;



}


void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){

}
