/*
 * CAN_Display.h
 *
 *  Created on: 19.10.2025
 *      Author: stancecoke
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

#ifndef INC_CAN_DISPLAY_H_
#define INC_CAN_DISPLAY_H_
#include "main.h"

void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS);
void display_init(void);

typedef struct
{

	uint8_t       	target;
	uint8_t       	source;
	uint8_t       	operation;
	uint16_t       	command;


}Ext_ID_t;

#endif /* INC_CAN_DISPLAY_H_ */


