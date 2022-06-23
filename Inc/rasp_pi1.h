/****************
   Description : Gestion des fonctionnalit?s du referee System
   Auteur : S?bastien FAGUET
	 Periph?riques : UART 6
*****************/

#ifndef RASP_PI1_H
#define RASP_PI1_H

#include "main.h"                                                                                     
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "oled.h"
#include "motors.h"

#define UART6_MAX_LEN     30
#define UART6_RX_BUFFLEN  50

typedef __packed struct
{
	float value;
	
} rasp_pi1_t;


/* Function called when data is recieved from referee system */
void rasp_pi1_callback_handler(int8_t length);

void uart6_init(void);

#endif // __RASP_PI1_H__
