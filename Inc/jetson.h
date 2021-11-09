/****************
   Description : Gestion des informations du Jetson
   Auteurs : Arthur VAN BETSBRUGGE, Ariane BOYCZUM-AUGER, Katherine ZAMUDIO-TURCOTTE, Marc-Antoine HIEN
	 Periph�riques : UART 7
*****************/


#ifndef JETSON_H
#define JETSON_H

#include "main.h"                                                                                     
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "oled.h"
#include "motors.h"

#define UART7_MAX_LEN     30
#define UART7_RX_BUFFLEN  50


typedef __packed struct
{
	// TODO : switch_target comment
	uint8_t switch_target; 		// Switches taget: 0x00; no target, 0x4C; left target; right target
	uint8_t switch_target_mode; // Switches target type: 0x72; robot, 0x52: rune 
} switch_information_t;

typedef __packed struct
{
	uint8_t  target_located; 
	uint16_t theta_target_location; //mrad
	int16_t  phi_target_location; 	//mrad
	uint16_t d_target_location; 	//mm
} robot_target_coordinates_t;

typedef __packed struct
{
	uint8_t  target_located; 
	uint8_t  rune_type;
	uint8_t  rune_direction;
	uint16_t theta_target_location; 
	uint16_t phi_target_location; 
	uint16_t d_target_location; 
	uint16_t rune_time;
} rune_target_coordinates_t;

typedef __packed struct
{
	switch_information_t switch_information;
	robot_target_coordinates_t robot_target_coordinates;
	rune_target_coordinates_t  rune_target_coordinates;
} jetson_t;

void jetson_uart_send_command(uint8_t command);

void jetson_callback_handler(int8_t lenght);

void uart7_init(void);

#endif // JETSON_H
