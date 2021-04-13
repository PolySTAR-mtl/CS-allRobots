/****************
   Description : Gestion des informations du Jetson
   Auteurs : Arthur VAN BETSBRUGGE, Ariane BOYCZUM-AUGER, Katherine ZAMUDIO-TURCOTTE, Marc-Antoine HIEN
	 Periphériques : UART 7
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
	uint8_t switch_target; //Change la cible: 0x00; rien, 0x4C; change pour cible à gauche; change pour cible à droite
	uint8_t switch_target_mode; //Change le type de cible: 0x72; robot, 0x52: rune 
} switch_informations_t;

typedef __packed struct
{
	uint8_t target_located; 
	uint16_t teta_target_location; //mrad
	int16_t phi_target_location; //mrad
	uint16_t d_target_location; //mm
} robot_target_coordinates_t;

typedef __packed struct
{
	uint8_t target_located; 
	uint8_t rune_type;
	uint8_t rune_direction;
	uint16_t teta_target_location; 
	uint16_t phi_target_location; 
	uint16_t d_target_location; 
	uint16_t rune_time;
} rune_target_coordinates_t;

typedef __packed struct
{
	switch_informations_t switch_informations;
	robot_target_coordinates_t robot_target_coordinates;
	rune_target_coordinates_t rune_target_coordinates;
} jetson_t;

void jetson_uart_send_command(uint8_t command);

void jetson_callback_handler(int8_t lenght);

void uart7_init(void);

#endif // JETSON_H
