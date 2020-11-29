/****************
   Description : Gestion des moteurs
   Auteur : Sébastien FAGUET
*****************/

#ifndef MOTORS
#define MOTORS

#include "main.h"
#include "PID.h"
#include "can.h"
#include "leds.h"
#include "robot_configuration.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


enum motors {
	/* can motors */
	M3508,
	GM6020,
	M2006,
	general_motor,
	/* pwm motors */
	M2305,
};

typedef struct{
    uint16_t    angle;			//0 à 8191
    float    		angle_360;	//0.0 à 360.0 deg
    float     	speed; //rpm
    int16_t     torque;
		int8_t			temp;
} motor_info_t;


// Une struct par moteur
typedef struct
{
	uint16_t   	can_rx_id;
	uint16_t    can_tx_frame;
	uint16_t    can_tx_id;
	motor_info_t info;				// Specific info about the motor
	int type; 								// Type du moteur, ex:GM6020
	//pid_struct_t pid_pos;			// Controler
	struct pid_controller pid;		// Controler
	float consigne;
	float command;
	float MAX_POSITION;
	float MIN_POSITION;
	uint32_t signOfLife_tick;
	char debug_name[50];
} motor_t;



/* Envoie les commandes sur le bus CAN pour tous les moteurs CAN */
void can_send_command(void);

/* Fonction appelée lors de la réception d'une information provenant d'un moteur */
void can_motors_callback_handler(int16_t rx_id, uint8_t* rx_buff);

/* Modifie la consigne tout en vérifiant les limites de postion */
void add_consigne_position(motor_t* motor, float value);

/* Initialise le CAN 1 */
void can1_init(void);

#endif
