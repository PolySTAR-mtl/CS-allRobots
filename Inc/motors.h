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
#include "tim.h"
#include "robot_configuration.h"
#include "referee_system.h"
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
	struct pid_controller pid;		// Controler
	float consigne;
	float command;
	float direction; //1 ou -1
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
void add_consigne_position(motor_t* motor, float value, float coeff);

/* Initialise la tourelle aux valeurs de départ des moteurs */
void init_tourelle_data(motor_t* motor);

/* Initialise le CAN 1 */
void can1_init(void);

/* Initialise le TIMER 1 pour les PWM */
void PWM_init(void);

/* Set le duty cycle de tous les channels PWM */ 
void PWM_SetAllDuty(TIM_HandleTypeDef *tim, float duty_ch1, float duty_ch2);

/* scales all PWM duty cycles between 0 and 1 */
void PWM_ScaleAll(TIM_HandleTypeDef *tim, bool switchRotationalDirection);

/*Rempli la structure motor.info avec les données provenant du moteur */
void fill_motor_data (motor_t* motor, uint8_t* rx_buff);

#endif
