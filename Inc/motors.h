/****************
   Description : Gestion des moteurs
   Auteur : S�bastien FAGUET
*****************/

#ifndef MOTORS
#define MOTORS

#include "main.h"
#include "PID.h"
#include "can.h"
#include "leds.h"
#include "tim.h"
#include "robot_configuration.h"
#include "rasp_pi1.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

enum motors {
	/* CAN motors */
	M3508,
	GM6020,
	M2006,
	general_motor,
	/* PWM motors */
	M2305,
};

typedef struct{
    uint16_t    angle;		// 0 - 8191
    float  		angle_360;	// 0.0 - 360.0 deg
    float     	speed; 		// RPM
    int16_t     torque;
	int8_t		temp;
} motor_info_t;


// One struct per motor
typedef struct
{
	uint16_t   	can_rx_id;
	uint16_t    can_tx_frame;
	uint16_t    can_tx_id;
	motor_info_t info;				// Specific info about motor
	int type; 						// Motor type, eg:GM6020
	struct pid_controller pid;		// Controller
	float setpoint;
	float command;
	float direction; 		// 1 or -1
	float MAX_POSITION;
	float MIN_POSITION;
	uint32_t signOfLife_tick;
	char debug_name[40];
} motor_t;

/* Sends commands on CAN bus for all CAN motors*/
void can_send_command(void);

/* Called when information is received from a motor */
void can_motors_callback_handler(int16_t rx_id, uint8_t* rx_buff);

/* Modifies setpoint after verifying position limits */
void add_setpoint_position(motor_t* motor, float value, float coeff);

/* Initializes turret to starting motor values */
void init_turret_data(motor_t* motor);

/* InitializesCAN 1 */
void can1_init(void);

/* Initializes TIMER 1 for PWM */
void PWM_init(void);

/* Sets duty cycle of all PWM channels */
void PWM_SetAllDuty(TIM_HandleTypeDef *tim, float duty_ch1, float duty_ch2);

/* Scales all PWM duty cycles between 0 and 1 */
void PWM_ScaleAll(TIM_HandleTypeDef *tim, bool switchRotationalDirection);

/* Fills motor.info structure with information from motor */
void fill_motor_data (motor_t* motor, uint8_t* rx_buff);

/* Deal with turret errors */
void throw_turret_error(void);

#endif
