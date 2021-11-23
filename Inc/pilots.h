/****************
   Description : Configuration du robot
   Auteur : S�bastien FAGUET
*****************/

#ifndef PILOTE
#define PILOTE

#include "main.h"

#define PILOT_DEFAULT 0
#define PILOT_ANTONIN 1

typedef struct
{
	float sensitivity_RC_deadzone;
	float sensitivity_mouse_deadzone;
	float sensitivity_ch_1;
	float sensitivity_ch_2;
	float sensitivity_mouse_x;
	float sensitivity_mouse_y;
	
	double sensitivity_chassis_RC_Vx;
	double sensitivity_chassis_RC_Vy;
	double sensitivity_chassis_RC_W;
	float sensitivity_chassis_keyboard_Vx;
	float sensitivity_chassis_keyboard_Vy;
	float sensitivity_chassis_mouse_W;
	
	float coefficient_ShiftChassis;
	float coefficient_EChassis;

} pilot_t;


/* Function that configures robot controls */
void pilot_init(uint8_t pilote_id);

#endif
