/****************
   Description : Configuration du robot
   Auteur : Sébastien FAGUET
*****************/

#ifndef PILOTE
#define PILOTE

#include "main.h"

#define PILOTE_ANTONIN 0

typedef struct
{
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
} pilote_t;


/* Fonction qui premet de configurer le pilote */
void piloteInit(uint8_t pilote_id);

#endif
