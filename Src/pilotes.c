/****************
   Description : Gestion des pilotes, sauvegarde de leur configuration
   Auteur : Sébastien FAGUET
*****************/
                  
#include "pilotes.h"

pilote_t pilote;

/* Fonction qui premet de configurer le robot */
void piloteInit(uint8_t pilote_id){
	/*
	Robot ID:
		0: Antonin;
	*/
	/* Receiver */
	pilote.sensitivity_RC_deadzone = 100; //Between 0 and 6600
	pilote.sensitivity_ch_1 = 0.00003;
	pilote.sensitivity_ch_2 = 0.00003;

	pilote.sensitivity_chassis_RC_Vx = 20;
	pilote.sensitivity_chassis_RC_Vy = 20;
	pilote.sensitivity_chassis_RC_W = 6;
	
	/* Mouse Keyboard */
	pilote.sensitivity_mouse_deadzone = 50; //Between 0 and 6600
	pilote.sensitivity_mouse_x = 0.0005;
	pilote.sensitivity_mouse_y = 0.0005;

	pilote.sensitivity_chassis_keyboard_Vx = 4300;
	pilote.sensitivity_chassis_keyboard_Vy = 4300;
	pilote.sensitivity_chassis_mouse_W = 160;
	
	pilote.coefficientShiftChassis = 0.5;
	pilote.coefficientEChassis = 1;
	
	switch(pilote_id){ //Configuration personnalisée
		/* Antonin */
		case PILOTE_ANTONIN:
			break;
		default:
			while(1);
	}
}
