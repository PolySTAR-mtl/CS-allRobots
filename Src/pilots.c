/****************
   Description : Gestion des pilotes, sauvegarde de leur configuration
   Auteur : S�bastien FAGUET
*****************/
                  
#include "pilots.h"

pilot_t pilot;

/* Fonction qui premet de configurer le robot */
void pilot_init(uint8_t pilot_id){
	/*
	Robot ID:
		0: Antonin;
	*/
	/* Receiver */
	pilot.sensitivity_RC_deadzone = 10; //Between 0 and 6600
	pilot.sensitivity_ch_1 = 0.00003;
	pilot.sensitivity_ch_2 = 0.00001;

	pilot.sensitivity_chassis_RC_Vx = 20;
	pilot.sensitivity_chassis_RC_Vy = 20;
	pilot.sensitivity_chassis_RC_W = 6;
	
	/* Mouse Keyboard */
	pilot.sensitivity_mouse_deadzone = 50; //Between 0 and 6600
	pilot.sensitivity_mouse_x = 0.0005;
	pilot.sensitivity_mouse_y = 0.0005;

	pilot.sensitivity_chassis_keyboard_Vx = 4300;
	pilot.sensitivity_chassis_keyboard_Vy = 4300;
	pilot.sensitivity_chassis_mouse_W = 160;
	
	/*Coefficient puissance chassis*/
	pilot.coefficient_ShiftChassis = 0.5;
	pilot.coefficient_EChassis = 1.2;
	
	switch(pilot_id){ //Configuration personnalis�e
		/* Antonin */
		case PILOTE_ANTONIN:
			break;
		default:
			while(1);
	}
}
