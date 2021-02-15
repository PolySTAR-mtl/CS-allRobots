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
	pilote.sensitivity_ch_1 = 0.00005;
	pilote.sensitivity_ch_2 = 0.00005;
	pilote.sensitivity_mouse_x = 0.000005;
	pilote.sensitivity_mouse_y = 0.000005;

	pilote.sensitivity_chassis_RC_Vx = 10;
	pilote.sensitivity_chassis_RC_Vy = 10;
	pilote.sensitivity_chassis_RC_W = 3;

	pilote.sensitivity_chassis_keyboard_Vx = 1000;
	pilote.sensitivity_chassis_keyboard_Vy = 1000;
	pilote.sensitivity_chassis_mouse_W = 5;
	
	switch(pilote_id){ //Configuration personnalisée
		/* Antonin */
		case PILOTE_ANTONIN:
			break;
		default:
			while(1);
	}
}
