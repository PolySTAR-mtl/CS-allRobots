/****************
   Description : Gestion des pilotes, sauvegarde de leur configuration
   Auteur : S�bastien FAGUET
*****************/
                  
#include "pilots.h"

pilot_t pilot;

/* Function that configures robot controls */
void pilot_init(uint8_t pilot_id){
	/*
	Pilot ID:
		0: Default;
		1: Antonin;
	*/
	/* Receiver */
	pilot.sensitivity_RC_deadzone = 10; //Between 0 and 6600
	pilot.sensitivity_ch_1 = 0.00003;  // Turret YAW
	pilot.sensitivity_ch_2 = 0.00001;  // Turret PITCH

	pilot.sensitivity_chassis_RC_Vx = 20; // TO DO : TESTER AVEC DES VALEURS PLUS PETITES SUR LE ROBOT
	pilot.sensitivity_chassis_RC_Vy = 20;
	pilot.sensitivity_chassis_RC_W = 6;
	
	/* Mouse Keyboard */
	pilot.sensitivity_mouse_deadzone = 50; //Between 0 and 6600
	pilot.sensitivity_mouse_x = 0.0005;
	pilot.sensitivity_mouse_y = 0.0005;

	pilot.sensitivity_chassis_keyboard_Vx = 4300;
	pilot.sensitivity_chassis_keyboard_Vy = 4300;
	pilot.sensitivity_chassis_mouse_W = 160;
	
	/* Chassis speed multipliers */
	pilot.coefficient_ShiftChassis = 0.5;
	pilot.coefficient_EChassis = 1.2;
	
	// TO DO : Changer les valeurs par défaut pour chaque pilote dans les case
	switch(pilot_id){ // Personalised configuration
		case PILOT_DEFAULT:
			break;
		case PILOT_ANTONIN:
			break;
		default:
			while(1);
	}


}