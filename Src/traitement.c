/****************
   Description : Fait la laison entre la télécommande et les consignes moteurs
   Auteur : Sébastien FAGUET
*****************/


#include "traitement.h"

#define MAX_BASE_SPEED_COEFF  10

/* On récupère les variables exterieurs */
extern receiver_RadioController_t receiver_RadioController;	
extern motor_t motors[MAX_MOTORS];

/* Calcul les pids de tous les moteurs (calcul des commandes en fonction des consignes */
void traitement_pids_compute(){
	for(int i = 0; i < MAX_MOTORS ; i++){
		if (pid_need_compute(&motors[i].pid)) {
			pid_compute(&motors[i].pid);
			//if(i == FEEDER) pid_debug_uart(&motors[i].pid);
		}
	}
}

/* Fonctions qui fait les liens entre les entrées (capteurs, radio controller, CV, ...) et les sorties (consignes moteurs), on peut créer plusieurs traitements */
void traitement_1(){
	add_consigne_position(&motors[TOURELLE_PITCH], (float)receiver_RadioController.data.ch2_float, 0.00005);
	add_consigne_position(&motors[TOURELLE_YAW], 	(float)receiver_RadioController.data.ch1_float, -0.00005);
			
	if(motors[TOURELLE_PITCH].info.angle_360 > 293){
		BOARD_LED_GREEN_ON();
	}else{
		BOARD_LED_GREEN_OFF();
	}
	
	
	switch(receiver_RadioController.data.sw1){
		case 1:
			motors[FEEDER].consigne = 0;
			break;
		case 3:
			motors[FEEDER].consigne = 5000;
			break;
		case 2:
			motors[FEEDER].consigne = 10000;
			break;
	}
	switch(receiver_RadioController.data.sw2){
		case 1:
			PWM_SetAllDuty(&htim1, 0.0, 0.0);
			break;
		case 3:
			PWM_SetAllDuty(&htim1, 0.25, 0.25);
			break;
		case 2:
			PWM_SetAllDuty(&htim1, 0.50, 0.50);
			break;
	}
	
	
	motors[FRONT_LEFT].consigne 	= MAX_BASE_SPEED_COEFF*(receiver_RadioController.data.ch4 - receiver_RadioController.data.ch3 - 0.3*receiver_RadioController.data.wheel);
	motors[FRONT_RIGHT].consigne 	= -MAX_BASE_SPEED_COEFF*(receiver_RadioController.data.ch4 + receiver_RadioController.data.ch3 + 0.3*receiver_RadioController.data.wheel);
	motors[BACK_RIGHT].consigne 	= -MAX_BASE_SPEED_COEFF*(receiver_RadioController.data.ch4 - receiver_RadioController.data.ch3 + 0.3*receiver_RadioController.data.wheel);
	motors[BACK_LEFT].consigne 		= MAX_BASE_SPEED_COEFF*(receiver_RadioController.data.ch4 + receiver_RadioController.data.ch3 - 0.3*receiver_RadioController.data.wheel); 
}
