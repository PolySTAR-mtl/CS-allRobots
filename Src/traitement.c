/****************
   Description : Fait la laison entre la télécommande et les consignes moteurs
   Auteur : Sébastien FAGUET
*****************/


#include "traitement.h"
#include "pilotes.h"
#include "canon.h"

#define MAX_BASE_SPEED_COEFF  10

/* On récupère les variables exterieurs */
extern receiver_RadioController_t receiver_RadioController;	
extern motor_t motors[MAX_MOTORS];
extern pilote_t pilote;

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
	if(receiver_RadioController.keyboard_mode){
		double chassis_w;
		double tourelle_yaw;
		if(receiver_RadioController.data.kb.bit.CTRL){
			chassis_w = -receiver_RadioController.data.mouse.x;
			tourelle_yaw = 0;
		}else{
			chassis_w = 0;
			tourelle_yaw = receiver_RadioController.data.mouse.x;
		}
		add_consigne_position(&motors[TOURELLE_PITCH], receiver_RadioController.data.mouse.y, pilote.sensitivity_mouse_y);
		add_consigne_position(&motors[TOURELLE_YAW], tourelle_yaw, pilote.sensitivity_mouse_x);
		
		chassis_consigne(receiver_RadioController.data.kb.bit.W - receiver_RadioController.data.kb.bit.S, 
											receiver_RadioController.data.kb.bit.D - receiver_RadioController.data.kb.bit.A, 
											chassis_w);
		
		
		if(receiver_RadioController.data.mouse.l){
			canon_shoot(0.15, 2000);
		}else if(receiver_RadioController.data.mouse.r){
			canon_shoot(0.20, 5000);
		}else{
			canon_shoot_end();
		}
	}else{		
		add_consigne_position(&motors[TOURELLE_PITCH], receiver_RadioController.data.ch2_float, pilote.sensitivity_ch_2);
		add_consigne_position(&motors[TOURELLE_YAW], 	receiver_RadioController.data.ch1_float, pilote.sensitivity_ch_1);
	
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
		
		chassis_consigne(receiver_RadioController.data.ch4, receiver_RadioController.data.ch3, receiver_RadioController.data.wheel); 
	}
}

void chassis_consigne(double Vx, double Vy, double W){ 
	/*
		Vx: Avant / Arrière
		Vy:	Translation gauche / Droite
		W: Rotation
	*/
	double sensitivity_Vx;
	double sensitivity_Vy;
	double sensitivity_W;
	if(receiver_RadioController.keyboard_mode){
		sensitivity_Vx = pilote.sensitivity_chassis_keyboard_Vx;
		sensitivity_Vy = pilote.sensitivity_chassis_keyboard_Vy;
		sensitivity_W = pilote.sensitivity_chassis_mouse_W;
	}else{
		sensitivity_Vx = pilote.sensitivity_chassis_RC_Vx;
		sensitivity_Vy = pilote.sensitivity_chassis_RC_Vy;
		sensitivity_W = pilote.sensitivity_chassis_RC_W;
	}
	motors[FRONT_LEFT].consigne 	= sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W;
	motors[FRONT_RIGHT].consigne 	= -(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W);
	motors[BACK_RIGHT].consigne 	= -(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W);
	motors[BACK_LEFT].consigne 		= sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W; 
}	
