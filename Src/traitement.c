/****************
   Description : Fait la laison entre la t�l�commande et les consignes moteurs
   Auteur : S�bastien FAGUET
*****************/


#include "traitement.h"
#include "pilotes.h"
#include "canon.h"
#include "robot_configuration.h"


#define MAX_BASE_SPEED_COEFF  10
#define PI 3.14159265358979323846

/* On r�cup�re les variables exterieurs */
extern receiver_RadioController_t receiver_RadioController;	
extern motor_t motors[MAX_MOTORS];
extern pilote_t pilote;
extern jetson_t jetson;


extern float vitesse_snail; 
extern float cadence_coeff;
extern bool  inversion_gauchedroite;
extern bool  inversion_avantarriere;

/*mode de controle actuel*/
enum mode_assistance_ai_t mode_assistance_ai = automatique;

/* Calcul les pids de tous les moteurs (calcul des commandes en fonction des consignes */
void traitement_pids_compute(){
	for(int i = 0; i < MAX_MOTORS ; i++){
		if (pid_need_compute(&motors[i].pid)) {
			pid_compute(&motors[i].pid);
			//if(i == FEEDER) pid_debug_uart(&motors[i].pid);
		}
	}
}

/*Returns true if controller is in neutral state*/
bool isControllerNeutral(){
	
	// Joysticks in central position
	if(receiver_RadioController.data.ch1_float != 0){
		return false;
	}
	if(receiver_RadioController.data.ch2_float != 0){
		return false;
	}
	if(receiver_RadioController.data.ch3_float != 0){
		return false;
	}
	if(receiver_RadioController.data.ch4_float != 0){
		return false;
	}
	
	// Switch 1 in any position
	/*if(receiver_RadioController.data.sw1 != 1){
		return false;
	}*/
	
	// Switch 2 (SNAILS) in lower position
	if(receiver_RadioController.data.sw2 != 2){
		return false;
	}
	
	// Wheel not touched
	/*if(receiver_RadioController.data.wheel != 0){
		return false;
	}*/
	
	return true;
}


/* Fonctions qui fait les liens entre les entr�es (capteurs, radio controller, CV, ...) et les sorties (consignes moteurs), on peut cr�er plusieurs traitements */
void traitement_1(){
	
	if(receiver_RadioController.keyboard_mode){
		double chassis_w;
		double tourelle_yaw;
		if(!receiver_RadioController.data.kb.bit.CTRL){
			chassis_w = -receiver_RadioController.data.mouse.x;
			tourelle_yaw = 0;
		}else{
			chassis_w = 0;
			tourelle_yaw = receiver_RadioController.data.mouse.x;
		}
		
		/*gere l'assistance automatique*/
		//if(receiver_RadioController.data.kb.bit.Q) switch_assistance_ai();
		//if(mode_assistance_ai==automatique) auto_follow_target();
		
		
		
		add_consigne_position(&motors[TOURELLE_PITCH], receiver_RadioController.data.mouse.y, pilote.sensitivity_mouse_y);
		add_consigne_position(&motors[TOURELLE_YAW], tourelle_yaw, pilote.sensitivity_mouse_x);
		
		chassis_consigne(receiver_RadioController.data.kb.bit.W - receiver_RadioController.data.kb.bit.S, 
											receiver_RadioController.data.kb.bit.D - receiver_RadioController.data.kb.bit.A, 
											chassis_w);
		
		
		if(receiver_RadioController.data.mouse.l){
			canon_shoot(vitesse_snail/2, cadence_coeff * 1000);
		}else if(receiver_RadioController.data.mouse.r){
			canon_shoot(vitesse_snail, cadence_coeff * 1000);
		}else{
			canon_shoot_end();
		}
	}else{	
		
		
		//if(mode_assistance_ai==automatique) auto_follow_target();
		
		add_consigne_position(&motors[TOURELLE_PITCH], receiver_RadioController.data.ch2_float, pilote.sensitivity_ch_2);
		add_consigne_position(&motors[TOURELLE_YAW], 	receiver_RadioController.data.ch1_float, pilote.sensitivity_ch_1);
	
		switch(receiver_RadioController.data.sw1){
			case 1:
				break;
			case 3:
				break;
			case 2:
				break;
		}
		switch(receiver_RadioController.data.sw2){
			case 2:
				canon_shoot(0, 0);
				break;
			case 3:
				canon_shoot(vitesse_snail/2, 1000);
				break;
			case 1:
				canon_shoot(vitesse_snail, 1000);
				break;
		}
		
		chassis_consigne(receiver_RadioController.data.ch4, receiver_RadioController.data.ch3, receiver_RadioController.data.wheel); 
	}
}

void chassis_consigne(double Vx, double Vy, double W){ 
	/*
		Vx: Avant / Arri�re
		Vy:	Translation gauche / Droite
		W: Rotation
	*/
	double sensitivity_Vx;
	double sensitivity_Vy;
	double sensitivity_W;
	double sensitivity_deadzone;
	
	double coefficientShiftChassis;
	double coefficientEChassis;
	
	if (inversion_gauchedroite) {
		// channels 3 et 4 inversés... le 3 c'est en x et le 4 c'est en y normalement (selon la datasheet)
		// mais la ligne 132 de ce fichier étant erronée, on inverse le gauche-droite en inversant le Y et non le X
		Vy = -Vy;
	}
	
	if (inversion_avantarriere) {
		// channels 3 et 4 inversés... le 3 c'est en x et le 4 c'est en y normalement (selon la datasheet)
		// mais la ligne 132 de ce fichier étant erronée, on inverse le avant-arrière en inversant le X et non le Y
		Vx = -Vx;
	}
	
	if(receiver_RadioController.keyboard_mode){
		sensitivity_Vx = pilote.sensitivity_chassis_keyboard_Vx;
		sensitivity_Vy = pilote.sensitivity_chassis_keyboard_Vy;
		sensitivity_W = pilote.sensitivity_chassis_mouse_W;
		sensitivity_deadzone = 0;
		
		coefficientShiftChassis = pilote.coefficientShiftChassis;
		coefficientEChassis = pilote.coefficientEChassis;
	}else{
		sensitivity_Vx = pilote.sensitivity_chassis_RC_Vx;
		sensitivity_Vy = pilote.sensitivity_chassis_RC_Vy;
		sensitivity_W = pilote.sensitivity_chassis_RC_W;
		sensitivity_deadzone = pilote.sensitivity_RC_deadzone;
	}
	if(Vx > -sensitivity_deadzone && Vx < sensitivity_deadzone) Vx = 0;
	if(Vy > -sensitivity_deadzone && Vy < sensitivity_deadzone) Vy = 0;
	
	if(receiver_RadioController.data.kb.bit.SHIFT){ //applique les coefficients sur la vitesse du chassis en focntion de la touche appuyee
		motors[FRONT_LEFT].consigne 	= coefficientShiftChassis * (sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W);
		motors[FRONT_RIGHT].consigne 	= coefficientShiftChassis * (-(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W));
		motors[BACK_RIGHT].consigne 	= coefficientShiftChassis * (-(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W));
		motors[BACK_LEFT].consigne 		= coefficientShiftChassis * (sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W);
	} else if(receiver_RadioController.data.kb.bit.E){ 
		motors[FRONT_LEFT].consigne 	= coefficientEChassis * (sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W);
		motors[FRONT_RIGHT].consigne 	= coefficientEChassis * (-(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W));
		motors[BACK_RIGHT].consigne 	= coefficientEChassis * (-(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W));
		motors[BACK_LEFT].consigne 		= coefficientEChassis * (sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W);
	}else{
		motors[FRONT_LEFT].consigne 	= sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W;
		motors[FRONT_RIGHT].consigne 	= -(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W);
		motors[BACK_RIGHT].consigne 	= -(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W);
		motors[BACK_LEFT].consigne 		= sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W; 
	}
	
}	

/*change le mode de visee
On pourrait ajouter un coefficient qui est 1 en manuel et <1 en automatique qui s'applique
sur les consigne donnees par le pilote pour la tourelle
*/
void switch_assistance_ai(void){
	switch(mode_assistance_ai){
		case automatique : 
			mode_assistance_ai = manuel; 
		break;
		case manuel : 
			mode_assistance_ai = automatique; 
		break;
	}
}

/*
Fonction qui controle la position des moteurs GM6020 de la tourelle pour que celle-ci point une cible
*/
void auto_follow_target(void){
  
	uint16_t teta,d;
	int16_t phi;
	uint8_t target_located;
	
	jetson.switch_informations.switch_target_mode = 0x72; //Le target mode doit etre stocke autre part (car la trame switch_information on l'envoie on ne la recoit pas)
	switch(jetson.switch_informations.switch_target_mode){
		case 0x72 : 	/*Si la cible est une robot*/
			teta = jetson.robot_target_coordinates.teta_target_location;
			phi = jetson.robot_target_coordinates.phi_target_location;
			d = jetson.robot_target_coordinates.d_target_location;
			target_located = jetson.robot_target_coordinates.target_located;
			break;
		case 0x52 : /*Si la cible est une rune*/
			teta = jetson.rune_target_coordinates.teta_target_location;
			phi = jetson.rune_target_coordinates.phi_target_location;
			d = jetson.rune_target_coordinates.d_target_location;
	  	target_located = jetson.rune_target_coordinates.target_located;
			break;
	}
	uart_debug();
	
	// to delete soon (keep lines 181-194) :
			teta = jetson.robot_target_coordinates.teta_target_location;
			phi = jetson.robot_target_coordinates.phi_target_location;
			d = jetson.robot_target_coordinates.d_target_location;
			target_located = jetson.robot_target_coordinates.target_located;
	//
	
	if(target_located=='Y'){/*change les consignes juste si une cible est localisee*/
		float consigne_yaw = motors[TOURELLE_YAW].consigne + phi*0.0001; 
		float consigne_pitch = motors[TOURELLE_PITCH].consigne + (teta - (PI/2)*1000)*0.0001; //1 millirad = (180/PI)/1000 degres
		
		if(phi > 0){
			consigne_yaw = motors[TOURELLE_YAW].consigne + 0.0005; 
		}else{
			consigne_yaw = motors[TOURELLE_YAW].consigne - 0.0005;
		}
		consigne_yaw = motors[TOURELLE_YAW].consigne;
		if(teta > (PI/2)*1000){
			consigne_pitch = motors[TOURELLE_PITCH].consigne + 0.0005; 
		}else{
			consigne_pitch = motors[TOURELLE_PITCH].consigne - 0.0005;	
		}
		
		//Peut �tre utiliser add_consigne_position() a la place des lignes 246 � 272
		
		//consigne_pitch = motors[TOURELLE_PITCH].consigne;
		//S'assure que la consigne ne est entre 0 et 360 
		if(consigne_yaw > 360) consigne_yaw -= (float) 360.0;
		if(consigne_yaw < 0) 	consigne_yaw += (float) 360.0;
	
		if(consigne_pitch > 360) consigne_pitch -= (float) 360.0;
		if(consigne_pitch < 0) 	consigne_pitch += (float) 360.0;
	
		//S'assure que la consigne respecte les limites min et max
		if(motors[TOURELLE_YAW].MAX_POSITION > 0 && consigne_yaw > motors[TOURELLE_YAW].MAX_POSITION) {
			consigne_yaw = motors[TOURELLE_YAW].MAX_POSITION;
		}
		if(motors[TOURELLE_YAW].MIN_POSITION > 0 && consigne_yaw < motors[TOURELLE_YAW].MIN_POSITION){
			consigne_yaw = motors[TOURELLE_YAW].MIN_POSITION;
		}
		if(motors[TOURELLE_PITCH].MAX_POSITION > 0 && consigne_pitch > motors[TOURELLE_PITCH].MAX_POSITION){
			consigne_pitch = motors[TOURELLE_PITCH].MAX_POSITION;
		}
		if(motors[TOURELLE_PITCH].MIN_POSITION > 0 && consigne_pitch < motors[TOURELLE_PITCH].MIN_POSITION){
			consigne_pitch = motors[TOURELLE_PITCH].MIN_POSITION;
		}
		
		/*
		TODO : ajustement du "consigne_pitch" en fonction de la distance "d" de la cible
		*/
		
		/*Donne la nouvelle consigne aux moteurs*/
		motors[TOURELLE_YAW].consigne = consigne_yaw;
		motors[TOURELLE_PITCH].consigne = consigne_pitch;
	}
}



