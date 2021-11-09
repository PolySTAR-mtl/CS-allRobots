/****************
   Description : Gestion du cannon
   Auteur : S�bastien FAGUET
*****************/

                 
#include "canon.h"

#define TIME_BEFORE_START_FEEDER 10
#define TIME_BEFORE_STOP 100

extern motor_t motors[MAX_MOTORS];
extern refereeSystem_t refereeSystem;
uint32_t begin_canon_shoot = 0; //0: Pas de s�quence de d�but de tir initialis� sinon temps du d�but de la s�quence
uint32_t end_canon_shoot = 0; //0: Pas de s�quence de fin de tir initialis� sinon temps du d�but de la s�quence
uint32_t shooting = 0; //0: Pas de s�quence de tir initialis� sinon temps du d�but de la s�quence
float shoot_rate = 0; //Cadance de tir
float shoot_rate2 = 0; //Cadance de tir pour le feeder #2 si il est present (ex. HEROS)

//Demande de tirer
void canon_shoot_start(float speed, float rate){
	
	double coefficientVitesse = 1;
	double coefficientCadence = 1;
	/*
	if((refereeSystem.shoot_data.bullet_speed / refereeSystem.game_robot_status.shooter_id2_17mm_speed_limit) > 0.90){
		coefficientVitesse /= 1.2;
	} else if ((refereeSystem.shoot_data.bullet_speed / refereeSystem.game_robot_status.shooter_id2_17mm_speed_limit) < 0.70){
		coefficientVitesse *= 1.2;
	}
	speed *= coefficientVitesse;
	
	if((refereeSystem.power_heat_data.shooter_id2_17mm_cooling_heat / refereeSystem.game_robot_status.shooter_id2_17mm_cooling_limit) > 0.90){
		coefficientCadence = 0.5;
	} else if ((refereeSystem.power_heat_data.shooter_id2_17mm_cooling_heat / refereeSystem.game_robot_status.shooter_id2_17mm_cooling_limit) < 0.70){
		coefficientCadence = 1;
	}
	
	rate *= coefficientCadence;
	*/
	
	// Si une des valeurs est null, on demande l'arret des tirs
	if(speed == 0 || rate == 0){ 
		canon_shoot_end();
	}else{ 
		//Si nous ne sommes pas entrain de tirer, nous d�butons la s�quence de tir
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting == 0){
			begin_canon_shoot = HAL_GetTick();
			
			PWM_SetAllDuty(&htim1, speed, speed); //Demarage des snails
			shoot_rate = motors[FEEDER].direction * rate; //Sauvegarde de la cadance de tir
			
	        motors[FEEDER].setpoint = shoot_rate; //Demarage du feeder
			if (motors[FEEDER2].type == M2006) {
				shoot_rate2 = motors[FEEDER2].direction * rate;
			  motors[FEEDER2].setpoint = shoot_rate2; //Demarage du feeder #2 si il est present
		  }
		}
		
		//Prise en compte des nouvelles valeurs que si nous sommes entrain de tirer
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting != 0){ 
			PWM_SetAllDuty(&htim1, speed, speed); //Changement de la vitesse 
			shoot_rate = motors[FEEDER].direction * rate; //Changement de la cadance de tir
			motors[FEEDER].setpoint = shoot_rate; //Changement de la cadance de tir
			if (motors[FEEDER2].type == M2006) {
				shoot_rate2 = motors[FEEDER2].direction * rate;
			    motors[FEEDER2].setpoint = shoot_rate2; //Demarage du feeder #2 si il est present
		  }
		}
	}
}



//Fonction de traitement des tirs, a appeler � chaque boucle
void canon_process_inputs(){
	//Si on a demand� de commencer de tirer il y a plus de x ms, on d�mare maintenant le feeder
	if(begin_canon_shoot != 0 && HAL_GetTick() - begin_canon_shoot > TIME_BEFORE_START_FEEDER){
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = HAL_GetTick();
		
		motors[FEEDER].setpoint = shoot_rate; //Demarage du feeder
		if (motors[FEEDER2].type == M2006) {
			  motors[FEEDER2].setpoint = shoot_rate2; //Demarage du feeder #2 si il est present
		}
	}
	
	//Si on a demand� d'arreter de tirer il y a plus de x ms, on arrete tout
	if(end_canon_shoot != 0 && HAL_GetTick() - end_canon_shoot > TIME_BEFORE_STOP){ 
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = 0;
		
		PWM_SetAllDuty(&htim1, 0, 0); //Arret des snails
		motors[FEEDER].setpoint = 0; //Arret du feeder
		if (motors[FEEDER2].type == M2006) {
			motors[FEEDER2].setpoint = 0; //Arret du feeder #2 si il est present
		}
	}
}


//Demande l'arret des tirs
void canon_shoot_end(){
	begin_canon_shoot = 0;
	if(shooting != 0){
		// Si nous �tions entrain de tirer
		motors[FEEDER].setpoint = -7000; //On enleve les balles
		if (motors[FEEDER2].type == M2006) {
		  motors[FEEDER2].setpoint = -7000; //On enleve aussi les balles du 2e feeder si il est present
		}
		shooting = 0;
	}
	if(end_canon_shoot == 0){ 
		//Si nous n'avons pas encore commencer la s�quence de fin de tir
		end_canon_shoot = HAL_GetTick(); //D�but de la s�quence de fin de tir
	}
}
