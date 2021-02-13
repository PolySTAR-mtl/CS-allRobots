/****************
   Description : Gestion du cannon
   Auteur : Sébastien FAGUET
*****************/

                 
#include "canon.h"

#define TIME_BEFORE_START_FEEDER 10
#define TIME_BEFORE_STOP 100

extern motor_t motors[MAX_MOTORS];
uint32_t begin_canon_shoot = 0; //0: Pas de séquence de début de tir initialisé sinon temps du début de la séquence
uint32_t end_canon_shoot = 0; //0: Pas de séquence de fin de tir initialisé sinon temps du début de la séquence
uint32_t shooting = 0; //0: Pas de séquence de tir initialisé sinon temps du début de la séquence
float shoot_rate = 0; //Cadance de tir

//Demande de tirer
void canon_shoot(float speed, float rate){
	// Si une des valeurs est null, on demande l'arret des tirs
	if(speed == 0 || rate == 0){ 
		canon_shoot_end();
	}else{ 
		//Si nous ne sommes pas entrain de tirer, nous débutons la séquence de tir
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting == 0){
			begin_canon_shoot = HAL_GetTick();
			
			PWM_SetAllDuty(&htim1, speed, speed); //Démarage des snails
			shoot_rate = rate; //Sauvegarde de la cadance de tir
		}
		
		//Prise en compte des nouvelles valeurs que si nous sommes entrain de tirer
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting != 0){ 
			PWM_SetAllDuty(&htim1, speed, speed); //Changement de la vitesse 
			motors[FEEDER].consigne = rate; //Changement de la cadance de tir
			shoot_rate = rate; //Changement de la cadance de tir
		}
	}
}



//Fonction de traitement des tirs, a appeler à chaque boucle
void traitement_shoot(){
	//Si on a demandé de commencer de tirer il y a plus de x ms, on démare maintenant le feeder
	if(begin_canon_shoot != 0 && HAL_GetTick() - begin_canon_shoot > TIME_BEFORE_START_FEEDER){
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = HAL_GetTick();
		
		motors[FEEDER].consigne = shoot_rate; //Démarage du feeder
	}
	
	//Si on a demandé d'arreter de tirer il y a plus de x ms, on arrete tout
	if(end_canon_shoot != 0 && HAL_GetTick() - end_canon_shoot > TIME_BEFORE_STOP){ 
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = 0;
		
		PWM_SetAllDuty(&htim1, 0, 0); //Arret des snails
		motors[FEEDER].consigne = 0; //Arret du feeder
	}
}


//Demande l'arret des tirs
void canon_shoot_end(){
	begin_canon_shoot = 0;
	if(shooting != 0){
		// Si nous étions entrain de tirer
		motors[FEEDER].consigne = -7000; //On enlève les balles
		shooting = 0;
	}
	if(end_canon_shoot == 0){ 
		//Si nous n'avons pas encore commencer la séquence de fin de tir
		end_canon_shoot = HAL_GetTick(); //Début de la séquence de fin de tir
	}
}
