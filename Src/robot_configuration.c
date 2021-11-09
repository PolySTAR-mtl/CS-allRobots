/****************
   Description : Configuration du robot
   Auteur : S�bastien FAGUET
*****************/

#include "robot_configuration.h"

/* Cr�e le tableau contenant tous les moteurs du robot */
motor_t motors[MAX_MOTORS];

// Variable qui sauvegarde le type de robot
int robot_type;

/* CAN MOTORS DATASHEET
	M3508: Control by current
		TX :
			ID 1-4: tx_frame = 0x200 \ tx_id = ID
			ID 5-8: tx_frame = 0x1FF \ tx_id = (ID-4)
		RX :
			ID 1-8: 0x200 + ID

	M2006: Control by current
		TX :
			ID 1-4: tx_frame = 0x200 \ tx_id = ID
			ID 5-8: tx_frame = 0x1FF \ tx_id = (ID-4)
		RX :
			ID 1-8: 0x200 + ID

	GM6020: Control by voltage
		TX :
			ID 1-4: tx_frame = 0x1FF \ tx_id = ID
			ID 5-7: tx_frame = 0x2FF \ tx_id = (ID-4)
		RX :
			ID 1-7: 0x204 + ID
*/

float vitesse_snail = 0.0f;
float cadence_coeff = 0.0f;
bool inversion_gauchedroite = false;
bool inversion_avantarriere = false;


/* Fonction qui premet de configurer le robot */
void robotInit(uint8_t robot_id){
	/*
	Robot ID:
		1: Red Hero;
		2: Red Engineer;
		3/4/5: Red Standard;
		6: Red Aerial;
		7: Red Sentry;
		8: Red Dart Robot;
		9: Red Radar Station;
		101: Blue Hero;
		102: Blue Engineer;
		103/104/105: Blue Standard;
		106: Blue Aerial;
		107: Blue Sentry
		108: Blue Dart Robot;
		109: Blue Radar Station.
	*/
	if(robot_id > 100) robot_id -= 100; 
	float pid_chassis_p = 5.0;
	float pid_chassis_i = 5.0;
	float pid_chassis_d = 0.0;
	
	switch(robot_id){
		/* Standard */
		case 3:
		case 4: //Robot Meca STD
			vitesse_snail = 0.33;
			cadence_coeff = 1;
		  inversion_gauchedroite = false;
		  inversion_avantarriere = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_PITCH].debug_name, "TOURELLE PITCH");
			motors[TOURELLE_PITCH].type = GM6020;
			motors[TOURELLE_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TOURELLE_PITCH].can_tx_frame = 0x1FF; 
			motors[TOURELLE_PITCH].can_tx_id = 1;
			motors[TOURELLE_PITCH].MIN_POSITION = 65; //en deg
			motors[TOURELLE_PITCH].MAX_POSITION = 85; //en deg
			motors[TOURELLE_PITCH].setpoint = 54; //en deg //Valeur initiale
			motors[TOURELLE_PITCH].direction = -1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_PITCH].pid, 
							&motors[TOURELLE_PITCH].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_PITCH].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_PITCH].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							200, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_PITCH].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_PITCH].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_YAW].debug_name, "TOURELLE YAW");
			motors[TOURELLE_YAW].type = GM6020;
			motors[TOURELLE_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TOURELLE_YAW].can_tx_frame = 0x1FF; 
			motors[TOURELLE_YAW].can_tx_id = 2;
			motors[TOURELLE_YAW].MIN_POSITION = 15; //en deg
			motors[TOURELLE_YAW].MAX_POSITION = 160; //en deg
			motors[TOURELLE_YAW].setpoint = 325; //en deg //Valeur initiale
			motors[TOURELLE_YAW].direction = -1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_YAW].pid, 
							&motors[TOURELLE_YAW].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_YAW].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_YAW].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							200, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_YAW].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_YAW].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4; 
			motors[FEEDER].direction = 1;	
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FEEDER].command, 		//output: la commande que l'on envoie au moteur
							&motors[FEEDER].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							0.5, 0.5, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); //Minimum et maximum de la commande envoyable au moteur

			break;
		case 5: //ROBOT DJI
			vitesse_snail = 0.30;
			cadence_coeff = 1;
		  inversion_gauchedroite = false;
		  inversion_avantarriere = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_PITCH].debug_name, "TOURELLE PITCH");
			motors[TOURELLE_PITCH].type = GM6020;
			motors[TOURELLE_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TOURELLE_PITCH].can_tx_frame = 0x1FF; 
			motors[TOURELLE_PITCH].can_tx_id = 1;
			motors[TOURELLE_PITCH].MIN_POSITION = 240; //en deg
			motors[TOURELLE_PITCH].MAX_POSITION = 293; //en deg
			motors[TOURELLE_PITCH].setpoint = 250; //en deg //Valeur initiale
			motors[TOURELLE_PITCH].direction = 1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_PITCH].pid, 
							&motors[TOURELLE_PITCH].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_PITCH].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_PITCH].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							400, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_PITCH].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_PITCH].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_YAW].debug_name, "TOURELLE YAW");
			motors[TOURELLE_YAW].type = GM6020;
			motors[TOURELLE_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TOURELLE_YAW].can_tx_frame = 0x1FF; 
			motors[TOURELLE_YAW].can_tx_id = 2;
			motors[TOURELLE_YAW].MIN_POSITION = 110.71; //en deg
			motors[TOURELLE_YAW].MAX_POSITION = 308.44; //en deg
			motors[TOURELLE_YAW].setpoint = 208; //en deg //Valeur initiale
			motors[TOURELLE_YAW].direction = -1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_YAW].pid, 
							&motors[TOURELLE_YAW].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_YAW].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_YAW].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							200, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_YAW].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_YAW].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4;
			motors[FEEDER].direction = 1;			
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FEEDER].command, 		//output: la commande que l'on envoie au moteur
							&motors[FEEDER].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							0.5, 0.5, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); //Minimum et maximum de la commande envoyable au moteur

			break;
		case 6: //HÉRO
			vitesse_snail = 0.80;
			cadence_coeff = 2;
		  inversion_gauchedroite = false;
		  inversion_avantarriere = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FRONT_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[FRONT_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_RIGHT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_RIGHT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[BACK_LEFT].command, 		//output: la commande que l'on envoie au moteur
							&motors[BACK_LEFT].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							pid_chassis_p, pid_chassis_i, pid_chassis_d); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_PITCH].debug_name, "TOURELLE PITCH");
			motors[TOURELLE_PITCH].type = GM6020;
			motors[TOURELLE_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TOURELLE_PITCH].can_tx_frame = 0x1FF; 
			motors[TOURELLE_PITCH].can_tx_id = 1;
			motors[TOURELLE_PITCH].MIN_POSITION = 0; //en deg
			motors[TOURELLE_PITCH].MAX_POSITION = 359; //en deg
			motors[TOURELLE_PITCH].setpoint = 250; //en deg //Valeur initiale
			motors[TOURELLE_PITCH].direction = 1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_PITCH].pid, 
							&motors[TOURELLE_PITCH].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_PITCH].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_PITCH].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							400, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_PITCH].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_PITCH].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[TOURELLE_YAW].debug_name, "TOURELLE YAW");
			motors[TOURELLE_YAW].type = GM6020;
			motors[TOURELLE_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TOURELLE_YAW].can_tx_frame = 0x1FF; 
			motors[TOURELLE_YAW].can_tx_id = 2;
			motors[TOURELLE_YAW].MIN_POSITION = 0; //en deg
			motors[TOURELLE_YAW].MAX_POSITION = 359; //en deg
			motors[TOURELLE_YAW].setpoint = 208; //en deg //Valeur initiale
			motors[TOURELLE_YAW].direction = -1; //permet de choisir la direction de controle (-1 ou 1)
			pid_create(&motors[TOURELLE_YAW].pid, 
							&motors[TOURELLE_YAW].info.angle_360, 		//input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[TOURELLE_YAW].command, 						//output: la commande que l'on envoie au moteur
							&motors[TOURELLE_YAW].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							200, 100, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circulaire(&motors[TOURELLE_YAW].pid, 360); //Asservissement circulaire, permet, comme on fait une r�gulation en position, quand on est a position = 350 degr�e, que la setpoint est � 10deg, de ne pas faire tout le tour
			pid_limits(&motors[TOURELLE_YAW].pid, -30000, 30000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4; 
			motors[FEEDER].direction = -1;
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FEEDER].command, 		//output: la commande que l'on envoie au moteur
							&motors[FEEDER].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							0.5, 0.5, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); //Minimum et maximum de la commande envoyable au moteur
			
			strcpy(motors[FEEDER2].debug_name, "FEEDER2");
			motors[FEEDER2].type = M2006;
			motors[FEEDER2].can_rx_id = 0x200+8; // ID = 8
			motors[FEEDER2].can_tx_frame = 0x1FF; 
			motors[FEEDER2].can_tx_id = 7-3; 
			motors[FEEDER2].direction = -1;
			pid_create(&motors[FEEDER2].pid, 
							&motors[FEEDER2].info.speed, //input : le retour sur la quelle ont veut atteintre la setpoint 
							&motors[FEEDER2].command, 		//output: la commande que l'on envoie au moteur
							&motors[FEEDER2].setpoint, 	//setpoint: On veut que le moteur soit � cette position ou tourne a cette vitesse
							0.5, 0.5, 0); //k, i, d : les coefficient de r�gulation : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER2].pid, -10000, 10000); //Minimum et maximum de la commande envoyable au moteur

			break;
			
	}
}
