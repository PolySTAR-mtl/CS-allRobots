/****************
   Description : Fait la laison entre la t�l�commande et les consignes moteurs
   Auteur : S�bastien FAGUET
*****************/

#ifndef TRAITEMENT
#define TRAITEMENT

#include "BoardA_handle.h"
#include "oled_debug.h"
#include "oled.h"
#include "leds.h"
#include "receiver_RadioController.h"
#include "referee_system.h"
#include "jetson.h"
#include "motors.h"
#include "robot_configuration.h"
#include "pid.h"

/*decrit si on est en mode visee automatique ou manuelle*/
enum mode_assistance_ai_t{
	automatique, manuel
};

/*change le mode d'assisatance AI pour la visee*/
void switch_assistance_ai(void);

/* Calcul les pids de tous les moteurs (calcul des commandes en fonction des consignes */
void pid_compute_command(void);

/* Fonctions qui fait les liens entre les entr�es (capteurs, radio controller, CV, ...) et les sorties (consignes moteurs), on peut cr�er plusieurs traitements */
void processGeneralInputs(void);

void chassis_setpoint(double Vx, double Vy, double W);

/*Effectue le suivi automatique des cibles */
void auto_follow_target(void);

/*Convertit un angle en millirad en degres*/
float mrad_to_deg(float angle_in_millirad);

/*Retourne true si le controller est a une position neutre*/
bool isControllerNeutral(void);



#endif
