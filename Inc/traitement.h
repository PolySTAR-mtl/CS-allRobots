/****************
   Description : Fait la laison entre la télécommande et les consignes moteurs
   Auteur : Sébastien FAGUET
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

/* Calcul les pids de tous les moteurs (calcul des commandes en fonction des consignes */
void traitement_pids_compute(void);

/* Fonctions qui fait les liens entre les entrées (capteurs, radio controller, CV, ...) et les sorties (consignes moteurs), on peut créer plusieurs traitements */
void traitement_1(void);

#endif
