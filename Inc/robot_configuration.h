/****************
   Description : Configuration du robot
   Auteur : Sébastien FAGUET
*****************/

#ifndef ROBOT_CONFIGURATION
#define ROBOT_CONFIGURATION

#include "motors.h"
#include <string.h>
#include "leds.h"

#define MAX_MOTORS 20

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define TOURELLE_YAW 4
#define TOURELLE_PITCH 5
#define FEEDER 6



/* Fonction qui premet de configurer le robot */
void robotInit(uint8_t robot_id);

#endif
