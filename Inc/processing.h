/****************
   Description : Fait la laison entre la t�l�commande et les consignes moteurs
   Auteur : S�bastien FAGUET
*****************/

#ifndef PROCESSING
#define PROCESSING

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

/* Describes current aiming mode */
enum mode_assistance_ai_t{
	automatic, manual
};

/* Switches ai aim assist mode */
void switch_assistance_ai(void);

// Calculates PID control for all motors (calculate commands as functions of setpoints)
void pid_compute_command(void);

// Function that links inputs (sensors, radio controller, CV, etc.) and outputs (motor setpoints)
void process_general_inputs(void);

void chassis_setpoint(double Vx, double Vy, double W);

/* Automatically follows target */
void auto_follow_target(void);

/* Convert milirad to degrees */
float mrad_to_deg(float angle_in_millirad);

/* Returns true if controller is in neutral state */
bool is_controller_neutral(void);



#endif
