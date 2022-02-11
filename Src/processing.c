/****************
   Description : Fait la laison entre la t�l�commande et les consignes moteurs
   Auteur : S�bastien FAGUET
*****************/


#include "processing.h"
#include "pilots.h"
#include "canon.h"
#include "robot_configuration.h"


#define MAX_BASE_SPEED_COEFF  10
#define PI 3.14159265358979323846

// Recover external variables
extern receiver_RadioController_t receiver_RadioController;	
extern motor_t motors[MAX_MOTORS];
extern pilot_t pilot;
extern jetson_t jetson;
extern refereeSystem_t refereeSystem;

extern float snail_vel; 
extern float cadence_mult;
extern bool  invert_leftright;
extern bool  invert_frontback;
extern int robot_type;

/* Current control mode */
enum mode_assistance_ai_t mode_assistance_ai = manual;
enum mode_assistance_ai_t previous_mode_ai = automatic;

// Calculates PID control for all motors (calculate commands as functions of setpoints)
void pid_compute_command(){
	for(int i = 0; i < MAX_MOTORS ; i++){
		if (pid_need_compute(&motors[i].pid)) {
			pid_compute(&motors[i].pid);
			//if(i == FEEDER) pid_debug_uart(&motors[i].pid);
		}
	}
}

/* Returns true if controller is in neutral state */
bool is_controller_neutral(){
	
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


// Function that links inputs (sensors, radio controller, CV, etc.) and outputs (motor setpoints)
void process_general_inputs(){
	
	if (!receiver_RadioController.keyboard_mode) {
		//if(mode_assistance_ai==automatic) auto_follow_target();
		if(robot_type == 7){
			add_setpoint_position(&motors[TURRET_PITCH], -receiver_RadioController.data.ch2_float, pilot.sensitivity_ch_2);
			add_setpoint_position(&motors[TURRET_YAW], 	-receiver_RadioController.data.ch1_float, pilot.sensitivity_ch_1);
		}else{
			add_setpoint_position(&motors[TURRET_PITCH], receiver_RadioController.data.ch2_float, pilot.sensitivity_ch_2);
			add_setpoint_position(&motors[TURRET_YAW], 	receiver_RadioController.data.ch1_float, pilot.sensitivity_ch_1);
		}
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
				canon_shoot_start(0, 0);
				break;
			case 3:
				canon_shoot_start(snail_vel/2, 1000);
				break;
			case 1:
				canon_shoot_start(snail_vel, 1000);
				break;
		}
		
		// On a inversé par erreur les channel x et y
		// TODO: Corriger ceci (pas encore corrigé puisque beaucoup de code dépend de cette fonction)
		if (robot_type ==7){
			sentry_setpoint(receiver_RadioController.data.ch4); 
		}
		else{
			chassis_setpoint(receiver_RadioController.data.ch4, receiver_RadioController.data.ch3, receiver_RadioController.data.wheel); 
		}
			
	} else {
		double chassis_w;
		double turret_yaw;
		if(!receiver_RadioController.data.kb.bit.CTRL){
			chassis_w = -receiver_RadioController.data.mouse.x;
			turret_yaw = 0;
		}else{
			chassis_w = 0;
			turret_yaw = receiver_RadioController.data.mouse.x;
		}
		
		// Manages aim assist
		//if(receiver_RadioController.data.kb.bit.Q) switch_assistance_ai();
		//if(mode_assistance_ai==automatique) auto_follow_target();
		
		add_setpoint_position(&motors[TURRET_PITCH], receiver_RadioController.data.mouse.y, pilot.sensitivity_mouse_y);
		add_setpoint_position(&motors[TURRET_YAW], turret_yaw, pilot.sensitivity_mouse_x);
		
		chassis_setpoint(receiver_RadioController.data.kb.bit.W - receiver_RadioController.data.kb.bit.S, 
											receiver_RadioController.data.kb.bit.D - receiver_RadioController.data.kb.bit.A, 
											chassis_w);
		
		
		if(receiver_RadioController.data.mouse.l){
			canon_shoot_start(snail_vel/2, cadence_mult * 1000);
		}else if(receiver_RadioController.data.mouse.r){
			canon_shoot_start(snail_vel, cadence_mult * 1000);
		}else{
			canon_shoot_end();
		}
	}
}

void chassis_setpoint(double Vx, double Vy, double W){ 
	/*
		Vx: Forwards / Backwards
		Vy:	Left / Right translation
		W: Rotation
	*/
	double sensitivity_Vx;
	double sensitivity_Vy;
	double sensitivity_W;
	double sensitivity_deadzone;
	
	double coefficient_ShiftChassis;
	double coefficient_EChassis;
	double coefficient_Power = 1;
	/*
	if(refereeSystem.power_heat_data.chassis_power / refereeSystem.game_robot_status.chassis_power_limit > 0.90){
		if(coefficient_Power == 1) {
			coefficient_Power = 0.8;
		} else if (coefficient_Power < 1) {
			coefficient_Power /= 2;
		}
	}
	else {
		coefficient_Power = 1;
	}*/
	if (invert_leftright) {
		// channels 3 et 4 inversés... le 3 c'est en x et le 4 c'est en y normalement (selon la datasheet)
		// mais il y a une erreur dans process_general_inputs(), donc on inverse le gauche-droite en inversant le Y et non le X
		Vy = -Vy;
	}
	
	if (invert_frontback) {
		// channels 3 et 4 inversés... le 3 c'est en x et le 4 c'est en y normalement (selon la datasheet)
		// mais il y a une erreur dans process_general_inputs, donc on inverse le avant-arrière en inversant le X et non le Y
		Vx = -Vx;
	}
	
	if(receiver_RadioController.keyboard_mode){
		sensitivity_Vx = pilot.sensitivity_chassis_keyboard_Vx;
		sensitivity_Vy = pilot.sensitivity_chassis_keyboard_Vy;
		sensitivity_W = pilot.sensitivity_chassis_mouse_W;
		sensitivity_deadzone = 0;
		
		coefficient_ShiftChassis = pilot.coefficient_ShiftChassis;
		coefficient_EChassis = pilot.coefficient_EChassis;
	}else{
		sensitivity_Vx = pilot.sensitivity_chassis_RC_Vx;
		sensitivity_Vy = pilot.sensitivity_chassis_RC_Vy;
		sensitivity_W = pilot.sensitivity_chassis_RC_W;
		sensitivity_deadzone = pilot.sensitivity_RC_deadzone;
	}
	if(Vx > -sensitivity_deadzone && Vx < sensitivity_deadzone) Vx = 0;
	if(Vy > -sensitivity_deadzone && Vy < sensitivity_deadzone) Vy = 0;
	
	// Apply coefficients on chassis velocity if SHIFT or E are pressed
	if(receiver_RadioController.data.kb.bit.SHIFT){
		motors[FRONT_LEFT].setpoint 	= (coefficient_ShiftChassis * (sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W)) * coefficient_Power;
		motors[FRONT_RIGHT].setpoint 	= (coefficient_ShiftChassis * (-(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W))) * coefficient_Power;
		motors[BACK_RIGHT].setpoint 	= (coefficient_ShiftChassis * (-(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W))) * coefficient_Power;
		motors[BACK_LEFT].setpoint 		= (coefficient_ShiftChassis * (sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W)) * coefficient_Power;
	} else if(receiver_RadioController.data.kb.bit.E){ 
		motors[FRONT_LEFT].setpoint 	= (coefficient_EChassis * (sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W)) * coefficient_Power;
		motors[FRONT_RIGHT].setpoint 	= (coefficient_EChassis * (-(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W))) * coefficient_Power;
		motors[BACK_RIGHT].setpoint 	= (coefficient_EChassis * (-(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W))) * coefficient_Power;
		motors[BACK_LEFT].setpoint 		= (coefficient_EChassis * (sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W)) * coefficient_Power;
	}else{
		motors[FRONT_LEFT].setpoint 	= (sensitivity_Vx*Vx - sensitivity_Vy*Vy - sensitivity_W*W) * coefficient_Power;
		motors[FRONT_RIGHT].setpoint 	= (-(sensitivity_Vx*Vx + sensitivity_Vy*Vy + sensitivity_W*W)) * coefficient_Power;
		motors[BACK_RIGHT].setpoint 	= (-(sensitivity_Vx*Vx - sensitivity_Vy*Vy + sensitivity_W*W)) * coefficient_Power;
		motors[BACK_LEFT].setpoint 		= (sensitivity_Vx*Vx + sensitivity_Vy*Vy - sensitivity_W*W) * coefficient_Power; 
	}
	
}	

void sentry_setpoint(double Vx){ 
	/*
		Vx: left / right translation
	*/
	double sensitivity_Vx;
	double sensitivity_deadzone;
	
	double coefficient_Power = 1;
	/*
	if(refereeSystem.power_heat_data.chassis_power / refereeSystem.game_robot_status.chassis_power_limit > 0.90){
		if(coefficient_Power == 1) {
			coefficient_Power = 0.8;
		} else if (coefficient_Power < 1) {
			coefficient_Power /= 2;
		}
	}
	else {
		coefficient_Power = 1;
	}*/
	
	if (invert_leftright) {
		// channels 3 et 4 inversés... le 3 c'est en x et le 4 c'est en y normalement (selon la datasheet)
		// mais il y a une erreur dans process_general_inputs, donc on inverse le avant-arrière en inversant le X et non le Y
		Vx = -Vx;
	}
	
	
	sensitivity_Vx = pilot.sensitivity_chassis_RC_Vx;
	sensitivity_deadzone = pilot.sensitivity_RC_deadzone;

	if(Vx > -sensitivity_deadzone && Vx < sensitivity_deadzone) Vx = 0;
	
	// Apply coefficients on chassis velocity if SHIFT or E are pressed
	
	motors[FRONT_LEFT].setpoint 	= (sensitivity_Vx*Vx) * coefficient_Power;
	motors[BACK_RIGHT].setpoint 	= (-(sensitivity_Vx*Vx) * coefficient_Power);
	
}	

// Change targeting mode
/* TODO : On pourrait ajouter un coefficient qui est 1 en manuel et <1 en automatique qui s'applique
   sur les setpoint donnees par le pilot pour la tourelle */
void switch_assistance_ai(void){
		if (mode_assistance_ai != previous_mode_ai) {
			switch(mode_assistance_ai){
				case automatic : 
					mode_assistance_ai = manual; 
					pid_create(&motors[TURRET_YAW].pid, 
								&motors[TURRET_YAW].info.angle_360,		// PID input : feedback value on which we want to reach setpoint
								&motors[TURRET_YAW].command, 			// PID output : command sent to motor
								&motors[TURRET_YAW].setpoint, 			// Setpoint : speed or position to be reached by motor
								200, 100, 0);                           // Kp, Ki, Kd : Regulation coefficients
					pid_create(&motors[TURRET_PITCH].pid, 
								&motors[TURRET_PITCH].info.angle_360,	// PID input : feedback value on which we want to reach setpoint 
								&motors[TURRET_PITCH].command, 			// PID output : command sent to motor
								&motors[TURRET_PITCH].setpoint, 		// Setpoint : speed or position to be reached by motor
								200, 100, 0);                           // Kp, Ki, Kd : Regulation coefficients
					previous_mode_ai = automatic;
					break;
				case manual : 
					mode_assistance_ai = automatic; 
					pid_create(&motors[TURRET_YAW].pid, 
								&motors[TURRET_YAW].info.angle_360, 		
								&motors[TURRET_YAW].command, 						
								&motors[TURRET_YAW].setpoint, 					
								200, 400, 0);
					pid_create(&motors[TURRET_PITCH].pid, 
								&motors[TURRET_PITCH].info.angle_360, 		
								&motors[TURRET_PITCH].command, 						
								&motors[TURRET_PITCH].setpoint, 					
								400, 400, 0);
					previous_mode_ai = manual;
					break;
			}
		}
}

// Function that automatically controls turret's GM6020 motors to aim at a target
// TODO : Tester la vitesse de suivi des cibles de la tourelle et changer le coefficient de ralentissement si besoin
void auto_follow_target(void){
  
	static uint32_t tickstart = 0;
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}
		if ((HAL_GetTick() - tickstart) < 100){
		return;
	}
  
	uint16_t theta,d;
	int16_t phi;
	uint8_t target_located;
	
	jetson.switch_information.switch_target_mode = 0x72; // TODO : Le target mode doit etre stocke autre part (car la trame switch_information on l'envoie on ne la recoit pas)
	switch(jetson.switch_information.switch_target_mode){
		case 0x72 : // If target is a robot
			theta = jetson.robot_target_coordinates.theta_target_location;
			phi = jetson.robot_target_coordinates.phi_target_location;
			d = jetson.robot_target_coordinates.d_target_location;
			target_located = jetson.robot_target_coordinates.target_located;
			break;
		case 0x52 : // If target is a rune
			theta = jetson.rune_target_coordinates.theta_target_location;
			phi = jetson.rune_target_coordinates.phi_target_location;
			d = jetson.rune_target_coordinates.d_target_location;
	  	target_located = jetson.rune_target_coordinates.target_located;
			break;
	}
	//uart_debug();
	
	// to delete soon (keep lines 181-194) :
			theta = jetson.robot_target_coordinates.theta_target_location;
			phi = jetson.robot_target_coordinates.phi_target_location;
			d = jetson.robot_target_coordinates.d_target_location;
			target_located = jetson.robot_target_coordinates.target_located;
	//
	
	if(target_located=='Y'){ // Changes setpoints only if a target is located
		float setpoint_yaw;
		float setpoint_pitch;		
		
		// phi = 0 => No movement
		// phi > 0 => Yaw left
		// phi < 0 => Yaw right
		setpoint_yaw = motors[TURRET_YAW].setpoint - mrad_to_deg(phi) * 0.0001; //* coeff_ralentissement_yaw;
		
		// angle_pitch = 0 => No movement
		// angle_pitch > 0 => Pitch up
		// angle_pitch < 0 => Pitch down
		float angle_pitch = (PI/2)*1000 - theta; // Difference between current angle and target location (in millirads)
		setpoint_pitch = motors[TURRET_PITCH].setpoint - mrad_to_deg(angle_pitch) *0.0001; //* coeff_ralentissement_pitch;
		
		
		// TODO : Peut �tre utiliser add_setpoint_position() a la place des lignes 246 � 272
		// TODO : S'assure que la setpoint ne est entre 0 et 360 
		
		if(setpoint_yaw > 360) setpoint_yaw -= (float) 360.0;
		if(setpoint_yaw < 0) 	setpoint_yaw += (float) 360.0;
	
		if(setpoint_pitch > 360) setpoint_pitch -= (float) 360.0;
		if(setpoint_pitch < 0) 	setpoint_pitch += (float) 360.0;
	
		// Makes sure setpoint respects upper and lower limits
		if(motors[TURRET_YAW].MAX_POSITION > 0 && setpoint_yaw > motors[TURRET_YAW].MAX_POSITION) {
			setpoint_yaw = motors[TURRET_YAW].MAX_POSITION;
		}
		if(motors[TURRET_YAW].MIN_POSITION > 0 && setpoint_yaw < motors[TURRET_YAW].MIN_POSITION){
			setpoint_yaw = motors[TURRET_YAW].MIN_POSITION;
		}
		if(motors[TURRET_PITCH].MAX_POSITION > 0 && setpoint_pitch > motors[TURRET_PITCH].MAX_POSITION){
			setpoint_pitch = motors[TURRET_PITCH].MAX_POSITION;
		}
		if(motors[TURRET_PITCH].MIN_POSITION > 0 && setpoint_pitch < motors[TURRET_PITCH].MIN_POSITION){
			setpoint_pitch = motors[TURRET_PITCH].MIN_POSITION;
		}
		
		/*
		TODO : ajustement du "setpoint_pitch" en fonction de la distance "d" de la cible
		*/
		
		motors[TURRET_YAW].setpoint = setpoint_yaw;
		
		motors[TURRET_PITCH].setpoint = setpoint_pitch;
	 
	}
	else {
		motors[TURRET_YAW].setpoint = motors[TURRET_YAW].info.angle_360;
		motors[TURRET_YAW].command = motors[TURRET_YAW].info.angle_360;
		motors[TURRET_PITCH].setpoint = motors[TURRET_PITCH].info.angle_360;
		motors[TURRET_PITCH].command = motors[TURRET_PITCH].info.angle_360;
	}
}

// Converts millirads into degrees
float mrad_to_deg(float angle_in_millirad){
	// 1 millirad = (180/PI)*0.001 degrees
	return angle_in_millirad * (180/PI) * 0.001;
}
