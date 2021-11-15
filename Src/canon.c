/****************
   Description : Gestion du cannon
   Auteur : S�bastien FAGUET
*****************/

                 
#include "canon.h"

#define FEEDER_START_DELAY 10
#define SHUTDOWN_DELAY 100

extern motor_t motors[MAX_MOTORS];
extern refereeSystem_t refereeSystem;
uint32_t begin_canon_shoot = 0; // If 0, no firing start sequence initiated. Otherwise, timestamp of sequence start
uint32_t end_canon_shoot = 0; 	// If 0, no firing end sequence initiated. Otherwise, timestamp of sequence start
uint32_t shooting = 0; 			// If 0, not currently firing. Otherwise, timestamp of sequence start
float shoot_rate = 0;  // Fire rate
float shoot_rate2 = 0; // Fire rate for feeder #2, if it exists (ex. HEROS)

// Initiate firing start sequence
void canon_shoot_start(float speed, float rate){
	
	double speed_multiplier = 1;
	double rate_multiplier = 1;
	/*
	if((refereeSystem.shoot_data.bullet_speed / refereeSystem.game_robot_status.shooter_id2_17mm_speed_limit) > 0.90){
		speed_multiplier /= 1.2;
	} else if ((refereeSystem.shoot_data.bullet_speed / refereeSystem.game_robot_status.shooter_id2_17mm_speed_limit) < 0.70){
		speed_multiplier *= 1.2;
	}
	speed *= speed_multiplier;
	
	if((refereeSystem.power_heat_data.shooter_id2_17mm_cooling_heat / refereeSystem.game_robot_status.shooter_id2_17mm_cooling_limit) > 0.90){
		rate_multiplier = 0.5;
	} else if ((refereeSystem.power_heat_data.shooter_id2_17mm_cooling_heat / refereeSystem.game_robot_status.shooter_id2_17mm_cooling_limit) < 0.70){
		rate_multiplier = 1;
	}
	
	rate *= rate_multiplier;
	*/
	
	// If either input is zero, initiate firing end sequence
	if(speed == 0 || rate == 0){ 
		canon_shoot_end();
	}else{ 
		// If not currently firing or stopping, begin firing sequence
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting == 0){
			begin_canon_shoot = HAL_GetTick();
			
			PWM_SetAllDuty(&htim1, speed, speed); // Start up snails
			shoot_rate = motors[FEEDER].direction * rate; // Store firing rate
			
	        motors[FEEDER].setpoint = shoot_rate; // Start up feeder
			if (motors[FEEDER2].type == M2006) {
				shoot_rate2 = motors[FEEDER2].direction * rate;
			  motors[FEEDER2].setpoint = shoot_rate2; // Start up feeder #2 if present
		  }
		}
		
		// If currently firing, update values
		if(begin_canon_shoot == 0 && end_canon_shoot == 0 && shooting != 0){ 
			PWM_SetAllDuty(&htim1, speed, speed); // Update speed
			shoot_rate = motors[FEEDER].direction * rate; // Update local firing rate variable
			motors[FEEDER].setpoint = shoot_rate; // Update firing rate in feeder motor struct
			if (motors[FEEDER2].type == M2006) {
				shoot_rate2 = motors[FEEDER2].direction * rate;
			    motors[FEEDER2].setpoint = shoot_rate2; // Update firing rate in feeder #2 motor struct, if present
		  }
		}
	}
}


// Firing sequence processing function, called each loop
void canon_process_inputs(){
	// If in firing start sequence, and FEEDER_START_DELAY has elapsed, start feeder
	if(begin_canon_shoot != 0 && HAL_GetTick() - begin_canon_shoot > FEEDER_START_DELAY){
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = HAL_GetTick();
		
		motors[FEEDER].setpoint = shoot_rate; // Start feeder
		if (motors[FEEDER2].type == M2006) {
			  motors[FEEDER2].setpoint = shoot_rate2; // Start feeder #2, if present
		}
	}
	
	// If in firing end sequence, and SHUTDOWN_DELAY has elapsed, stop everything 
	if(end_canon_shoot != 0 && HAL_GetTick() - end_canon_shoot > SHUTDOWN_DELAY){ 
		begin_canon_shoot = 0;
		end_canon_shoot = 0;
		shooting = 0;
		
		PWM_SetAllDuty(&htim1, 0, 0); // Stop snails
		motors[FEEDER].setpoint = 0; // Stop feeder
		if (motors[FEEDER2].type == M2006) {
			motors[FEEDER2].setpoint = 0; // Stop feeder #2, if present
		}
	}
}


// Initiate firing end sequence
void canon_shoot_end(){
	begin_canon_shoot = 0;
	if(shooting != 0){
		// If currently firing
		motors[FEEDER].setpoint = -7000; // Remove balls
		if (motors[FEEDER2].type == M2006) {
		  motors[FEEDER2].setpoint = -7000; // Remove balls from second feeder if present
		}
		shooting = 0;
	}
	if(end_canon_shoot == 0){ 
		// If firing end sequence has not been initiated
		end_canon_shoot = HAL_GetTick(); // Store time of sequence start
	}
}
