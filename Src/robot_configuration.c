/****************
   Description : Configuration du robot
   Auteur : S�bastien FAGUET
*****************/

#include "robot_configuration.h"

// Creates array containing all of the robot's motors
motor_t motors[MAX_MOTORS];

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

float snail_vel = 0.0f;
float cadence_mult = 0.0f;
bool invert_leftright = false;
bool invert_frontback = false;


// Function that configures robot
void robot_init(uint8_t robot_id){
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
		case 4: // Meca STD Robot
			snail_vel = 0.33;
			cadence_mult = 1;
		  invert_leftright = false;
		  invert_frontback = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, // PID input : feedback value on which we want to reach setpoint 
							&motors[FRONT_LEFT].command, 	// PID output : command sent to motor
							&motors[FRONT_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint 
							&motors[FRONT_RIGHT].command,	 // PID output : command sent to motor
							&motors[FRONT_RIGHT].setpoint, 	 // Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint 
							&motors[BACK_RIGHT].command, 	// PID output : command sent to motor
							&motors[BACK_RIGHT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed,  // PID input : feedback value on which we want to reach setpoint  
							&motors[BACK_LEFT].command, 	// PID output : command sent to motor
							&motors[BACK_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_PITCH].debug_name, "TURRET PITCH");
			motors[TURRET_PITCH].type = GM6020;
			motors[TURRET_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TURRET_PITCH].can_tx_frame = 0x1FF; 
			motors[TURRET_PITCH].can_tx_id = 1;
			motors[TURRET_PITCH].MIN_POSITION = 65; // Degrees
			motors[TURRET_PITCH].MAX_POSITION = 85; // Degrees
			motors[TURRET_PITCH].setpoint = 54; // Degrees //Initial value
			motors[TURRET_PITCH].direction = -1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_PITCH].pid, 
							&motors[TURRET_PITCH].info.angle_360, // PID input : feedback value on which we want to reach setpoint  
							&motors[TURRET_PITCH].command, 		  // PID output : command sent to motor
							&motors[TURRET_PITCH].setpoint, 	  // Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							200, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_PITCH].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_PITCH].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_YAW].debug_name, "TURRET YAW");
			motors[TURRET_YAW].type = GM6020;
			motors[TURRET_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TURRET_YAW].can_tx_frame = 0x1FF; 
			motors[TURRET_YAW].can_tx_id = 2;
			motors[TURRET_YAW].MIN_POSITION = 15; // Degrees
			motors[TURRET_YAW].MAX_POSITION = 160; // Degrees
			motors[TURRET_YAW].setpoint = 325; // Degrees //Initial value
			motors[TURRET_YAW].direction = -1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_YAW].pid, 
							&motors[TURRET_YAW].info.angle_360, // PID input : feedback value on which we want to reach setpoint 
							&motors[TURRET_YAW].command, 		// PID output : command sent to motor
							&motors[TURRET_YAW].setpoint, 	  	// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							200, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_YAW].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_YAW].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4; 
			motors[FEEDER].direction = 1;	
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, // PID input : feedback value on which we want to reach setpoint 
							&motors[FEEDER].command, 	// PID output : command sent to motor
							&motors[FEEDER].setpoint, 	// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							0.5, 0.5, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); // Minimum and maximum command that can be sent to motor

			break;
		case 5: // DJI ROBOT
			snail_vel = 0.30;
			cadence_mult = 1;
		  invert_leftright = false;
		  invert_frontback = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FRONT_LEFT].command, 	// PID output : command sent to motor
							&motors[FRONT_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FRONT_RIGHT].command, 	 // PID output : command sent to motor
							&motors[FRONT_RIGHT].setpoint, 	 // Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[BACK_RIGHT].command, 	// PID output : command sent to motor
							&motors[BACK_RIGHT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed,  // PID input : feedback value on which we want to reach setpoint  
							&motors[BACK_LEFT].command, 	// PID output : command sent to motor
							&motors[BACK_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_PITCH].debug_name, "TURRET PITCH");
			motors[TURRET_PITCH].type = GM6020;
			motors[TURRET_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TURRET_PITCH].can_tx_frame = 0x1FF; 
			motors[TURRET_PITCH].can_tx_id = 1;
			motors[TURRET_PITCH].MIN_POSITION = 240; // Degrees
			motors[TURRET_PITCH].MAX_POSITION = 293; // Degrees
			motors[TURRET_PITCH].setpoint = 250; // Degrees //Initial value
			motors[TURRET_PITCH].direction = 1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_PITCH].pid, 
							&motors[TURRET_PITCH].info.angle_360, 	// PID input : feedback value on which we want to reach setpoint  
							&motors[TURRET_PITCH].command, 			// PID output : command sent to motor
							&motors[TURRET_PITCH].setpoint, 		// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							400, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_PITCH].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_PITCH].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_YAW].debug_name, "TURRET YAW");
			motors[TURRET_YAW].type = GM6020;
			motors[TURRET_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TURRET_YAW].can_tx_frame = 0x1FF; 
			motors[TURRET_YAW].can_tx_id = 2;
			motors[TURRET_YAW].MIN_POSITION = 110.71; // Degrees
			motors[TURRET_YAW].MAX_POSITION = 308.44; // Degrees
			motors[TURRET_YAW].setpoint = 208; // Degrees //Initial value
			motors[TURRET_YAW].direction = -1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_YAW].pid, 
							&motors[TURRET_YAW].info.angle_360, // PID input : feedback value on which we want to reach setpoint  
							&motors[TURRET_YAW].command, 		// PID output : command sent to motor
							&motors[TURRET_YAW].setpoint, 		// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							200, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_YAW].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_YAW].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4;
			motors[FEEDER].direction = 1;			
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FEEDER].command, 	// PID output : command sent to motor
							&motors[FEEDER].setpoint, 	// Setpoint : speed or position to be reached by motor
							0.5, 0.5, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); // Minimum and maximum command that can be sent to motor

			break;

		case 6: // HERO
			snail_vel = 0.80;
			cadence_mult = 2;
		  invert_leftright = false;
		  invert_frontback = false;
		
			strcpy(motors[FRONT_LEFT].debug_name, "FRONT_LEFT");
			motors[FRONT_LEFT].type = M3508;
			motors[FRONT_LEFT].can_rx_id = 0x200+1; // ID = 1
			motors[FRONT_LEFT].can_tx_frame = 0x200; 
			motors[FRONT_LEFT].can_tx_id = 1;
			pid_create(&motors[FRONT_LEFT].pid, 
							&motors[FRONT_LEFT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FRONT_LEFT].command, 	// PID output : command sent to motor
							&motors[FRONT_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
		
			strcpy(motors[FRONT_RIGHT].debug_name, "FRONT RIGHT");
			motors[FRONT_RIGHT].type = M3508;
			motors[FRONT_RIGHT].can_rx_id = 0x200+2; // ID = 2
			motors[FRONT_RIGHT].can_tx_frame = 0x200; 
			motors[FRONT_RIGHT].can_tx_id = 2;
			pid_create(&motors[FRONT_RIGHT].pid, 
							&motors[FRONT_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FRONT_RIGHT].command, 	 // PID output : command sent to motor
							&motors[FRONT_RIGHT].setpoint, 	 // Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FRONT_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_RIGHT].debug_name, "BACK RIGHT");
			motors[BACK_RIGHT].type = M3508;
			motors[BACK_RIGHT].can_rx_id = 0x200+3; // ID = 3
			motors[BACK_RIGHT].can_tx_frame = 0x200; 
			motors[BACK_RIGHT].can_tx_id = 3;
			pid_create(&motors[BACK_RIGHT].pid, 
							&motors[BACK_RIGHT].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[BACK_RIGHT].command, 	// PID output : command sent to motor
							&motors[BACK_RIGHT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_RIGHT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[BACK_LEFT].debug_name, "BACK LEFT");
			motors[BACK_LEFT].type = M3508;
			motors[BACK_LEFT].can_rx_id = 0x200+4; // ID = 4
			motors[BACK_LEFT].can_tx_frame = 0x200; 
			motors[BACK_LEFT].can_tx_id = 4;
			pid_create(&motors[BACK_LEFT].pid, 
							&motors[BACK_LEFT].info.speed,  // PID input : feedback value on which we want to reach setpoint  
							&motors[BACK_LEFT].command, 	// PID output : command sent to motor
							&motors[BACK_LEFT].setpoint, 	// Setpoint : speed or position to be reached by motor
							pid_chassis_p, pid_chassis_i, pid_chassis_d); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[BACK_LEFT].pid, -16384, 16384); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_PITCH].debug_name, "TURRET PITCH");
			motors[TURRET_PITCH].type = GM6020;
			motors[TURRET_PITCH].can_rx_id = 0x204+1; // ID = 1
			motors[TURRET_PITCH].can_tx_frame = 0x1FF; 
			motors[TURRET_PITCH].can_tx_id = 1;
			motors[TURRET_PITCH].MIN_POSITION = 0; // Degrees
			motors[TURRET_PITCH].MAX_POSITION = 359; // Degrees
			motors[TURRET_PITCH].setpoint = 250; // Degrees //Initial value
			motors[TURRET_PITCH].direction = 1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_PITCH].pid, 
							&motors[TURRET_PITCH].info.angle_360, // PID input : feedback value on which we want to reach setpoint  
							&motors[TURRET_PITCH].command, 		  // PID output : command sent to motor
							&motors[TURRET_PITCH].setpoint, 	  // Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							400, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_PITCH].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_PITCH].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[TURRET_YAW].debug_name, "TURRET YAW");
			motors[TURRET_YAW].type = GM6020;
			motors[TURRET_YAW].can_rx_id = 0x204+2; // ID = 2
			motors[TURRET_YAW].can_tx_frame = 0x1FF; 
			motors[TURRET_YAW].can_tx_id = 2;
			motors[TURRET_YAW].MIN_POSITION = 0; // Degrees
			motors[TURRET_YAW].MAX_POSITION = 359; // Degrees
			motors[TURRET_YAW].setpoint = 208; // Degrees //Initial value
			motors[TURRET_YAW].direction = -1; // Selects control direction (-1 or 1)
			pid_create(&motors[TURRET_YAW].pid, 
							&motors[TURRET_YAW].info.angle_360, // PID input : feedback value on which we want to reach setpoint  
							&motors[TURRET_YAW].command, 		// PID output : command sent to motor
							&motors[TURRET_YAW].setpoint, 		// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							200, 100, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_circular(&motors[TURRET_YAW].pid, 360); // Circular control : allows motor setpoint to go from 359 deg to 001 deg without doing a full revolution
			pid_limits(&motors[TURRET_YAW].pid, -30000, 30000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[FEEDER].debug_name, "FEEDER");
			motors[FEEDER].type = M2006;
			motors[FEEDER].can_rx_id = 0x200+7; // ID = 7
			motors[FEEDER].can_tx_frame = 0x1FF; 
			motors[FEEDER].can_tx_id = 7-4; 
			motors[FEEDER].direction = -1;
			pid_create(&motors[FEEDER].pid, 
							&motors[FEEDER].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FEEDER].command, 	// PID output : command sent to motor
							&motors[FEEDER].setpoint, 	// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							0.5, 0.5, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER].pid, -10000, 10000); // Minimum and maximum command that can be sent to motor
			
			strcpy(motors[FEEDER2].debug_name, "FEEDER2");
			motors[FEEDER2].type = M2006;
			motors[FEEDER2].can_rx_id = 0x200+8; // ID = 8
			motors[FEEDER2].can_tx_frame = 0x1FF; 
			motors[FEEDER2].can_tx_id = 7-3; 
			motors[FEEDER2].direction = -1;
			pid_create(&motors[FEEDER2].pid, 
							&motors[FEEDER2].info.speed, // PID input : feedback value on which we want to reach setpoint  
							&motors[FEEDER2].command, 		// PID output : command sent to motor
							&motors[FEEDER2].setpoint, 	// Setpoint : speed or position to be reached by motor
							// TODO : Magic numbers
							0.5, 0.5, 0); // Kp, Ki, Kd : Regulation coefficients : http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
			pid_limits(&motors[FEEDER2].pid, -10000, 10000); // Minimum and maximum command that can be sent to motor

			break;
			
	}
}
