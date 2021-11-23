/****************
   Description : Gestion des fonctionnalit�s du referee System
   Auteur : S�bastien FAGUET
	 Periph�riques : UART 6
*****************/

#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#include "main.h"                                                                                     
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "oled.h"
#include "motors.h"

#define UART6_MAX_LEN     30
#define UART6_RX_BUFFLEN  50


typedef __packed struct
{
	uint8_t game_type : 4; 		// Competition Type 1: RoboMaster Robotics Competition; 2: RoboMaster Technical Challenge; 3: RoboMaster ICRA
	uint8_t game_progress : 4; 	// Current Competition Stage 0: Pre-match; 1: Setup Period; 2: Referee System Initialization Period; 3: 5-second Countdown; 4: Round Period; 5: Calculation Period
	uint16_t stage_remain_time; // Remaining time of the current period (unit: s)
} ext_game_status_t;

typedef __packed struct
{
	uint8_t winner; // 0: Draw; 1: Red win; 2: Blue win
} ext_game_result_t; 

typedef __packed struct
{
	uint16_t red_1_robot_HP; //Red 1 Hero HP. If the robot has not entered the stage or is defeated, its HP will be 0.
	uint16_t red_2_robot_HP; //Red 2 Engineer HP
	uint16_t red_3_robot_HP; //Red 3 Standard HP
	uint16_t red_4_robot_HP; //Red 4 Standard HP
	uint16_t red_5_robot_HP; //Red 5 Standard HP
	uint16_t red_7_robot_HP; //Red 7 Sentry HP
	uint16_t red_outpost_HP; //Red outpost HP
	uint16_t red_base_HP;    //Red Base HP
	uint16_t blue_1_robot_HP;//Blue 1 Hero HP
	uint16_t blue_2_robot_HP;//Blue 2 Engineer HP
	uint16_t blue_3_robot_HP;//Blue 3 Standard HP
	uint16_t blue_4_robot_HP;//Blue 4 Standard HP
	uint16_t blue_5_robot_HP;//Blue 5 Standard HP
	uint16_t blue_7_robot_HP;//Blue 7 Sentry HP
	uint16_t blue_outpost_HP;//Blue outpost HP
	uint16_t blue_base_HP;   //Blue Base HP
} ext_game_robot_HP_t;

typedef __packed struct
{
	uint8_t dart_belong; //Dart launchinging team: 1: From Red Team 2: From Blue Team
	uint16_t stage_remaining_time; //Remaining competition time when the dart is launched (s)
} ext_dart_status_t;

typedef __packed struct
{
	uint8_t F1_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F1_zone_buff_debuff_status:3;		//status information 1: Red Restoration Zone 2: Red Projectile Supplier Zone 3: Blue Restoration Zone 4: Blue Projectile Supplier Zone 5: Launch Penalty Zone 6; Movement Penalty Zone
	uint8_t F2_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F2_zone_buff_debuff_status:3;
	uint8_t F3_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F3_zone_buff_debuff_status:3;
	uint8_t F4_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F4_zone_buff_debuff_status:3;
	uint8_t F5_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F5_zone_buff_debuff_status:3;
	uint8_t F6_zone_status:1;								//activation status 0: unactivated 1: activated
	uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;

typedef __packed struct
{
	/*
	The occupation status of Landing Pad of one�s own side
		0 indicates no robot occupies;
		1 indicates that Aerial has occupied the Landing Pad but does not stop the propeller;
		2 indicates that Aerial has occupied the Landing Pad and stopped the propeller
	*/
	uint8_t status_of_Landing_Pad;
	uint8_t small_power_rune_status; //Small Power Rune and 1 indicates it has been activated;
	uint8_t large_base_shield_status; //Large Power Rune and 1 indicates it has been activated;
	uint8_t base_shield_status; //1 indicates that Base has Virtual Shield HP;  0 indicates that Base has no Virtual Shield HP;
} ext_event_data_t;

typedef __packed struct
{
	uint8_t supply_projectile_id; //Projectile Supplier outlet ID: 1: Projectile Supplier outlet #1; 2: Projectile Supplier outlet #2
	/*
	Projectile Supply robot ID: 
	0 indicates that no robot supplies projectile; 
	1 indicates that Red Hero supplies; 
	2 Red Engineer; 
	3/4/5 Red Standard; 
	101 Blue Hero; 
	102 Blue Engineer; 
	103/104/105 Blue Standard
	*/
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step; //The open and close mode of Projectile outlet: 0 indicates close; 1 indicates preparing for projectiles, 2 indicates falling projectiles
	uint8_t supply_projectile_quantity; //Quantity of Projectile Supply: 50: 50 projectiles 100: 100 projectiles 150: 150 projectiles 200: 200 projectiles
} ext_supply_projectile_action_t;

typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit;
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;
 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
 uint16_t chassis_volt;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t chassis_power_buffer;
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
 uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
	ext_game_status_t game_status;
	ext_game_result_t game_result;
	ext_game_robot_HP_t game_robot_HP;
	ext_dart_status_t dart_status;
	ext_ICRA_buff_debuff_zone_status_t ICRA_buff_debuff_zone_status;
	ext_event_data_t event_data;
	ext_supply_projectile_action_t supply_projectile_action;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t power_heat_data;
	ext_robot_hurt_t robot_hurt;
	ext_shoot_data_t shoot_data;
} refereeSystem_t;


/* Function called when data is recieved from referee system */
void refereeSystem_callback_handler(int8_t length);

void uart6_init(void);

#endif // __REFEREE_SYSTEM_H__
