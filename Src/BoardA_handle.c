/****************
   Description : Gestion des fonctionnalit�s sur la BoardA (LEDS, signes de vie, ...)
   Auteur : S�bastien FAGUET
*****************/


#include "main.h"
#include "leds.h"
#include "motors.h"
#include "usart.h"
#include "robot_configuration.h"
#include "receiver_RadioController.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h> 

/* Create sign of life variables. Value indicates last time it was updated */
uint32_t signOfLife_Receiver_RadioController_tick = 0;
uint32_t signOfLife_refereeSystem_tick = 0;
uint32_t signOfLife_jetson_tick = 0;
uint32_t signOfLife_CAN1_tick = 0;

/* Retrieve external variables for debug display */
extern motor_t motors[MAX_MOTORS];
extern receiver_RadioController_t receiver_RadioController;
extern jetson_t jetson;

/* Manages signs of life. LED RED : Program is running */
void signOfLife(){
	static uint32_t tickstart = 0;
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}
  if ((HAL_GetTick() - tickstart) > 1000){
		tickstart = HAL_GetTick();
		BOARD_LED_RED_TOGGLE();
	}
	signOfLife_Receiver_RadioController();
	signOfLife_refereeSystem();
	signOfLife_can1();
}

/* Manages reciever sign of life. LED A : Receiving data from Radio Controller*/
void signOfLife_Receiver_RadioController(){
  if ((HAL_GetTick() - signOfLife_Receiver_RadioController_tick) < 50){
		BOARD_LED_A_OFF();
		pid_enable(true);
	}else{
		killMotors();
		error_boardA(4);
	}
}

/* Manages referee system sign of life. LED B : Receiving data from Referee System*/
void signOfLife_refereeSystem(){
  if ((HAL_GetTick() - signOfLife_refereeSystem_tick) < 2000){
		BOARD_LED_B_ON();
	}else{
		BOARD_LED_B_OFF();
	}
}

/* Manages CAN sign of life. LED C : Receiving data on bus CAN1*/
void signOfLife_can1(){
  if ((HAL_GetTick() - motors[TURRET_YAW].signOfLife_tick) < 100){
		BOARD_LED_C_ON();
	}else{
		BOARD_LED_C_OFF();
	}
	if ((HAL_GetTick() - motors[TURRET_PITCH].signOfLife_tick) < 100){
		BOARD_LED_D_ON();
	}else{
		BOARD_LED_D_OFF();
	}
	if ((HAL_GetTick() - motors[FRONT_RIGHT].signOfLife_tick) < 100){
		BOARD_LED_E_ON();
	}else{
		BOARD_LED_E_OFF();
	}
	if ((HAL_GetTick() - motors[FRONT_LEFT].signOfLife_tick) < 100){
		BOARD_LED_F_ON();
	}else{
		BOARD_LED_F_OFF();
		killMotors();
	}
	if ((HAL_GetTick() - motors[BACK_RIGHT].signOfLife_tick) < 100){
		BOARD_LED_G_ON();
	}else{
		BOARD_LED_G_OFF();
		killMotors();
	}
	if ((HAL_GetTick() - motors[BACK_LEFT].signOfLife_tick) < 100){
		BOARD_LED_H_ON();
	}else{
		BOARD_LED_H_OFF();
	}
}

/* Emergency stop for motors */
void killMotors(){
	receiver_RadioController.data.ch1_float = 0;
		receiver_RadioController.data.ch2_float = 0;
		receiver_RadioController.data.ch3_float = 0;
		receiver_RadioController.data.ch4_float = 0;
		receiver_RadioController.data.sw2 = 2;
		receiver_RadioController.data.wheel = 0;
		pid_enable(false);
		
		for(int i = 0; i < MAX_MOTORS; i++){
			if(motors[i].type != GM6020){
				motors[i].setpoint = 0;
				motors[i].command = 0;
			} else {
				motors[i].setpoint = motors[i].info.angle_360;
				motors[i].command = motors[i].info.angle_360;
			}
		}
		canon_shoot_end();	
}

/* Allows display of debug information on UART
Example : uart_debug_printf("\tAngle: %u (%x)\r\n", motors[TURRET_YAW].info.angle, motors[TURRET_YAW].info.angle);
*/
void uart_debug(){
	static uint32_t tickstart = 0;
	//static char buff[1000] = {0};
	//static char buff2[1000] = {0};
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}
  if ((HAL_GetTick() - tickstart) < 200){
		return;
	}
	tickstart = HAL_GetTick();
	//snprintf(buff2, 1000, "%c,teta=%d,phi=%d,d=%d                  \r", jetson.robot_target_coordinates.target_located, jetson.robot_target_coordinates.theta_target_location, jetson.robot_target_coordinates.phi_target_location, jetson.robot_target_coordinates.d_target_location);
	//HAL_UART_Transmit_DMA(&huart8, (uint8_t*)buff2, strlen(buff2));
	
	//uart_debug_command("[2J"); //Clear entire screen
	uart_debug_printf("%c,teta=%d,phi=%d,d=%d \r\n", jetson.robot_target_coordinates.target_located, jetson.robot_target_coordinates.theta_target_location, jetson.robot_target_coordinates.phi_target_location, jetson.robot_target_coordinates.d_target_location);
	//uart_debug_printf("\r\nTOURELLE YAW\r\n");
	//uart_debug_printf("\tAngle: %f \r\n", motors[TURRET_PITCH].info.angle_360);
	//uart_debug_
	/*printf("\tSpeed: %f\r\n", motors[TURRET_YAW].info.speed);
	uart_debug_printf("\tTorque: %d\r\n", motors[TURRET_YAW].info.torque);
	uart_debug_printf("\tTemperature: %d\r\n", motors[TURRET_YAW].info.temp);
	uart_debug_printf("\tCommand: %f\r\n", motors[TURRET_YAW].command);*/
}

/* Print function for debug UART*/
void uart_debug_printf(const char *fmt,...){
	uint8_t BUF[128] = {0};
	va_list ap;

	va_start(ap, fmt);
	vsprintf((char *)BUF, fmt, ap);
	va_end(ap);

	HAL_UART_Transmit(&huart8, BUF, 128, 10);
}

/* Special command function for debug UART */
void uart_debug_command(char* command){
	//http://www.climagic.org/mirrors/VT100_Escape_Codes.html
	char BUF[10] = {27,0};
	strcpy(BUF+1, command);
	HAL_UART_Transmit(&huart8, (uint8_t*) BUF, 4, 10);
}

/**

  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

void error_boardA(uint8_t errorCode){    
	BOARD_LED_ALL_OFF();    
	if (errorCode & 0x1){
    BOARD_LED_A_ON();
	}    
	if (errorCode & 0x2){
		BOARD_LED_B_ON();
	}    
	if (errorCode & 0x4){
		BOARD_LED_C_ON();
	}    
	if (errorCode & 0x8){
		BOARD_LED_D_ON();
		while((HAL_GetTick() - signOfLife_Receiver_RadioController_tick) < 50);
	}    
	if (errorCode & 0x10){
		BOARD_LED_E_ON();
	}    
	if (errorCode & 0x20){
		BOARD_LED_F_ON();
	}    
	if (errorCode & 0x40){
		BOARD_LED_G_ON();
	}    
	if (errorCode & 0x80){
		BOARD_LED_H_ON();
	}    
	BOARD_LED_RED_ON();
	
}
