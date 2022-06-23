/****************
   Description : Gestion des fonctionnalit?s du referee System
   Auteur : S?bastien FAGUET
	 Periph?riques : UART 6
*****************/


#include "rasp_pi1.h"

// Create Referee System variable
rasp_pi1_t rasp_pi1;

// Create data retrieval buffer for Referee System
uint8_t uart6_rx_buff[UART6_RX_BUFFLEN];

// Retrieve external variables
extern uint32_t signOfLife_refereeSystem_tick;

// Function called went data is received from receiver
void rasp_pi1_callback_handler(int8_t length) {
	/* frame_header (5-byte) */
	uint8_t end = uart6_rx_buff[5];
	
  if(end != (uint8_t)'\0') return;
	
	char ch[5] = {(char)uart6_rx_buff[0], (char)uart6_rx_buff[1], 
	(char)uart6_rx_buff[2], (char)uart6_rx_buff[3], 
	(char)uart6_rx_buff[4]};
	
	signOfLife_refereeSystem_tick = HAL_GetTick();
	
	sscanf(ch, "%f", &rasp_pi1.value);
	
}

/**
  * @brief   initialize uart6 device 
  * @param   
  * @retval  
  */
void uart6_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	uart_receive_dma_no_it(&huart6, uart6_rx_buff, UART6_MAX_LEN);
}
