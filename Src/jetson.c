/****************
   Description : Gestion des informations du Jetson
   Auteurs : Arthur VAN BETSBRUGGE, Ariane BOYCZUM-AUGER, Katherine ZAMUDIO-TURCOTTE, Marc-Antoine HIEN
	 Periph�riques : UART 7
*****************/


#include "jetson.h"

jetson_t jetson;

/* On cr�e le buffer de r�cup�ration de donn�es du referee System */
uint8_t uart7_rx_buff[UART6_RX_BUFFLEN];

/* On r�cup�re les variables exterieurs */
extern uint32_t signOfLife_jetson_tick;

/* fonction appel�e lorsqu'on recoit une unformation du r�cepteur */
void jetson_callback_handler(int8_t lenght){
	/* frame_header (5-byte) */
	uint8_t 	SOF 					= uart7_rx_buff[0];
	uint16_t 	cmd_id 				= (uart7_rx_buff[1] << 8 | uart7_rx_buff[2]);
	uint16_t 	data_length 	= uart7_rx_buff[3];
	
	if(SOF != 0xA5) return;
	signOfLife_jetson_tick = HAL_GetTick();
	
	
	/* data (n-byte) */
	switch(cmd_id){
		case 0x0002:
			memcpy(&jetson.robot_target_coordinates, &uart7_rx_buff[4], data_length);
			break;
		case 0x0003:
			memcpy(&jetson.rune_target_coordinates, &uart7_rx_buff[4], data_length);
			break;
		default: return;
	}
	
	/* frame_tail (2-byte, CRC16, whole package check)  */
	//uint16_t crc16 = (uart6_rx_buff[7+data_length] << 8 | uart6_rx_buff[7+data_length+1]);
}

/**
  * @brief   initialize uart7 device 
  * @param   
  * @retval  
  */
void uart7_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&huart7);
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);

	uart_receive_dma_no_it(&huart7, uart7_rx_buff, UART7_MAX_LEN);
}









