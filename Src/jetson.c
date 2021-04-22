/****************
   Description : Gestion des informations du Jetson
   Auteurs : Arthur VAN BETSBRUGGE, Ariane BOYCZUM-AUGER, Katherine ZAMUDIO-TURCOTTE, Marc-Antoine HIEN
	 Periphériques : UART 7
*****************/


#include "jetson.h"

jetson_t jetson;

/* On crée le buffer de récupération de données du referee System */
uint8_t uart7_rx_buff[UART7_RX_BUFFLEN];

/* On récupère les variables exterieurs */
extern uint32_t signOfLife_jetson_tick;

/* fonction appelée lorsqu'on recoit une unformation du récepteur */
void jetson_callback_handler(int8_t lenght){
	
	/* frame_header (5-byte) */
	uint8_t 	SOF 					= uart7_rx_buff[0];
	uint16_t 	cmd_id 				= (uart7_rx_buff[2] << 8 | uart7_rx_buff[1]);
	uint16_t 	data_length 	= uart7_rx_buff[3];
	
	if(SOF != 0xFC) return;
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
}

/* Fonction d'envoie des commandes à la jetson */
void jetson_uart_send_command(uint8_t command){

	jetson.switch_informations.switch_target = command;
	
	uint8_t buffer[5];
	
	uint8_t 	SOF 					= 0xFC;
	uint16_t 	cmd_id 				= 0x0001; 	//switch target
	uint8_t 	data_length 	= 0x01;
	
	buffer[0] = SOF;
	buffer[1] = cmd_id;
	buffer[2] = cmd_id >> 2;
	buffer[3] = data_length;
	buffer[4] = jetson.switch_informations.switch_target;
	
	//envoie les 5 octets de la commande au uart7 vers la jetson
	HAL_UART_Transmit(&huart7, (uint8_t*) buffer, 5, 10);
	
	jetson.switch_informations.switch_target = 0x00;
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










