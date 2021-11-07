/****************
   Description : Gestion des fonctionnalités du receiver Radio Controller
   Auteur : Sébastien FAGUET
	 Periphériques : UART 1 RX (PB7) - DBUS
*****************/
                  
#include "receiver_RadioController.h"
#include "jetson.h"


uint8_t uart1_rx_buff[UART1_RX_BUFFLEN];
receiver_RadioController_t receiver_RadioController = {false};


/* On récupère les variables exterieurs */
extern uint32_t signOfLife_Receiver_RadioController_tick;

/* fonction appelée lorsqu'on recoit une unformation du récepteur */
void receiver_RadioController_callback_handler()
{
	signOfLife_Receiver_RadioController_tick = HAL_GetTick();
	memcpy(&receiver_RadioController.last_data, &receiver_RadioController.data, sizeof(struct receiver_RadioController_data));

	receiver_RadioController.data.ch1 = (uart1_rx_buff[0] | uart1_rx_buff[1] << 8) & 0x07FF;
	receiver_RadioController.data.ch1 -= 1024;
	receiver_RadioController.data.ch2 = (uart1_rx_buff[1] >> 3 | uart1_rx_buff[2] << 5) & 0x07FF;
	receiver_RadioController.data.ch2 -= 1024;
	receiver_RadioController.data.ch3 = (uart1_rx_buff[2] >> 6 | uart1_rx_buff[3] << 2 | uart1_rx_buff[4] << 10) & 0x07FF;
	receiver_RadioController.data.ch3 -= 1024;
	receiver_RadioController.data.ch4 = (uart1_rx_buff[4] >> 1 | uart1_rx_buff[5] << 7) & 0x07FF;
	receiver_RadioController.data.ch4 -= 1024;

	receiver_RadioController.data.ch1_float = (float) (receiver_RadioController.data.ch1 / 6.6); //Entre -100% et 100%
	receiver_RadioController.data.ch2_float = (float) (receiver_RadioController.data.ch2 / 6.6);
	receiver_RadioController.data.ch3_float = (float) (receiver_RadioController.data.ch3 / 6.6);
	receiver_RadioController.data.ch4_float = (float) (receiver_RadioController.data.ch4 / 6.6);
	
	receiver_RadioController.data.sw1 = ((uart1_rx_buff[5] >> 4) & 0x000C) >> 2;
	receiver_RadioController.data.sw2 = (uart1_rx_buff[5] >> 4) & 0x0003;
	
	if ((abs(receiver_RadioController.data.ch1) > 660)) receiver_RadioController.data.ch1 = 0;
	if ((abs(receiver_RadioController.data.ch2) > 660)) receiver_RadioController.data.ch2 = 0;
	if ((abs(receiver_RadioController.data.ch3) > 660)) receiver_RadioController.data.ch3 = 0;
	if ((abs(receiver_RadioController.data.ch4) > 660)) receiver_RadioController.data.ch4 = 0;

	receiver_RadioController.data.mouse.x = uart1_rx_buff[6] | (uart1_rx_buff[7] << 8); // x axis
	receiver_RadioController.data.mouse.y = -(uart1_rx_buff[8] | (uart1_rx_buff[9] << 8));
	receiver_RadioController.data.mouse.z = uart1_rx_buff[10] | (uart1_rx_buff[11] << 8);

	receiver_RadioController.data.mouse.l = uart1_rx_buff[12];
	receiver_RadioController.data.mouse.r = uart1_rx_buff[13];

	receiver_RadioController.data.kb.key_code = uart1_rx_buff[14] | uart1_rx_buff[15] << 8; // key borad code

	receiver_RadioController.data.wheel = (uart1_rx_buff[16] | uart1_rx_buff[17] << 8);
	receiver_RadioController.data.wheel -= 1024;
	
	if(receiver_RadioController.data.mouse.x != 0 ||
					receiver_RadioController.data.mouse.y != 0 ||
					receiver_RadioController.data.mouse.z != 0 ||
					receiver_RadioController.data.mouse.l != 0 ||
					receiver_RadioController.data.kb.key_code != 0){
		receiver_RadioController.keyboard_mode = true;
	}
					
	//Envoie a la jetson l'info de changer de cible dès que le bouton X ou Z est presse
	if(receiver_RadioController.data.kb.bit.X && !receiver_RadioController.last_data.kb.bit.X){
		jetson_uart_send_command('R');
	}else if(receiver_RadioController.data.kb.bit.Z && !receiver_RadioController.last_data.kb.bit.Z){
	  jetson_uart_send_command('L');
	}
}


/**
  * @brief   initialize dbus uart device 
  * @param   
  * @retval  
  */
void uart1_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	uart_receive_dma_no_it(&huart1, uart1_rx_buff, UART1_MAX_LEN);
}
