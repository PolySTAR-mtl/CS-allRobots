/****************
   Description : Gestion des fonctionnalités sur la BoardA
   Auteur : Sébastien FAGUET
*****************/

#ifndef BOARDA_HANDLE
#define BOARDA_HANDLE

#include "main.h"

void signOfLife(void);
void signOfLife_Receiver_RadioController(void);
void signOfLife_refereeSystem(void);
void signOfLife_can1(void);
void killMotors(void);
void uart_debug(void);
void uart_debug_printf(const char *fmt,...);
void uart_debug_command(char* command);
int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void error_board_A(uint8_t errorCode);

#endif
