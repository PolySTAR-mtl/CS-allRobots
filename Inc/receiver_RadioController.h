/****************
   Description : Gestion des fonctionnalit�s du receiver Radio Controller
   Auteur : S�bastien FAGUET
	 Periph�riques : UART 1 RX (PB7) - DBUS
*****************/

#ifndef RECEIVER_RADIOCONTROLLER
#define RECEIVER_RADIOCONTROLLER
                                                                                            
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "main.h"

#define UART1_MAX_LEN     50
#define UART1_RX_BUFFLEN  18


typedef struct receiver_RadioController receiver_RadioController_t;
typedef struct receiver_RadioController_data receiver_RadioController_data_t;

struct receiver_RadioController_data
{
    /* rocker channel information */
    int16_t ch1; // Between -660 and 660
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
		float ch1_float; // Between -100.0 and 100.0
		float ch2_float;
		float ch3_float;
		float ch4_float;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
						uint16_t SPACE : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
						uint16_t T : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
    int16_t wheel;
};

struct receiver_RadioController
{
		char keyboard_mode; //0: RC, 1: Mouse and keyboard
    struct receiver_RadioController_data data;
    struct receiver_RadioController_data last_data;
    uint16_t state;
    uint16_t frequence;
};

void uart1_init(void);

/* Function called when data is recieved from controller */
void receiver_RadioController_callback_handler(void);
#endif

