/****************
   Description : Gestion des fonctionnalités du OLED
   Auteur : Sébastien FAGUET
	 Periphériques : SPI1(PA7, PB3), ADC1(PA6), PB9, PB10
*****************/


#ifndef DEVICE_OLED
#define DEVICE_OLED

#include "spi.h"
#include "math.h"
#include "adc.h"
#include <stdio.h>
#include <stdarg.h>         
#include <stdlib.h>
#include <stdint.h>
#include "main.h"

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12

#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)

#define BUTTON_PRESSED  0
#define BUTTON_LEFT     847
#define BUTTON_RIGHT    1730
#define BUTTON_UP       2457
#define BUTTON_DOWN     3280
#define BUTTON_STATIC   4090
#define BUTTON_RANGE    128 

typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;

/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);

// Drawing functions
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...);
void oled_LOGO(void);
//Fonctions avec le boutton
float oled_button_get_volt(void);
int16_t oled_button_val(void);

#endif
