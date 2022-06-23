/****************
   Description : Gestion du debug gr�ce au OLED
   Auteur : S�bastien FAGUET
*****************/


#ifndef DEVICE_OLED_DEBUG
#define DEVICE_OLED_DEBUG

#include "main.h"
#include "oled.h"
#include "receiver_RadioController.h"
#include "rasp_pi1.h"

void oled_debug(void);
void oled_debug_display(void);
void oled_debug_display_ReceiverRadioController(int16_t button);
void oled_debug_display_RefereeSystem(int16_t button);

#endif
