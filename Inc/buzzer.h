/****************
   Description : Buzzer
   Auteurs : Katherine ZAMUDIO-TURCOTTE
   Periphériques : TIMER 12
*****************/


#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"                                                                                     

#define DEFAULT_PITCH_BUZZ      1000


void start_buzz_pitch(uint16_t pitch);

void start_buzz(void);

void stop_buzz(void);

#endif // BUZZER_H
