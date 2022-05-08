/****************
   Description : Buzzer
   Auteurs : Katherine ZAMUDIO-TURCOTTE
   Periph�riques : TIMER 12
*****************/


#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"                                                                                     

#define DEFAULT_PITCH_BUZZ      40000
#define MAX_PSC                 2


void buzzer_set_pitch(uint16_t pitch);

void buzzer_start(void);

void buzzer_stop(void);

#endif // BUZZER_H
