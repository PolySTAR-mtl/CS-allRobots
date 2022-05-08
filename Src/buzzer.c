/****************
   Description : Buzzer
   Auteurs : Katherine ZAMUDIO-TURCOTTE
   Periph�riques : TIMER 12
*****************/


#include "buzzer.h"


// Start a buzz at a specific pitch
void buzzer_set_pitch(uint16_t pitch)
{		
    __HAL_TIM_PRESCALER(&htim12, MAX_PSC);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pitch);
}

// Start a buzz at the default pitch
void buzzer_start(void)
{	
    buzzer_set_pitch(DEFAULT_PITCH_BUZZ); 
}

// Stop the buzz
void buzzer_stop(void)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);    // Stop the buzzer
}







