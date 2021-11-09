/****************
   Description : Buzzer
   Auteurs : Katherine ZAMUDIO-TURCOTTE
   Periph�riques : TIMER 12
*****************/


#include "buzzer.h"

// Start a buzz at a specific pitch
void buzzer_set_pitch(uint16_t pitch)
{
    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pitch;  // Change pitch with PULSE value
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
		HAL_Delay(100);
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);    // Start the buzzer
}

// Start a buzz at the default pitch
void buzzer_start(void)
{
    //buzzer_set_pitch(DEFAULT_PITCH_BUZZ); 
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

// Stop the buzz
void buzzer_stop(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);    // Stop the buzzer
}







