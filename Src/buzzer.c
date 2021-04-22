/****************
   Description : Buzzer
   Auteurs : Katherine ZAMUDIO-TURCOTTE
   Periphériques : TIMER 12
*****************/


#include "buzzer.h"

// start a buzz at a precise pitch
void start_buzz_pitch(uint16_t pitch)
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

// start a buzz at the default pitch
void start_buzz(void)
{
    //start_buzz_pitch(DEFAULT_PITCH_BUZZ); 
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

// stop the buzz
void stop_buzz(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);    // Start the buzzer
}







