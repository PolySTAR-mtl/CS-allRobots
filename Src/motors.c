/****************
   Description : Gestion des moteurs
   Auteur : S�bastien FAGUET
*****************/

#include "motors.h"

/* On r�cup�re les variables exterieurs */
extern motor_t motors[MAX_MOTORS];
extern pilote_t pilote;
extern receiver_RadioController_t receiver_RadioController;
extern refereeSystem_t refereeSystem;

/* Envoie les commandes sur le bus CAN pour tous les moteurs CAN */
void can_send_command(){
	static uint32_t tickstart = 0;
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}
  if((HAL_GetTick() - tickstart) < 10){
		return;
	}
	tickstart = HAL_GetTick();
	
	int32_t trams[3] = {0x200, 0x1FF, 0x2FF};
	for(int i = 0 ; i < 3 ; i++){
		CAN_TxHeaderTypeDef tx_header;
		uint8_t             tx_data[8] = {0};
			
		tx_header.StdId = trams[i];
		tx_header.IDE   = CAN_ID_STD;
		tx_header.RTR   = CAN_RTR_DATA;
		tx_header.DLC   = 8;
		tx_header.TransmitGlobalTime = DISABLE;
		
		for(int j = 0; j < MAX_MOTORS ; j++){
			if(motors[j].can_tx_frame == trams[i] && motors[j].can_tx_id == 1){
					tx_data[0] = ((int16_t) motors[j].command >> 8)&0xff;
					tx_data[1] = ((int16_t) motors[j].command )&0xff;
			}
			if(motors[j].can_tx_frame == trams[i] && motors[j].can_tx_id == 2){
				tx_data[2] = ((int16_t) motors[j].command >> 8)&0xff;
				tx_data[3] = ((int16_t) motors[j].command )&0xff;
			}
			if(motors[j].can_tx_frame == trams[i] && motors[j].can_tx_id == 3){
				tx_data[4] = ((int16_t) motors[j].command >> 8)&0xff;
				tx_data[5] = ((int16_t) motors[j].command )&0xff;
			}
			if(motors[j].can_tx_frame == trams[i] && motors[j].can_tx_id == 4){
				tx_data[6] = ((int16_t) motors[j].command >> 8)&0xff;
				tx_data[7] = ((int16_t) motors[j].command )&0xff;
			}
		}
		HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
	}
}

/* Fonction appel�e lors de la r�ception d'une information provenant d'un moteur */
void can_motors_callback_handler(int16_t rx_id, uint8_t* rx_buff){
	for(int j = 0; j < MAX_MOTORS ; j++){
		if(rx_id == motors[j].can_rx_id){
			//Si c'est le premier paquet, aller le lire et initialiser la tourelle
			if(motors[j].type == GM6020 && motors[j].signOfLife_tick == 0) {
						fill_motor_data(&motors[j], rx_buff);
						init_turret_data(&motors[j]);
						BOARD_LED_A_OFF();
			}
			fill_motor_data(&motors[j], rx_buff);
			return;
		}
	}
}

void init_turret_data(motor_t* motor){
	motor->setpoint = motor->info.angle_360;
	if(motor->MAX_POSITION > motor->MIN_POSITION){ 
		if(motor->MAX_POSITION < motor->info.angle_360 || motor->MIN_POSITION > motor->info.angle_360){
			error_boardA(1);
			if(motor->can_rx_id == 0x0205){
				return;
			} else {
				while(true);
			}
		}
	} else { // Si l'angle 0 est entre le min et le max (donc min > max)
		if(motor->MAX_POSITION < motor->info.angle_360 && motor->MIN_POSITION > motor->info.angle_360){
			error_boardA(1);
			if(motor->can_rx_id == 0x0205){
				return;
			} else {
				while(true);
			}
		}
	}
}

/*Rempli la structure motor.info avec les donn�es provenant du moteur */
void fill_motor_data (motor_t* motor, uint8_t* rx_buff){
	int16_t speed;
	motor->signOfLife_tick = HAL_GetTick();
	motor->info.angle = (int16_t)(rx_buff[0] << 8 | rx_buff[1]); 				//0 � 8191
	motor->info.angle_360 = 360.0 * motor->info.angle / 8191.0; 			//0 � 360 deg
	speed = (rx_buff[2] << 8 | rx_buff[3]);//rpm
	motor->info.speed = (float)speed;//rpm
	motor->info.torque = (int16_t)(rx_buff[4] << 8 | rx_buff[5]);
}

/* Modifie la setpoint tout en v�rifiant les limites de postion */
void add_setpoint_position(motor_t* motor, float value, float coeff){
	
	//S'assure que le moteur r�pond avant d'envoyer des consignes
	if (motor->signOfLife_tick != 0 && HAL_GetTick() - motor->signOfLife_tick > 100) return;
	
	double sensitivity_deadzone;
	if(receiver_RadioController.keyboard_mode){
		sensitivity_deadzone = pilote.sensitivity_mouse_deadzone;
	}else{
		sensitivity_deadzone = pilote.sensitivity_RC_deadzone;
	}
	if(value > -sensitivity_deadzone && value < sensitivity_deadzone) value = 0;
	
	float consigne_position = motor->setpoint + (motor->direction * value * coeff);
	if(consigne_position > 360) consigne_position -= (float) 360.0;
	if(consigne_position < 0) 	consigne_position += (float) 360.0;
	
	if(motor->MAX_POSITION > motor->MIN_POSITION){
		if(motor->MAX_POSITION > 0 && consigne_position > motor->MAX_POSITION) consigne_position = motor->MAX_POSITION;
		if(motor->MIN_POSITION > 0 && consigne_position < motor->MIN_POSITION) consigne_position = motor->MIN_POSITION;
	}else{
		if(motor->MAX_POSITION > 0 && consigne_position > motor->MAX_POSITION) {
			if(motor->MIN_POSITION > 0 && consigne_position < motor->MIN_POSITION) {
				if((consigne_position - motor->MAX_POSITION) < (motor->MIN_POSITION - consigne_position)){
					consigne_position = motor->MAX_POSITION;
				}else{
					consigne_position = motor->MIN_POSITION;
				}
			}
		}
	}
		
	motor->setpoint = consigne_position;
}

/* Initialise le CAN 1 */
void can1_init(){
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
  HAL_CAN_Start(&hcan1);                          // start can
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can rx interrupt
}





/* Initialise le TIMER 1 pour les PWM */
void PWM_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // start pwm output
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/* Set le duty cycle de tous les channels PWM */ 
void PWM_SetAllDuty(TIM_HandleTypeDef *tim, float duty_ch1, float duty_ch2){
	duty_ch1 = duty_ch1/10 + 0.10f; //rescale before send
	duty_ch2 = duty_ch2/10 + 0.10f;
	tim->Instance->CCR1 = (10000*duty_ch1) - 1;
	tim->Instance->CCR2 = (10000*duty_ch2) - 1;
}

/* scales all PWM duty cycles between 0 and 1 
Cette fonction permet d'�talonner le snail pour que ca commande 0 = min, et 1 = max

Pour utiliser cette fonction, il faut le faire moteur PWM par moteur PWM
	1- D�brancher l'alimentation de tous les moteurs PWM
	2- Brancher l'alimentation du moteur PWM a �talonner sur la sortie XT30 de la BOARD A - POWER 1
  2- Envoyer sur la board A gr�ce au ST LINK un programe avec cette fonction appel� dans le main.c, avant la boucle infinie
	3- Brancher l'alimentation 24V de la board A, le programme va se lancer : 
		a) va couper l'alimentation 24V du snail
    b) Met l'impulsion PWM au max
    c) Attends, allume le snail, attends
		d) Coupe tous le signal PWM

*/
void PWM_ScaleAll(TIM_HandleTypeDef *tim, bool switchRotationalDirection){ //il faudrait jouer sur l'allumage des ports d'alimentation des snails (voir Nathan pour plus de d�tails)
	HAL_GPIO_WritePin(GPIOH, BOARD_POWER1_CTRL_Pin, GPIO_PIN_RESET); // switch off 24v power
	PWM_SetAllDuty(&htim1,1,1);
	HAL_Delay(10);
	
	HAL_GPIO_WritePin(GPIOH, BOARD_POWER1_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
	if(switchRotationalDirection) HAL_Delay(7500);
	else HAL_Delay(3500);
	
	PWM_SetAllDuty(&htim1,0,0);
	HAL_Delay(500);
	while(1);
}
