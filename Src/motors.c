/****************
   Description : Gestion des moteurs
   Auteur : Sébastien FAGUET
*****************/

#include "motors.h"

/* On récupère les variables exterieurs */
extern motor_t motors[MAX_MOTORS];

/* Envoie les commandes sur le bus CAN pour tous les moteurs CAN */
void can_send_command(){
	static uint32_t tickstart = 0;
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}
  if((HAL_GetTick() - tickstart) < 10){
		return;
	}
	
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

/* Fonction appelée lors de la réception d'une information provenant d'un moteur */
void can_motors_callback_handler(int16_t rx_id, uint8_t* rx_buff){
	int16_t speed;
	for(int j = 0; j < MAX_MOTORS ; j++){
		if(rx_id == motors[j].can_rx_id){
			motors[j].signOfLife_tick = HAL_GetTick();
			switch(motors[j].type){
				case GM6020:
				case M3508:
				case M2006:
					motors[j].info.angle = (int16_t)(rx_buff[0] << 8 | rx_buff[1]); 				//0 à 8191
					motors[j].info.angle_360 = 360.0 * motors[j].info.angle / 8191.0; 			//0 à 360 deg
					speed = (rx_buff[2] << 8 | rx_buff[3]);//rpm
					motors[j].info.speed = (float)speed;//rpm
					motors[j].info.torque = (int16_t)(rx_buff[4] << 8 | rx_buff[5]);
					break;
			}
			return;
		}
	}
}

/* Modifie la consigne tout en vérifiant les limites de postion */
void add_consigne_position(motor_t* motor, float value){
	float consigne_position = motor->consigne;
	consigne_position += value;
	if(consigne_position > 360) consigne_position -= (float) 360.0;
	if(consigne_position < 0) 	consigne_position += (float) 360.0;
	if(motor->MAX_POSITION > 0 && consigne_position > motor->MAX_POSITION) consigne_position = motor->MAX_POSITION;
	if(motor->MIN_POSITION > 0 && consigne_position < motor->MIN_POSITION) consigne_position = motor->MIN_POSITION;
	motor->consigne = consigne_position;
}

/* Initialise le CAN 1 */
void can1_init(){
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
