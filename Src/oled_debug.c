/****************
   Description : Gestion du debug grâce au OLED
   Auteur : Sébastien FAGUET
*****************/

#include "oled_debug.h"

/* On stock à quelle page on est, initilisation au logo => page = 0 */
int8_t oled_debug_page = 0;

/* On récupère les variables exterieurs pour affichage de debug */
extern receiver_RadioController_t receiver_RadioController;
//extern refereeSystem_t refereeSystem;

void oled_debug(){
	static uint32_t tickstart = 0;
	if(tickstart == 0){
		tickstart = HAL_GetTick();
	}

  if ((HAL_GetTick() - tickstart) > 100){
		tickstart = HAL_GetTick();
		oled_debug_display();
	}
}

void oled_debug_display(){
	static int16_t raisingEdge = -1;
	int16_t button;
	if(oled_button_val() != raisingEdge){
		button = raisingEdge = oled_button_val();
	}else{
		button = BUTTON_STATIC;
	}
	
	
	if(button == BUTTON_RIGHT){
		oled_debug_page++;
	}
	if(button == BUTTON_LEFT){
		oled_debug_page--;
		if(oled_debug_page == -1){
			oled_debug_page = 2;
		}
	}
	
	oled_clear(Pen_Clear);
	switch(oled_debug_page){
		case 0:
			oled_LOGO();
			break;
		case 1:
			oled_debug_display_ReceiverRadioController(button);
			break;
		case 2:
			oled_debug_display_RefereeSystem(button);
			break;
		default:
			oled_debug_page = 0;
	}
	oled_refresh_gram();
}

void oled_debug_display_ReceiverRadioController(int16_t button){
	int16_t max_scroll = 9; 
	static int16_t scroll = 0;
	if(button == BUTTON_UP){
		scroll--;
	}
	if(button == BUTTON_DOWN){
		scroll++;
	}
	
	if(scroll == -1){
		scroll = max_scroll;
	}
	if(scroll == max_scroll+1){
		scroll = 0;
	}
	
	int ligne = 1;
	for(int16_t i = scroll ; i < scroll + 4 ; i++ ){
		switch(i){
			case 0: oled_printf(ligne, 1,"<-Radio Controller->"); break;
			case 1: oled_printf(ligne, 1,"Left T/B: %i", receiver_RadioController.data.ch1); break;
			case 2: oled_printf(ligne, 1,"Left L/R: %i", receiver_RadioController.data.ch2); break;
			case 3: oled_printf(ligne, 1,"Right T/B: %i", receiver_RadioController.data.ch3); break;
			case 4: oled_printf(ligne, 1,"Right L/R: %i", receiver_RadioController.data.ch4); break;
			case 5: oled_printf(ligne, 1,"Switch L: %i", receiver_RadioController.data.sw1); break;
			case 6: oled_printf(ligne, 1,"Switch R: %i", receiver_RadioController.data.sw2); break;
			case 7: oled_printf(ligne, 1,"Mouse X: %i", receiver_RadioController.data.mouse.x); break;
			case 8: oled_printf(ligne, 1,"Mouse Y: %i", receiver_RadioController.data.mouse.y); break;
			case 9: oled_printf(ligne, 1,"Mouse Z: %i", receiver_RadioController.data.mouse.z); break;
			case 10: oled_printf(ligne, 1,"Keyboard: %i", receiver_RadioController.data.kb.key_code); break;
			case 11: oled_printf(ligne, 1,"Left Dial: %i", receiver_RadioController.data.wheel); break;
		}
		ligne++;
	}
}



void oled_debug_display_RefereeSystem(int16_t button){
	int16_t max_scroll = 5; 
	static int16_t scroll = 0;
	if(button == BUTTON_UP){
		scroll--;
	}
	if(button == BUTTON_DOWN){
		scroll++;
	}
	
	if(scroll == -1){
		scroll = max_scroll;
	}
	if(scroll == max_scroll+1){
		scroll = 0;
	}
	
	int ligne = 1;
	for(int16_t i = scroll ; i < scroll + 4 ; i++ ){
		switch(i){
			case 0: oled_printf(ligne, 1,"<- Referee System ->"); break;
			//case 1: oled_printf(ligne, 1,"0x0001:", refereeSystem.game_status.game_type); break;
			//case 2: oled_printf(ligne, 2,"game_type: %i", refereeSystem.game_status.game_type); break;
			//case 3: oled_printf(ligne, 2,"game_progress: %i", refereeSystem.game_status.game_progress); break;
			//case 4: oled_printf(ligne, 2,"remain_time: %i", refereeSystem.game_status.stage_remain_time); break;
			//case 5: oled_printf(ligne, 1,"0x0002:"); break;
			//case 6: oled_printf(ligne, 2,"winner: %i", refereeSystem.game_result.winner); break;
		}
		ligne++;
	}
}
