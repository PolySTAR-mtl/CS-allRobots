/****************
   Description : Gestion du cannon
   Auteur : S�bastien FAGUET
*****************/

#ifndef CANON
#define CANON

#include "main.h"
#include "referee_system.h"

void canon_shoot_start(float speed, float rate);
void canon_process_inputs(void);
void canon_shoot_end(void);

#endif
