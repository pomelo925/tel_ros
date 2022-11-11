#ifndef _STAEG1_H_
#define _STAGE1_H_

#include "race/mecanum.h"

/** Checkpoint (relative) Define **/
POINT START_PHOTO_1(15, 100-15); 

POINT START_PHOTO_2(0,0);


POINT B_PUT(35, 115); 
POINT B_LEAVE(0, 40);  
POINT B_NEXT_STAGE(25, 80);  // reach stage 2 RED_START 

void run1();
#endif