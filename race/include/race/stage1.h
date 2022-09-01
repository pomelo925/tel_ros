#ifndef _STAEG1_H_
#define _STAGE1_H_

#include "ros/ros.h"
#include "race/encoder.h"

/** Checkpoint (relative) Define **/
POINT A_START_PUSH(15, 100-15); // expand arm preparing for pushing cubes
POINT A_STOP_PUSH(0, 35); 
POINT A_CORNER(0, 20+15);
POINT A_CAM(-15-25, 0);  // let camera at mid 

POINT B_PUT(35, 115); 
POINT B_LEAVE(0, 40);  
POINT B_NEXT_STAGE(25, 80);  // reach stage 2 RED_START 

#endif