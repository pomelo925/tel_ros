#ifndef _STAGE3_H_
#define _STAGE3_H_

#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/microswitch.h"
#include "race/scara.h"

/** Checkpoint Define **/
POINT UP(0,200);  // velocity may be too high
POINT DOWN(0,50);  
POINT LEFT(-60,0);
POINT TOUCH(0, 100);

void run3();

#endif