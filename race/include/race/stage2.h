#ifndef _STAGE2_H_
#define _STAGE2_H_

#include "ros/ros.h"
#include "race/mecanum.h"
#include "race/microswitch.h"
#include "race/vision.h"

/** Checkpoint Define **/
POINT RED__START(0, 0);  // UNSURE
POINT RED__END(0, 50+35);

POINT GREEN__START(-(25+10+22.5/2), 0);
POINT GREEN__END(0, 35+45+30);

POINT BLUE__START(22.5+10+20, 0);
POINT BLUE__END(0, 30+40+45);


void run2();
#endif