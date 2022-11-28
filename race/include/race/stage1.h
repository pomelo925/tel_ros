#ifndef _STAEG1_H_
#define _STAGE1_H_

#include "race/scara.h"
#include "race/mecanum.h"
#include "race/vision.h"
// #include "race/microswitch.h"

#define stage1_yaml "/home/ditrobotics/TEL/src/race/path/stage1.yaml"
#define PHASE_ONE 1
#define PHASE_TWO 1+5
#define PHASE_THREE 1+5+3
#define PHASE_FOUR 1+5+3+4

void run1();

#endif