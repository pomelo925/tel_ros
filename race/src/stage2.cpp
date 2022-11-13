#include "race/stage2.h"

void run2(void){
    MECANUM::moveTo(RED__START);
    MECANUM::moveTo(RED__END);
    MECANUM::moveTo(GREEN__START);
    MECANUM::moveTo(GREEN__END);
    MECANUM::moveTo(BLUE__START);
    MECANUM::moveTo(BLUE__END);
}