#ifndef _EKF_H_
#define _EKF_H_

#include "race/encoder.h"
#include "race/microswitch.h"

namespace EKF{
    void init(void);

    void moveTo(double x, double y, double z);
    void moveTo(double x_cor, double y_cor, double z_cor, CH_MICRO condition);
    
    void moveTo(POINT point);
    void moveTo(POINT point, CH_MICRO condition);
}

#endif