#include "race/stage1_lib.h"

// 主程式
void run(){
    initial_all();
    phase1();
}

// 第一階段：起點 ~ 影像辨識起點
void phase1(){
    moveTo(-20,50);
}

// 初始化所有用到的 package
void initial_all(){
    velocity_initial();
}

