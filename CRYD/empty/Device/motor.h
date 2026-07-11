#ifndef Motor_H
#define Motor_H

#include "board.h"

void Motor_Init(void);
void Set_Left_Speed(int duty);
void Set_Right_Speed(int duty);

#endif