#include "Motor.h"

#define PWM_DUTY_MAX        ( 2133 )

#define DRV8701E_DIR_LEFT(x)    ((x) ? (DL_GPIO_setPins(MOTOR_DIR1_PORT, MOTOR_DIR1_PIN_26_PIN)) : (DL_GPIO_clearPins(MOTOR_DIR1_PORT, MOTOR_DIR1_PIN_26_PIN)) )
#define DRV8701E_DIR_RIGHT(x)   ((x) ? (DL_GPIO_setPins(MOTOR_DIR2_PORT, MOTOR_DIR2_PIN_27_PIN)) : (DL_GPIO_clearPins(MOTOR_DIR2_PORT, MOTOR_DIR2_PIN_27_PIN)) )

#define DRV8701E_PWM_LEFT(x)    (DL_TimerA_setCaptureCompareValue(PWM_FOR_MOTOR_INST, x, GPIO_PWM_FOR_MOTOR_C0_IDX))
#define DRV8701E_PWM_RIGHT(x)   (DL_TimerA_setCaptureCompareValue(PWM_FOR_MOTOR_INST, x, GPIO_PWM_FOR_MOTOR_C1_IDX))

void Motor_Init(void) {
    // PWM???SysConfig????????
}

void Set_Left_Speed(int duty) {
    int max = PWM_DUTY_MAX - 30;
    duty = duty > max ? max : (duty < -max ? (-max) : duty);
    
    if(duty >= 0) {
        DRV8701E_DIR_LEFT(0);
        DRV8701E_PWM_LEFT(duty);
    } else {
        DRV8701E_DIR_LEFT(1);
        DRV8701E_PWM_LEFT(-duty);
    }
}

void Set_Right_Speed(int duty) {
    int max = PWM_DUTY_MAX - 30;
    duty = duty > max ? max : (duty < -max ? (-max) : duty);
    
    if(duty >= 0) {
        DRV8701E_DIR_RIGHT(0);
        DRV8701E_PWM_RIGHT(duty);
    } else {
        DRV8701E_DIR_RIGHT(1);
        DRV8701E_PWM_RIGHT(-duty);
    }
}