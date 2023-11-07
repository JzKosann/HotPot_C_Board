////
//// Created by ShiF on 2023/9/14.
////
//
//#include "servo.h"
//#include "gimbalc.h"
//#include "tim.h"
//
////extern TIM_HandleTypeDef htim4;
////static int8_t servo_status;
////void Servo_StartPWM(void)
////{
////    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
////    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1150);
////    servo_status = SERVO_OFF;
////}
////
////void  Servo_on(void)
////{
////    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 550); //舵机转90°
////    servo_status = SERVO_ON;
////}
////
////void Servo_off(void)
////{
////    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1150);
////    servo_status = SERVO_OFF;
////}
////
////int8_t Servo_GetStatus(void)
////{
////    return servo_status;
////}