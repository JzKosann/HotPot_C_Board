//
// Created by shif on 2023/9/12.
//
#include "pidC.h"
//COMMENT:要不要使用c++更新下PID？
/*
	* @name   PID_Update
	* @brief  更新PID的数据，即更新PID的输入值
	* @param  WhichPID PID结构体指针,NowInput 当前PID输入值
  	* @retval None
*/
void PID_Update(PID_t *WhichPID, float NowInput) {
    WhichPID->PID_Input = NowInput;
    if (WhichPID->State_RampOrNormal == Ramp_e) {
        if (WhichPID->RampCountTime < WhichPID->RampTargetTime) {
            ++WhichPID->RampCountTime;
        } else {
            WhichPID->RampCountTime = 0;
            if (WhichPID->PID_Target < WhichPID->RampTarget) { //斜坡函数还没计数完，
                WhichPID->PID_Target += WhichPID->RampTargetStep;
                if (WhichPID->PID_Target >= WhichPID->RampTarget) {
                    WhichPID->PID_Target = WhichPID->RampTarget;
                    WhichPID->State_RampOrNormal = Normal_e;
                }
            } else if (WhichPID->PID_Target > WhichPID->RampTarget) {
                WhichPID->PID_Target -= WhichPID->RampTargetStep;
                if (WhichPID->PID_Target <= WhichPID->RampTarget) {
                    WhichPID->PID_Target = WhichPID->RampTarget;
                    WhichPID->State_RampOrNormal = Normal_e;
                }
            } else { //斜坡函数计数已经完成，要退出斜坡函数模式
                WhichPID->State_RampOrNormal = Normal_e;
            }
        }
    }
    WhichPID->PID_Err_lastlast = WhichPID->PID_Err_last;
    WhichPID->PID_Err_last = WhichPID->PID_Err_now;
    WhichPID->PID_Err_now = WhichPID->PID_Target - NowInput;

    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision)
        WhichPID->PID_Err_now = 0;

    WhichPID->PID_Err_all += WhichPID->PID_Err_now;

    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax) {
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    } else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax) {
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }
}

/*
	* @name   PID_GetPositionPID
	* @brief  位置式PID
	* @param  WhichPID PID结构体指针
    * @retval None
*/
float PID_GetPositionPID(PID_t *WhichPID) {
    WhichPID->PID_Out =
            WhichPID->Kp1 * WhichPID->PID_Err_now + WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
    WhichPID->PID_Out += (WhichPID->PID_Err_all * WhichPID->Ki1);

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep) {
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    }
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep) {
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
    }

    WhichPID->PID_lastout = WhichPID->PID_Out;

    return WhichPID->PID_Out;
}
/*
	* @name   PID_SpeedParamInit
	* @brief  初始化PID速度环的参数
	* @param  WhichPID PID结构体指针
	* @retval None
*/
void PID_SpeedParamInit(PID_t *WhichPID) { //初始化PID的默认参数
    WhichPID->Kp1 = 20.0;
    WhichPID->Ki1 = 1;
    WhichPID->Kd1 = 0;
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Normal_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
    WhichPID->RampTargetTime = RampDefault_Time;
    WhichPID->RampTargetStep = RampDefault_Step;
    WhichPID->PID_WorkType = PositionPID_e;
    WhichPID->Connected = 0;
    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/*
	* @name   PID_PosParamInit
	* @brief  初始化PID位置环的参数
	* @param  WhichPID PID结构体指针
	* @retval None
*/
void PID_PosParamInit(PID_t *WhichPID) {
    WhichPID->Kp1 = 10;
    WhichPID->Ki1 = 0;
    WhichPID->Kd1 = 0;
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Normal_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
    WhichPID->RampTargetTime = RampDefault_Time;
    WhichPID->RampTargetStep = RampDefault_Step;
    WhichPID->PID_WorkType = PositionPID_e;
    WhichPID->Connected = 0;
    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/*
	* @name   PID_SetTargetWithRamp
	* @brief  设置PID的目标值，使用斜坡函数
	* @param  WhichPID PID结构体指针,Tartget PID目标值
	* @retval None
*/
void PID_SetTargetWithRamp(PID_t *WhichPID, float Tartget) {
    if (WhichPID->RampTarget != Tartget) {
        WhichPID->RampTarget = Tartget;
        WhichPID->State_RampOrNormal = Ramp_e;
    }
}

/*
	* @name   PID_Clear
	* @brief  清除PID
	* @param  WhichPID PID结构体指针
	* @retval None
*/
void PID_Clear(PID_t *WhichPID) {
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Normal_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
}
