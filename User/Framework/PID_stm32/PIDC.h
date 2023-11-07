/*
 * File: PID.h
 *
 * Code generated for Simulink model :PID.
 *
 * Model version      : 1.17
 * Simulink Coder version    : 9.4 (R2020b) 29-Jul-2020
 * TLC version       : 9.4 (Aug 20 2020)
 * C/C++ source code generated on  : Wed Apr  5 04:53:36 2023
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#ifndef RTW_HEADER_PID_h_
#define RTW_HEADER_PID_h_
#ifdef __cplusplus
extern "C" {
#endif
#include <stddef.h>
#include "STM32_Config.h"
#include "PID_External_Functions.h"
#ifndef PID_COMMON_INCLUDES_
#define PID_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PID_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
#define rtmStepTask(rtm, idx)          ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
#define rtmTaskCounter(rtm, idx)       ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define PID_M                          (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T FilterDifferentiatorTF_states;/* '<S36>/Filter Differentiator TF' */
  real_T Integrator_DSTATE;            /* '<S43>/Integrator' */
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T FilterDifferentiatorTF_states_b;/* '<S144>/Filter Differentiator TF' */
  real_T Integrator_DSTATE_m;          /* '<S151>/Integrator' */
  real_T FilterDifferentiatorTF_states_bz;/* '<S90>/Filter Differentiator TF' */
  real_T Integrator_DSTATE_n;          /* '<S97>/Integrator' */
  real_T UD_DSTATE_o;                  /* '<S2>/UD' */
  real_T FilterDifferentiatorTF_states_a;/* '<S198>/Filter Differentiator TF' */
  real_T Integrator_DSTATE_n3;         /* '<S205>/Integrator' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T YawAngle_set;                 /* '<Root>/YawAngle_set' */
  real_T YawAngle_Now;                 /* '<Root>/YawAngle_Now' */
  real_T YawSpeed_Now;                 /* '<Root>/YawSpeed_Now' */
  real_T YawP_P;                       /* '<Root>/YawP_P' */
  real_T YawP_I;                       /* '<Root>/YawP_I' */
  real_T YawP_D;                       /* '<Root>/YawP_D' */
  real_T YawP_N;                       /* '<Root>/YawP_N' */
  real_T YawS_P;                       /* '<Root>/YawS_P' */
  real_T YawS_I;                       /* '<Root>/YawS_I' */
  real_T YawS_D;                       /* '<Root>/YawS_D' */
  real_T YawS_N;                       /* '<Root>/YawS_N' */
  real_T YawP_MO;                      /* '<Root>/YawP_MO' */
  real_T YawP_LO;                      /* '<Root>/YawP_LO' */
  real_T YawS_MO;                      /* '<Root>/YawS_MO' */
  real_T YawS_LO;                      /* '<Root>/YawS_LO' */
  real_T PihAngle_set;                 /* '<Root>/PihAngle_set' */
  real_T PihAngle_Now;                 /* '<Root>/PihAngle_Now' */
  real_T PihSpeed_Now;                 /* '<Root>/PihSpeed_Now' */
  real_T PihP_P;                       /* '<Root>/PihP_P' */
  real_T PihP_I;                       /* '<Root>/PihP_I' */
  real_T PihP_D;                       /* '<Root>/PihP_D' */
  real_T PihP_N;                       /* '<Root>/PihP_N' */
  real_T PihS_P;                       /* '<Root>/PihS_P' */
  real_T PihS_I;                       /* '<Root>/PihS_I' */
  real_T PihS_D;                       /* '<Root>/PihS_D' */
  real_T PihS_N;                       /* '<Root>/PihS_N' */
  real_T PihP_MO;                      /* '<Root>/PihP_MO' */
  real_T PihP_LO;                      /* '<Root>/PihP_LO' */
  real_T PihS_MO;                      /* '<Root>/PihS_MO' */
  real_T PihS_LO;                      /* '<Root>/PihS_LO' */
  real_T Yaw_Dif_Gain;                 /* '<Root>/Yaw_Dif_Gain' */
  real_T Pih_Dif_Gain;                 /* '<Root>/Pih_Dif_Gain' */
  real_T Weight_Now;                   /* '<Root>/Weight_Now' */
  real_T Weight_Gain;                  /* '<Root>/Weight_Gain' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T YawCurrent;                   /* '<Root>/YawCurrent' */
  real_T PihCurrent;                   /* '<Root>/PihCurrent' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* External function called from main */
extern void PID_SetEventsForThisBaseStep(boolean_T *eventFlags);

/* Model entry point functions */
extern void PID_SetEventsForThisBaseStep(boolean_T *eventFlags);
extern void PID_initialize(void);
extern void PID_step(int_T tid);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Duplicate' : Unused code path elimination
 * Block '<S2>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S105>/Data Type Duplicate' : Unused code path elimination
 * Block '<S105>/Data Type Propagation' : Unused code path elimination
 * Block '<S159>/Data Type Duplicate' : Unused code path elimination
 * Block '<S159>/Data Type Propagation' : Unused code path elimination
 * Block '<S213>/Data Type Duplicate' : Unused code path elimination
 * Block '<S213>/Data Type Propagation' : Unused code path elimination
 * Block '<S36>/Passthrough for tuning' : Eliminate redundant data type conversion
 * Block '<S90>/Passthrough for tuning' : Eliminate redundant data type conversion
 * Block '<S144>/Passthrough for tuning' : Eliminate redundant data type conversion
 * Block '<S198>/Passthrough for tuning' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PID'
 * '<S1>'   : 'PID/Discrete Derivative'
 * '<S2>'   : 'PID/Discrete Derivative1'
 * '<S3>'   : 'PID/PosC'
 * '<S4>'   : 'PID/PosC1'
 * '<S5>'   : 'PID/SpeedC'
 * '<S6>'   : 'PID/SpeedC1'
 * '<S7>'   : 'PID/PosC/Anti-windup'
 * '<S8>'   : 'PID/PosC/D Gain'
 * '<S9>'   : 'PID/PosC/Filter'
 * '<S10>'  : 'PID/PosC/Filter ICs'
 * '<S11>'  : 'PID/PosC/I Gain'
 * '<S12>'  : 'PID/PosC/Ideal P Gain'
 * '<S13>'  : 'PID/PosC/Ideal P Gain Fdbk'
 * '<S14>'  : 'PID/PosC/Integrator'
 * '<S15>'  : 'PID/PosC/Integrator ICs'
 * '<S16>'  : 'PID/PosC/N Copy'
 * '<S17>'  : 'PID/PosC/N Gain'
 * '<S18>'  : 'PID/PosC/P Copy'
 * '<S19>'  : 'PID/PosC/Parallel P Gain'
 * '<S20>'  : 'PID/PosC/Reset Signal'
 * '<S21>'  : 'PID/PosC/Saturation'
 * '<S22>'  : 'PID/PosC/Saturation Fdbk'
 * '<S23>'  : 'PID/PosC/Sum'
 * '<S24>'  : 'PID/PosC/Sum Fdbk'
 * '<S25>'  : 'PID/PosC/Tracking Mode'
 * '<S26>'  : 'PID/PosC/Tracking Mode Sum'
 * '<S27>'  : 'PID/PosC/Tsamp - Integral'
 * '<S28>'  : 'PID/PosC/Tsamp - Ngain'
 * '<S29>'  : 'PID/PosC/postSat Signal'
 * '<S30>'  : 'PID/PosC/preSat Signal'
 * '<S31>'  : 'PID/PosC/Anti-windup/Disc. Clamping Parallel'
 * '<S32>'  : 'PID/PosC/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S33>'  : 'PID/PosC/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S34>'  : 'PID/PosC/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S35>'  : 'PID/PosC/D Gain/External Parameters'
 * '<S36>'  : 'PID/PosC/Filter/Disc. Trapezoidal Filter'
 * '<S37>'  : 'PID/PosC/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S38>'  : 'PID/PosC/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S39>'  : 'PID/PosC/Filter ICs/Internal IC - Filter'
 * '<S40>'  : 'PID/PosC/I Gain/External Parameters'
 * '<S41>'  : 'PID/PosC/Ideal P Gain/Passthrough'
 * '<S42>'  : 'PID/PosC/Ideal P Gain Fdbk/Passthrough'
 * '<S43>'  : 'PID/PosC/Integrator/Discrete'
 * '<S44>'  : 'PID/PosC/Integrator ICs/Internal IC'
 * '<S45>'  : 'PID/PosC/N Copy/External Parameters'
 * '<S46>'  : 'PID/PosC/N Gain/External Parameters'
 * '<S47>'  : 'PID/PosC/P Copy/Disabled'
 * '<S48>'  : 'PID/PosC/Parallel P Gain/External Parameters'
 * '<S49>'  : 'PID/PosC/Reset Signal/Disabled'
 * '<S50>'  : 'PID/PosC/Saturation/External'
 * '<S51>'  : 'PID/PosC/Saturation/External/Saturation Dynamic'
 * '<S52>'  : 'PID/PosC/Saturation Fdbk/Passthrough'
 * '<S53>'  : 'PID/PosC/Sum/Sum_PID'
 * '<S54>'  : 'PID/PosC/Sum Fdbk/Enabled'
 * '<S55>'  : 'PID/PosC/Tracking Mode/Disabled'
 * '<S56>'  : 'PID/PosC/Tracking Mode Sum/Passthrough'
 * '<S57>'  : 'PID/PosC/Tsamp - Integral/Passthrough'
 * '<S58>'  : 'PID/PosC/Tsamp - Ngain/Passthrough'
 * '<S59>'  : 'PID/PosC/postSat Signal/Feedback_Path'
 * '<S60>'  : 'PID/PosC/preSat Signal/Feedback_Path'
 * '<S61>'  : 'PID/PosC1/Anti-windup'
 * '<S62>'  : 'PID/PosC1/D Gain'
 * '<S63>'  : 'PID/PosC1/Filter'
 * '<S64>'  : 'PID/PosC1/Filter ICs'
 * '<S65>'  : 'PID/PosC1/I Gain'
 * '<S66>'  : 'PID/PosC1/Ideal P Gain'
 * '<S67>'  : 'PID/PosC1/Ideal P Gain Fdbk'
 * '<S68>'  : 'PID/PosC1/Integrator'
 * '<S69>'  : 'PID/PosC1/Integrator ICs'
 * '<S70>'  : 'PID/PosC1/N Copy'
 * '<S71>'  : 'PID/PosC1/N Gain'
 * '<S72>'  : 'PID/PosC1/P Copy'
 * '<S73>'  : 'PID/PosC1/Parallel P Gain'
 * '<S74>'  : 'PID/PosC1/Reset Signal'
 * '<S75>'  : 'PID/PosC1/Saturation'
 * '<S76>'  : 'PID/PosC1/Saturation Fdbk'
 * '<S77>'  : 'PID/PosC1/Sum'
 * '<S78>'  : 'PID/PosC1/Sum Fdbk'
 * '<S79>'  : 'PID/PosC1/Tracking Mode'
 * '<S80>'  : 'PID/PosC1/Tracking Mode Sum'
 * '<S81>'  : 'PID/PosC1/Tsamp - Integral'
 * '<S82>'  : 'PID/PosC1/Tsamp - Ngain'
 * '<S83>'  : 'PID/PosC1/postSat Signal'
 * '<S84>'  : 'PID/PosC1/preSat Signal'
 * '<S85>'  : 'PID/PosC1/Anti-windup/Disc. Clamping Parallel'
 * '<S86>'  : 'PID/PosC1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S87>'  : 'PID/PosC1/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S88>'  : 'PID/PosC1/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S89>'  : 'PID/PosC1/D Gain/External Parameters'
 * '<S90>'  : 'PID/PosC1/Filter/Disc. Trapezoidal Filter'
 * '<S91>'  : 'PID/PosC1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S92>'  : 'PID/PosC1/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S93>'  : 'PID/PosC1/Filter ICs/Internal IC - Filter'
 * '<S94>'  : 'PID/PosC1/I Gain/External Parameters'
 * '<S95>'  : 'PID/PosC1/Ideal P Gain/Passthrough'
 * '<S96>'  : 'PID/PosC1/Ideal P Gain Fdbk/Passthrough'
 * '<S97>'  : 'PID/PosC1/Integrator/Discrete'
 * '<S98>'  : 'PID/PosC1/Integrator ICs/Internal IC'
 * '<S99>'  : 'PID/PosC1/N Copy/External Parameters'
 * '<S100>' : 'PID/PosC1/N Gain/External Parameters'
 * '<S101>' : 'PID/PosC1/P Copy/Disabled'
 * '<S102>' : 'PID/PosC1/Parallel P Gain/External Parameters'
 * '<S103>' : 'PID/PosC1/Reset Signal/Disabled'
 * '<S104>' : 'PID/PosC1/Saturation/External'
 * '<S105>' : 'PID/PosC1/Saturation/External/Saturation Dynamic'
 * '<S106>' : 'PID/PosC1/Saturation Fdbk/Passthrough'
 * '<S107>' : 'PID/PosC1/Sum/Sum_PID'
 * '<S108>' : 'PID/PosC1/Sum Fdbk/Enabled'
 * '<S109>' : 'PID/PosC1/Tracking Mode/Disabled'
 * '<S110>' : 'PID/PosC1/Tracking Mode Sum/Passthrough'
 * '<S111>' : 'PID/PosC1/Tsamp - Integral/Passthrough'
 * '<S112>' : 'PID/PosC1/Tsamp - Ngain/Passthrough'
 * '<S113>' : 'PID/PosC1/postSat Signal/Feedback_Path'
 * '<S114>' : 'PID/PosC1/preSat Signal/Feedback_Path'
 * '<S115>' : 'PID/SpeedC/Anti-windup'
 * '<S116>' : 'PID/SpeedC/D Gain'
 * '<S117>' : 'PID/SpeedC/Filter'
 * '<S118>' : 'PID/SpeedC/Filter ICs'
 * '<S119>' : 'PID/SpeedC/I Gain'
 * '<S120>' : 'PID/SpeedC/Ideal P Gain'
 * '<S121>' : 'PID/SpeedC/Ideal P Gain Fdbk'
 * '<S122>' : 'PID/SpeedC/Integrator'
 * '<S123>' : 'PID/SpeedC/Integrator ICs'
 * '<S124>' : 'PID/SpeedC/N Copy'
 * '<S125>' : 'PID/SpeedC/N Gain'
 * '<S126>' : 'PID/SpeedC/P Copy'
 * '<S127>' : 'PID/SpeedC/Parallel P Gain'
 * '<S128>' : 'PID/SpeedC/Reset Signal'
 * '<S129>' : 'PID/SpeedC/Saturation'
 * '<S130>' : 'PID/SpeedC/Saturation Fdbk'
 * '<S131>' : 'PID/SpeedC/Sum'
 * '<S132>' : 'PID/SpeedC/Sum Fdbk'
 * '<S133>' : 'PID/SpeedC/Tracking Mode'
 * '<S134>' : 'PID/SpeedC/Tracking Mode Sum'
 * '<S135>' : 'PID/SpeedC/Tsamp - Integral'
 * '<S136>' : 'PID/SpeedC/Tsamp - Ngain'
 * '<S137>' : 'PID/SpeedC/postSat Signal'
 * '<S138>' : 'PID/SpeedC/preSat Signal'
 * '<S139>' : 'PID/SpeedC/Anti-windup/Disc. Clamping Parallel'
 * '<S140>' : 'PID/SpeedC/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S141>' : 'PID/SpeedC/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S142>' : 'PID/SpeedC/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S143>' : 'PID/SpeedC/D Gain/External Parameters'
 * '<S144>' : 'PID/SpeedC/Filter/Disc. Trapezoidal Filter'
 * '<S145>' : 'PID/SpeedC/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S146>' : 'PID/SpeedC/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S147>' : 'PID/SpeedC/Filter ICs/Internal IC - Filter'
 * '<S148>' : 'PID/SpeedC/I Gain/External Parameters'
 * '<S149>' : 'PID/SpeedC/Ideal P Gain/Passthrough'
 * '<S150>' : 'PID/SpeedC/Ideal P Gain Fdbk/Passthrough'
 * '<S151>' : 'PID/SpeedC/Integrator/Discrete'
 * '<S152>' : 'PID/SpeedC/Integrator ICs/Internal IC'
 * '<S153>' : 'PID/SpeedC/N Copy/External Parameters'
 * '<S154>' : 'PID/SpeedC/N Gain/External Parameters'
 * '<S155>' : 'PID/SpeedC/P Copy/Disabled'
 * '<S156>' : 'PID/SpeedC/Parallel P Gain/External Parameters'
 * '<S157>' : 'PID/SpeedC/Reset Signal/Disabled'
 * '<S158>' : 'PID/SpeedC/Saturation/External'
 * '<S159>' : 'PID/SpeedC/Saturation/External/Saturation Dynamic'
 * '<S160>' : 'PID/SpeedC/Saturation Fdbk/Passthrough'
 * '<S161>' : 'PID/SpeedC/Sum/Sum_PID'
 * '<S162>' : 'PID/SpeedC/Sum Fdbk/Enabled'
 * '<S163>' : 'PID/SpeedC/Tracking Mode/Disabled'
 * '<S164>' : 'PID/SpeedC/Tracking Mode Sum/Passthrough'
 * '<S165>' : 'PID/SpeedC/Tsamp - Integral/Passthrough'
 * '<S166>' : 'PID/SpeedC/Tsamp - Ngain/Passthrough'
 * '<S167>' : 'PID/SpeedC/postSat Signal/Feedback_Path'
 * '<S168>' : 'PID/SpeedC/preSat Signal/Feedback_Path'
 * '<S169>' : 'PID/SpeedC1/Anti-windup'
 * '<S170>' : 'PID/SpeedC1/D Gain'
 * '<S171>' : 'PID/SpeedC1/Filter'
 * '<S172>' : 'PID/SpeedC1/Filter ICs'
 * '<S173>' : 'PID/SpeedC1/I Gain'
 * '<S174>' : 'PID/SpeedC1/Ideal P Gain'
 * '<S175>' : 'PID/SpeedC1/Ideal P Gain Fdbk'
 * '<S176>' : 'PID/SpeedC1/Integrator'
 * '<S177>' : 'PID/SpeedC1/Integrator ICs'
 * '<S178>' : 'PID/SpeedC1/N Copy'
 * '<S179>' : 'PID/SpeedC1/N Gain'
 * '<S180>' : 'PID/SpeedC1/P Copy'
 * '<S181>' : 'PID/SpeedC1/Parallel P Gain'
 * '<S182>' : 'PID/SpeedC1/Reset Signal'
 * '<S183>' : 'PID/SpeedC1/Saturation'
 * '<S184>' : 'PID/SpeedC1/Saturation Fdbk'
 * '<S185>' : 'PID/SpeedC1/Sum'
 * '<S186>' : 'PID/SpeedC1/Sum Fdbk'
 * '<S187>' : 'PID/SpeedC1/Tracking Mode'
 * '<S188>' : 'PID/SpeedC1/Tracking Mode Sum'
 * '<S189>' : 'PID/SpeedC1/Tsamp - Integral'
 * '<S190>' : 'PID/SpeedC1/Tsamp - Ngain'
 * '<S191>' : 'PID/SpeedC1/postSat Signal'
 * '<S192>' : 'PID/SpeedC1/preSat Signal'
 * '<S193>' : 'PID/SpeedC1/Anti-windup/Disc. Clamping Parallel'
 * '<S194>' : 'PID/SpeedC1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S195>' : 'PID/SpeedC1/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S196>' : 'PID/SpeedC1/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S197>' : 'PID/SpeedC1/D Gain/External Parameters'
 * '<S198>' : 'PID/SpeedC1/Filter/Disc. Trapezoidal Filter'
 * '<S199>' : 'PID/SpeedC1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S200>' : 'PID/SpeedC1/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S201>' : 'PID/SpeedC1/Filter ICs/Internal IC - Filter'
 * '<S202>' : 'PID/SpeedC1/I Gain/External Parameters'
 * '<S203>' : 'PID/SpeedC1/Ideal P Gain/Passthrough'
 * '<S204>' : 'PID/SpeedC1/Ideal P Gain Fdbk/Passthrough'
 * '<S205>' : 'PID/SpeedC1/Integrator/Discrete'
 * '<S206>' : 'PID/SpeedC1/Integrator ICs/Internal IC'
 * '<S207>' : 'PID/SpeedC1/N Copy/External Parameters'
 * '<S208>' : 'PID/SpeedC1/N Gain/External Parameters'
 * '<S209>' : 'PID/SpeedC1/P Copy/Disabled'
 * '<S210>' : 'PID/SpeedC1/Parallel P Gain/External Parameters'
 * '<S211>' : 'PID/SpeedC1/Reset Signal/Disabled'
 * '<S212>' : 'PID/SpeedC1/Saturation/External'
 * '<S213>' : 'PID/SpeedC1/Saturation/External/Saturation Dynamic'
 * '<S214>' : 'PID/SpeedC1/Saturation Fdbk/Passthrough'
 * '<S215>' : 'PID/SpeedC1/Sum/Sum_PID'
 * '<S216>' : 'PID/SpeedC1/Sum Fdbk/Enabled'
 * '<S217>' : 'PID/SpeedC1/Tracking Mode/Disabled'
 * '<S218>' : 'PID/SpeedC1/Tracking Mode Sum/Passthrough'
 * '<S219>' : 'PID/SpeedC1/Tsamp - Integral/Passthrough'
 * '<S220>' : 'PID/SpeedC1/Tsamp - Ngain/Passthrough'
 * '<S221>' : 'PID/SpeedC1/postSat Signal/Feedback_Path'
 * '<S222>' : 'PID/SpeedC1/preSat Signal/Feedback_Path'
 */
#ifdef __cplusplus
}
#endif
#endif                                 /* RTW_HEADER_PID_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PID.h
 */
