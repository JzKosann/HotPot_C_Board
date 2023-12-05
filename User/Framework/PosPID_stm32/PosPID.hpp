/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PosPID.h
 *
 * Code generated for Simulink model :PosPID.
 *
 * Model version      : 1.2
 * Simulink Coder version    : 23.2 (R2023b) 01-Aug-2023
 * TLC version       : 23.2 (Nov 08 2023)
 * C/C++ source code generated on  : Wed Nov  8 20:43:21 2023
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
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

#ifndef RTW_HEADER_PosPID_h_
#define RTW_HEADER_PosPID_h_
#ifndef PosPID_COMMON_INCLUDES_
#define PosPID_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PosPID_COMMON_INCLUDES_ */

#include "PosPID_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S36>/Integrator' */
  real_T Filter_DSTATE;                /* '<S31>/Filter' */
} DW_PosPID;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Kp;                           /* '<Root>/Kp' */
  real_T Kd;                           /* '<Root>/Kd' */
  real_T N;                            /* '<Root>/N' */
  real_T OUTMAX;                       /* '<Root>/OUTMAX' */
  real_T OUTLOW;                       /* '<Root>/OUTLOW' */
  real_T Tar;                          /* '<Root>/Tar' */
  real_T Now;                          /* '<Root>/Now' */
  real_T Ki;                           /* '<Root>/Ki' */
} ExtU_PosPID;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T spd;                          /* '<Root>/spd' */
} ExtY_PosPID;

/* Real-time Model Data Structure */
struct tag_RTM_PosPID {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PosPID PosPID_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PosPID PosPID_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PosPID PosPID_Y;

/* Model entry point functions */
extern void PosPID_initialize(void);
extern void PosPID_step(void);

/* Real-time Model object */
extern RT_MODEL_PosPID *const PosPID_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'PosPID'
 * '<S1>'   : 'PosPID/PID Controller'
 * '<S2>'   : 'PosPID/PID Controller/Anti-windup'
 * '<S3>'   : 'PosPID/PID Controller/D Gain'
 * '<S4>'   : 'PosPID/PID Controller/Filter'
 * '<S5>'   : 'PosPID/PID Controller/Filter ICs'
 * '<S6>'   : 'PosPID/PID Controller/I Gain'
 * '<S7>'   : 'PosPID/PID Controller/Ideal P Gain'
 * '<S8>'   : 'PosPID/PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'PosPID/PID Controller/Integrator'
 * '<S10>'  : 'PosPID/PID Controller/Integrator ICs'
 * '<S11>'  : 'PosPID/PID Controller/N Copy'
 * '<S12>'  : 'PosPID/PID Controller/N Gain'
 * '<S13>'  : 'PosPID/PID Controller/P Copy'
 * '<S14>'  : 'PosPID/PID Controller/Parallel P Gain'
 * '<S15>'  : 'PosPID/PID Controller/Reset Signal'
 * '<S16>'  : 'PosPID/PID Controller/Saturation'
 * '<S17>'  : 'PosPID/PID Controller/Saturation Fdbk'
 * '<S18>'  : 'PosPID/PID Controller/Sum'
 * '<S19>'  : 'PosPID/PID Controller/Sum Fdbk'
 * '<S20>'  : 'PosPID/PID Controller/Tracking Mode'
 * '<S21>'  : 'PosPID/PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'PosPID/PID Controller/Tsamp - Integral'
 * '<S23>'  : 'PosPID/PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'PosPID/PID Controller/postSat Signal'
 * '<S25>'  : 'PosPID/PID Controller/preSat Signal'
 * '<S26>'  : 'PosPID/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'PosPID/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'PosPID/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S29>'  : 'PosPID/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S30>'  : 'PosPID/PID Controller/D Gain/External Parameters'
 * '<S31>'  : 'PosPID/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'PosPID/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'PosPID/PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'PosPID/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'PosPID/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'PosPID/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'PosPID/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'PosPID/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'PosPID/PID Controller/N Gain/External Parameters'
 * '<S40>'  : 'PosPID/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'PosPID/PID Controller/Parallel P Gain/External Parameters'
 * '<S42>'  : 'PosPID/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'PosPID/PID Controller/Saturation/External'
 * '<S44>'  : 'PosPID/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S45>'  : 'PosPID/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'PosPID/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'PosPID/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'PosPID/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'PosPID/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'PosPID/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'PosPID/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'PosPID/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'PosPID/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_PosPID_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PosPID.h
 */
