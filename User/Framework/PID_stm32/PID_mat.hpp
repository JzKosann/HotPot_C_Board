/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_mat.h
 *
 * Code generated for Simulink model :PID_mat.
 *
 * Model version      : 1.1
 * Simulink Coder version    : 23.2 (R2023b) 01-Aug-2023
 * TLC version       : 23.2 (Nov 08 2023)
 * C/C++ source code generated on  : Wed Nov  8 15:52:54 2023
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

#ifndef RTW_HEADER_PID_mat_h_
#define RTW_HEADER_PID_mat_h_
#ifndef PID_mat_COMMON_INCLUDES_
#define PID_mat_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PID_mat_COMMON_INCLUDES_ */

#include "PID_mat_types.h"

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
} DW_PID_mat;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T OUTMAX;                       /* '<Root>/OUTMAX' */
  real_T OUTLOW;                       /* '<Root>/OUTLOW' */
  real_T Ki;                           /* '<Root>/Ki' */
  real_T Kp;                           /* '<Root>/Kp' */
  real_T Tar;                          /* '<Root>/Tar' */
  real_T Now;                          /* '<Root>/Now' */
} ExtU_PID_mat;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Current;                      /* '<Root>/Current' */
} ExtY_PID_mat;

/* Real-time Model Data Structure */
struct tag_RTM_PID_mat {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PID_mat PID_mat_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PID_mat PID_mat_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PID_mat PID_mat_Y;

/* Model entry point functions */
extern void PID_mat_initialize(void);
extern void PID_mat_step(void);

/* Real-time Model object */
extern RT_MODEL_PID_mat *const PID_mat_M;

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
 * '<Root>' : 'PID_mat'
 * '<S1>'   : 'PID_mat/PID Controller'
 * '<S2>'   : 'PID_mat/PID Controller/Anti-windup'
 * '<S3>'   : 'PID_mat/PID Controller/D Gain'
 * '<S4>'   : 'PID_mat/PID Controller/Filter'
 * '<S5>'   : 'PID_mat/PID Controller/Filter ICs'
 * '<S6>'   : 'PID_mat/PID Controller/I Gain'
 * '<S7>'   : 'PID_mat/PID Controller/Ideal P Gain'
 * '<S8>'   : 'PID_mat/PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'PID_mat/PID Controller/Integrator'
 * '<S10>'  : 'PID_mat/PID Controller/Integrator ICs'
 * '<S11>'  : 'PID_mat/PID Controller/N Copy'
 * '<S12>'  : 'PID_mat/PID Controller/N Gain'
 * '<S13>'  : 'PID_mat/PID Controller/P Copy'
 * '<S14>'  : 'PID_mat/PID Controller/Parallel P Gain'
 * '<S15>'  : 'PID_mat/PID Controller/Reset Signal'
 * '<S16>'  : 'PID_mat/PID Controller/Saturation'
 * '<S17>'  : 'PID_mat/PID Controller/Saturation Fdbk'
 * '<S18>'  : 'PID_mat/PID Controller/Sum'
 * '<S19>'  : 'PID_mat/PID Controller/Sum Fdbk'
 * '<S20>'  : 'PID_mat/PID Controller/Tracking Mode'
 * '<S21>'  : 'PID_mat/PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'PID_mat/PID Controller/Tsamp - Integral'
 * '<S23>'  : 'PID_mat/PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'PID_mat/PID Controller/postSat Signal'
 * '<S25>'  : 'PID_mat/PID Controller/preSat Signal'
 * '<S26>'  : 'PID_mat/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'PID_mat/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'PID_mat/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S29>'  : 'PID_mat/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S30>'  : 'PID_mat/PID Controller/D Gain/Disabled'
 * '<S31>'  : 'PID_mat/PID Controller/Filter/Disabled'
 * '<S32>'  : 'PID_mat/PID Controller/Filter ICs/Disabled'
 * '<S33>'  : 'PID_mat/PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'PID_mat/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'PID_mat/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'PID_mat/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'PID_mat/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'PID_mat/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S39>'  : 'PID_mat/PID Controller/N Gain/Disabled'
 * '<S40>'  : 'PID_mat/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'PID_mat/PID Controller/Parallel P Gain/External Parameters'
 * '<S42>'  : 'PID_mat/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'PID_mat/PID Controller/Saturation/External'
 * '<S44>'  : 'PID_mat/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S45>'  : 'PID_mat/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'PID_mat/PID Controller/Sum/Sum_PI'
 * '<S47>'  : 'PID_mat/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'PID_mat/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'PID_mat/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'PID_mat/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'PID_mat/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'PID_mat/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'PID_mat/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_PID_mat_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PID_mat.h
 */
