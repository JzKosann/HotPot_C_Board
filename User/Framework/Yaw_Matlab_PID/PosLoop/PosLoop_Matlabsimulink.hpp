/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PosLoop_Matlabsimulink.h
 *
 * Code generated for Simulink model 'PosLoop_Matlabsimulink'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Sun Jan  7 23:20:40 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PosLoop_Matlabsimulink_h_
#define RTW_HEADER_PosLoop_Matlabsimulink_h_
#ifndef PosLoop_Matlabsimulink_COMMON_INCLUDES_
#define PosLoop_Matlabsimulink_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                             /* PosLoop_Matlabsimulink_COMMON_INCLUDES_ */

#include "PosLoop_Matlabsimulink_types.h"

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
} DW_PosLoop_Matlabsimulink;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Pos_Tar;                      /* '<Root>/Pos_Tar' */
  real_T Pos_now;                      /* '<Root>/Pos_now' */
  real_T Pos_Kp;                       /* '<Root>/Pos_Kp' */
  real_T Pos_Ki;                       /* '<Root>/Pos_Ki' */
  real_T Pos_Kd;                       /* '<Root>/Pos_Kd' */
  real_T Pos_Kn;                       /* '<Root>/Pos_Kn' */
  real_T Pos_OUTMAX;                   /* '<Root>/Pos_OUTMAX' */
  real_T Pos_OUTLOW;                   /* '<Root>/Pos_OUTLOW' */
} ExtU_PosLoop_Matlabsimulink;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Pos_Output;                   /* '<Root>/Pos_Output' */
} ExtY_PosLoop_Matlabsimulink;

/* Real-time Model Data Structure */
struct tag_RTM_PosLoop_Matlabsimulink {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_Y;

/* Model entry point functions */
extern void PosLoop_Matlabsimulink_initialize(void);
extern void PosLoop_Matlabsimulink_step(void);
extern void PosLoop_Matlabsimulink_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PosLoop_Matlabsimulink *const PosLoop_Matlabsimulink_M;

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
 * '<Root>' : 'PosLoop_Matlabsimulink'
 * '<S1>'   : 'PosLoop_Matlabsimulink/PID Controller'
 * '<S2>'   : 'PosLoop_Matlabsimulink/PID Controller/Anti-windup'
 * '<S3>'   : 'PosLoop_Matlabsimulink/PID Controller/D Gain'
 * '<S4>'   : 'PosLoop_Matlabsimulink/PID Controller/Filter'
 * '<S5>'   : 'PosLoop_Matlabsimulink/PID Controller/Filter ICs'
 * '<S6>'   : 'PosLoop_Matlabsimulink/PID Controller/I Gain'
 * '<S7>'   : 'PosLoop_Matlabsimulink/PID Controller/Ideal P Gain'
 * '<S8>'   : 'PosLoop_Matlabsimulink/PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'PosLoop_Matlabsimulink/PID Controller/Integrator'
 * '<S10>'  : 'PosLoop_Matlabsimulink/PID Controller/Integrator ICs'
 * '<S11>'  : 'PosLoop_Matlabsimulink/PID Controller/N Copy'
 * '<S12>'  : 'PosLoop_Matlabsimulink/PID Controller/N Gain'
 * '<S13>'  : 'PosLoop_Matlabsimulink/PID Controller/P Copy'
 * '<S14>'  : 'PosLoop_Matlabsimulink/PID Controller/Parallel P Gain'
 * '<S15>'  : 'PosLoop_Matlabsimulink/PID Controller/Reset Signal'
 * '<S16>'  : 'PosLoop_Matlabsimulink/PID Controller/Saturation'
 * '<S17>'  : 'PosLoop_Matlabsimulink/PID Controller/Saturation Fdbk'
 * '<S18>'  : 'PosLoop_Matlabsimulink/PID Controller/Sum'
 * '<S19>'  : 'PosLoop_Matlabsimulink/PID Controller/Sum Fdbk'
 * '<S20>'  : 'PosLoop_Matlabsimulink/PID Controller/Tracking Mode'
 * '<S21>'  : 'PosLoop_Matlabsimulink/PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'PosLoop_Matlabsimulink/PID Controller/Tsamp - Integral'
 * '<S23>'  : 'PosLoop_Matlabsimulink/PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'PosLoop_Matlabsimulink/PID Controller/postSat Signal'
 * '<S25>'  : 'PosLoop_Matlabsimulink/PID Controller/preSat Signal'
 * '<S26>'  : 'PosLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'PosLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'PosLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S29>'  : 'PosLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S30>'  : 'PosLoop_Matlabsimulink/PID Controller/D Gain/External Parameters'
 * '<S31>'  : 'PosLoop_Matlabsimulink/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'PosLoop_Matlabsimulink/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'PosLoop_Matlabsimulink/PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'PosLoop_Matlabsimulink/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'PosLoop_Matlabsimulink/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'PosLoop_Matlabsimulink/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'PosLoop_Matlabsimulink/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'PosLoop_Matlabsimulink/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'PosLoop_Matlabsimulink/PID Controller/N Gain/External Parameters'
 * '<S40>'  : 'PosLoop_Matlabsimulink/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'PosLoop_Matlabsimulink/PID Controller/Parallel P Gain/External Parameters'
 * '<S42>'  : 'PosLoop_Matlabsimulink/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'PosLoop_Matlabsimulink/PID Controller/Saturation/External'
 * '<S44>'  : 'PosLoop_Matlabsimulink/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S45>'  : 'PosLoop_Matlabsimulink/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'PosLoop_Matlabsimulink/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'PosLoop_Matlabsimulink/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'PosLoop_Matlabsimulink/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'PosLoop_Matlabsimulink/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'PosLoop_Matlabsimulink/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'PosLoop_Matlabsimulink/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'PosLoop_Matlabsimulink/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'PosLoop_Matlabsimulink/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                /* RTW_HEADER_PosLoop_Matlabsimulink_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
