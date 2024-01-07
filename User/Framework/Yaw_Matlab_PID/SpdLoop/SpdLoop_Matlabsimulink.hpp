/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: SpdLoop_Matlabsimulink.h
 *
 * Code generated for Simulink model 'SpdLoop_Matlabsimulink'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Mon Jan  8 00:32:48 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_SpdLoop_Matlabsimulink_h_
#define RTW_HEADER_SpdLoop_Matlabsimulink_h_
#ifndef SpdLoop_Matlabsimulink_COMMON_INCLUDES_
#define SpdLoop_Matlabsimulink_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                             /* SpdLoop_Matlabsimulink_COMMON_INCLUDES_ */

#include "SpdLoop_Matlabsimulink_types.h"

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
} DW_SpdLoop_Matlabsimulink;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Spd_Tar;                      /* '<Root>/Spd_Tar' */
  real_T Spd_now;                      /* '<Root>/Spd_now' */
  real_T Spd_Kp;                       /* '<Root>/Spd_Kp' */
  real_T Spd_Ki;                       /* '<Root>/Spd_Ki' */
  real_T Spd_OUTMAX;                   /* '<Root>/Spd_OUTMAX' */
  real_T Spd_OUTLOW;                   /* '<Root>/Spd_OUTLOW' */
  real_T Spd_Kd;                       /* '<Root>/Spd_Kd' */
  real_T Spd_Kn;                       /* '<Root>/Spd_Kn' */
} ExtU_SpdLoop_Matlabsimulink;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Spd_Output;                   /* '<Root>/Spd_Output' */
} ExtY_SpdLoop_Matlabsimulink;

/* Real-time Model Data Structure */
struct tag_RTM_SpdLoop_Matlabsimulink {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_SpdLoop_Matlabsimulink SpdLoop_Matlabsimulink_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_SpdLoop_Matlabsimulink SpdLoop_Matlabsimulink_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_SpdLoop_Matlabsimulink SpdLoop_Matlabsimulink_Y;

/* Model entry point functions */
extern void SpdLoop_Matlabsimulink_initialize(void);
extern void SpdLoop_Matlabsimulink_step(void);
extern void SpdLoop_Matlabsimulink_terminate(void);

/* Real-time Model object */
extern RT_MODEL_SpdLoop_Matlabsimulink *const SpdLoop_Matlabsimulink_M;

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
 * '<Root>' : 'SpdLoop_Matlabsimulink'
 * '<S1>'   : 'SpdLoop_Matlabsimulink/PID Controller'
 * '<S2>'   : 'SpdLoop_Matlabsimulink/PID Controller/Anti-windup'
 * '<S3>'   : 'SpdLoop_Matlabsimulink/PID Controller/D Gain'
 * '<S4>'   : 'SpdLoop_Matlabsimulink/PID Controller/Filter'
 * '<S5>'   : 'SpdLoop_Matlabsimulink/PID Controller/Filter ICs'
 * '<S6>'   : 'SpdLoop_Matlabsimulink/PID Controller/I Gain'
 * '<S7>'   : 'SpdLoop_Matlabsimulink/PID Controller/Ideal P Gain'
 * '<S8>'   : 'SpdLoop_Matlabsimulink/PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'SpdLoop_Matlabsimulink/PID Controller/Integrator'
 * '<S10>'  : 'SpdLoop_Matlabsimulink/PID Controller/Integrator ICs'
 * '<S11>'  : 'SpdLoop_Matlabsimulink/PID Controller/N Copy'
 * '<S12>'  : 'SpdLoop_Matlabsimulink/PID Controller/N Gain'
 * '<S13>'  : 'SpdLoop_Matlabsimulink/PID Controller/P Copy'
 * '<S14>'  : 'SpdLoop_Matlabsimulink/PID Controller/Parallel P Gain'
 * '<S15>'  : 'SpdLoop_Matlabsimulink/PID Controller/Reset Signal'
 * '<S16>'  : 'SpdLoop_Matlabsimulink/PID Controller/Saturation'
 * '<S17>'  : 'SpdLoop_Matlabsimulink/PID Controller/Saturation Fdbk'
 * '<S18>'  : 'SpdLoop_Matlabsimulink/PID Controller/Sum'
 * '<S19>'  : 'SpdLoop_Matlabsimulink/PID Controller/Sum Fdbk'
 * '<S20>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tracking Mode'
 * '<S21>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tsamp - Integral'
 * '<S23>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'SpdLoop_Matlabsimulink/PID Controller/postSat Signal'
 * '<S25>'  : 'SpdLoop_Matlabsimulink/PID Controller/preSat Signal'
 * '<S26>'  : 'SpdLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'SpdLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'SpdLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S29>'  : 'SpdLoop_Matlabsimulink/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S30>'  : 'SpdLoop_Matlabsimulink/PID Controller/D Gain/External Parameters'
 * '<S31>'  : 'SpdLoop_Matlabsimulink/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'SpdLoop_Matlabsimulink/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'SpdLoop_Matlabsimulink/PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'SpdLoop_Matlabsimulink/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'SpdLoop_Matlabsimulink/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'SpdLoop_Matlabsimulink/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'SpdLoop_Matlabsimulink/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'SpdLoop_Matlabsimulink/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'SpdLoop_Matlabsimulink/PID Controller/N Gain/External Parameters'
 * '<S40>'  : 'SpdLoop_Matlabsimulink/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'SpdLoop_Matlabsimulink/PID Controller/Parallel P Gain/External Parameters'
 * '<S42>'  : 'SpdLoop_Matlabsimulink/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'SpdLoop_Matlabsimulink/PID Controller/Saturation/External'
 * '<S44>'  : 'SpdLoop_Matlabsimulink/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S45>'  : 'SpdLoop_Matlabsimulink/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'SpdLoop_Matlabsimulink/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'SpdLoop_Matlabsimulink/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'SpdLoop_Matlabsimulink/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'SpdLoop_Matlabsimulink/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'SpdLoop_Matlabsimulink/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                /* RTW_HEADER_SpdLoop_Matlabsimulink_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
