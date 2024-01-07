/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PosLoop_Matlabsimulink.c
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

#include "PosLoop_Matlabsimulink.hpp"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_Y;

/* Real-time model */
static RT_MODEL_PosLoop_Matlabsimulink PosLoop_Matlabsimulink_M_;
RT_MODEL_PosLoop_Matlabsimulink *const PosLoop_Matlabsimulink_M =
  &PosLoop_Matlabsimulink_M_;

/* Model step function */
void PosLoop_Matlabsimulink_step(void)
{
  real_T rtb_IProdOut;
  real_T rtb_NProdOut;
  real_T rtb_Sum;
  real_T tmp;
  int8_T tmp_0;
  int8_T tmp_1;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/Pos_Tar'
   *  Inport: '<Root>/Pos_now'
   */
  rtb_IProdOut = PosLoop_Matlabsimulink_U.Pos_Tar -
    PosLoop_Matlabsimulink_U.Pos_now;

  /* Product: '<S39>/NProd Out' incorporates:
   *  DiscreteIntegrator: '<S31>/Filter'
   *  Inport: '<Root>/Pos_Kd'
   *  Inport: '<Root>/Pos_Kn'
   *  Product: '<S30>/DProd Out'
   *  Sum: '<S31>/SumD'
   */
  rtb_NProdOut = (rtb_IProdOut * PosLoop_Matlabsimulink_U.Pos_Kd -
                  PosLoop_Matlabsimulink_DW.Filter_DSTATE) *
    PosLoop_Matlabsimulink_U.Pos_Kn;

  /* Sum: '<S46>/Sum' incorporates:
   *  DiscreteIntegrator: '<S36>/Integrator'
   *  Inport: '<Root>/Pos_Kp'
   *  Product: '<S41>/PProd Out'
   */
  rtb_Sum = (rtb_IProdOut * PosLoop_Matlabsimulink_U.Pos_Kp +
             PosLoop_Matlabsimulink_DW.Integrator_DSTATE) + rtb_NProdOut;

  /* Switch: '<S44>/Switch2' incorporates:
   *  Inport: '<Root>/Pos_OUTLOW'
   *  Inport: '<Root>/Pos_OUTMAX'
   *  RelationalOperator: '<S44>/LowerRelop1'
   *  RelationalOperator: '<S44>/UpperRelop'
   *  Switch: '<S44>/Switch'
   */
  if (rtb_Sum > PosLoop_Matlabsimulink_U.Pos_OUTMAX) {
    /* Outport: '<Root>/Pos_Output' */
    PosLoop_Matlabsimulink_Y.Pos_Output = PosLoop_Matlabsimulink_U.Pos_OUTMAX;
  } else if (rtb_Sum < PosLoop_Matlabsimulink_U.Pos_OUTLOW) {
    /* Switch: '<S44>/Switch' incorporates:
     *  Inport: '<Root>/Pos_OUTLOW'
     *  Outport: '<Root>/Pos_Output'
     */
    PosLoop_Matlabsimulink_Y.Pos_Output = PosLoop_Matlabsimulink_U.Pos_OUTLOW;
  } else {
    /* Outport: '<Root>/Pos_Output' incorporates:
     *  Switch: '<S44>/Switch'
     */
    PosLoop_Matlabsimulink_Y.Pos_Output = rtb_Sum;
  }

  /* End of Switch: '<S44>/Switch2' */

  /* Switch: '<S29>/Switch' incorporates:
   *  Inport: '<Root>/Pos_OUTLOW'
   *  Inport: '<Root>/Pos_OUTMAX'
   *  RelationalOperator: '<S29>/u_GTE_up'
   *  RelationalOperator: '<S29>/u_GT_lo'
   *  Switch: '<S29>/Switch1'
   */
  if (rtb_Sum >= PosLoop_Matlabsimulink_U.Pos_OUTMAX) {
    tmp = PosLoop_Matlabsimulink_U.Pos_OUTMAX;
  } else if (rtb_Sum > PosLoop_Matlabsimulink_U.Pos_OUTLOW) {
    /* Switch: '<S29>/Switch1' */
    tmp = rtb_Sum;
  } else {
    tmp = PosLoop_Matlabsimulink_U.Pos_OUTLOW;
  }

  /* Sum: '<S29>/Diff' incorporates:
   *  Switch: '<S29>/Switch'
   */
  rtb_Sum -= tmp;

  /* Product: '<S33>/IProd Out' incorporates:
   *  Inport: '<Root>/Pos_Ki'
   */
  rtb_IProdOut *= PosLoop_Matlabsimulink_U.Pos_Ki;

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant'
   *  Constant: '<S26>/Constant2'
   *  RelationalOperator: '<S26>/fix for DT propagation issue'
   */
  if (rtb_Sum > 0.0) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S26>/Switch2' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant3'
   *  Constant: '<S26>/Constant4'
   *  RelationalOperator: '<S26>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0) {
    tmp_1 = 1;
  } else {
    tmp_1 = -1;
  }

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant1'
   *  Logic: '<S26>/AND3'
   *  RelationalOperator: '<S26>/Equal1'
   *  RelationalOperator: '<S26>/Relational Operator'
   *  Switch: '<S26>/Switch1'
   *  Switch: '<S26>/Switch2'
   */
  if ((rtb_Sum != 0.0) && (tmp_0 == tmp_1)) {
    rtb_IProdOut = 0.0;
  }

  /* Update for DiscreteIntegrator: '<S36>/Integrator' incorporates:
   *  Switch: '<S26>/Switch'
   */
  PosLoop_Matlabsimulink_DW.Integrator_DSTATE += 0.002 * rtb_IProdOut;

  /* Update for DiscreteIntegrator: '<S31>/Filter' */
  PosLoop_Matlabsimulink_DW.Filter_DSTATE += 0.002 * rtb_NProdOut;
}

/* Model initialize function */
void PosLoop_Matlabsimulink_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void PosLoop_Matlabsimulink_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
