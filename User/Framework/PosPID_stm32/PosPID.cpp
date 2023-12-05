/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PosPID.c
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

#include "PosPID.hpp"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_PosPID PosPID_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PosPID PosPID_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PosPID PosPID_Y;

/* Real-time model */
static RT_MODEL_PosPID PosPID_M_;
RT_MODEL_PosPID *const PosPID_M = &PosPID_M_;

/* Model step function */
void PosPID_step(void)
{
  real_T rtb_IProdOut;
  real_T rtb_NProdOut;
  real_T rtb_Sum;
  real_T tmp;
  int8_T tmp_0;
  int8_T tmp_1;
  rtb_IProdOut = PosPID_U.Tar - PosPID_U.Now;
  rtb_NProdOut = (rtb_IProdOut * PosPID_U.Kd - PosPID_DW.Filter_DSTATE) *
    PosPID_U.N;
  rtb_Sum = (rtb_IProdOut * PosPID_U.Kp + PosPID_DW.Integrator_DSTATE) +
    rtb_NProdOut;
  if (rtb_Sum > PosPID_U.OUTMAX) {
    PosPID_Y.spd = PosPID_U.OUTMAX;
  } else if (rtb_Sum < PosPID_U.OUTLOW) {
    PosPID_Y.spd = PosPID_U.OUTLOW;
  } else {
    PosPID_Y.spd = rtb_Sum;
  }

  if (rtb_Sum >= PosPID_U.OUTMAX) {
    tmp = PosPID_U.OUTMAX;
  } else if (rtb_Sum > PosPID_U.OUTLOW) {
    tmp = rtb_Sum;
  } else {
    tmp = PosPID_U.OUTLOW;
  }

  rtb_Sum -= tmp;
  rtb_IProdOut *= PosPID_U.Ki;
  if (rtb_Sum > 0.0) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  if (rtb_IProdOut > 0.0) {
    tmp_1 = 1;
  } else {
    tmp_1 = -1;
  }

  if ((rtb_Sum != 0.0) && (tmp_0 == tmp_1)) {
    rtb_IProdOut = 0.0;
  }

  PosPID_DW.Integrator_DSTATE += 0.001 * rtb_IProdOut;
  PosPID_DW.Filter_DSTATE += 0.001 * rtb_NProdOut;
}

/* Model initialize function */
void PosPID_initialize(void)
{
  /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PosPID.c
 */
