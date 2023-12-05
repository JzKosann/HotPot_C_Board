/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_mat.c
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

#include "PID_mat.hpp"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_PID_mat PID_mat_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PID_mat PID_mat_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PID_mat PID_mat_Y;

/* Real-time model */
static RT_MODEL_PID_mat PID_mat_M_;
RT_MODEL_PID_mat *const PID_mat_M = &PID_mat_M_;

/* Model step function */
void PID_mat_step(void)
{
  real_T rtb_IProdOut;
  real_T rtb_Sum;
  real_T tmp;
  int8_T tmp_0;
  int8_T tmp_1;
  rtb_IProdOut = PID_mat_U.Tar - PID_mat_U.Now;
  rtb_Sum = rtb_IProdOut * PID_mat_U.Kp + PID_mat_DW.Integrator_DSTATE;
  if (rtb_Sum > PID_mat_U.OUTMAX) {
    PID_mat_Y.Current = PID_mat_U.OUTMAX;
  } else if (rtb_Sum < PID_mat_U.OUTLOW) {
    PID_mat_Y.Current = PID_mat_U.OUTLOW;
  } else {
    PID_mat_Y.Current = rtb_Sum;
  }

  if (rtb_Sum >= PID_mat_U.OUTMAX) {
    tmp = PID_mat_U.OUTMAX;
  } else if (rtb_Sum > PID_mat_U.OUTLOW) {
    tmp = rtb_Sum;
  } else {
    tmp = PID_mat_U.OUTLOW;
  }

  rtb_Sum -= tmp;
  rtb_IProdOut *= PID_mat_U.Ki;
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

  PID_mat_DW.Integrator_DSTATE += 0.001 * rtb_IProdOut;
}

/* Model initialize function */
void PID_mat_initialize(void)
{
  /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PID_mat.c
 */
