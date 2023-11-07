/*
 * File: PID.c
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

#include "PIDC.h"
#include "debugc.h"
#define NumBitsPerChar                 8U

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern void PID_step0(void);
extern void PID_step1(void);
static void rate_monotonic_scheduler(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0 } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to "remember" which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void PID_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(rtM, 1));
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.005s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function for TID0 */
void PID_step0(void)                   /* Sample time: [0.001s, 0.0s] */
{
  {                                    /* Sample time: [0.001s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void PID_step1(void)                   /* Sample time: [0.005s, 0.0s] */
{
  real_T FilterDifferentiatorTF_tmp;
  real_T FilterDifferentiatorTF_tmp_b;
  real_T FilterDifferentiatorTF_tmp_i;
  real_T FilterDifferentiatorTF_tmp_j;
  real_T Integrator;
  real_T Integrator_b;
  real_T Integrator_f;
  real_T rtb_DProdOut;
  real_T rtb_DProdOut_g;
  real_T rtb_DProdOut_j;
  real_T rtb_DProdOut_jq;
  real_T rtb_NProdOut;
  real_T rtb_PProdOut;
  real_T rtb_SignPreIntegrator;
  real_T rtb_Sum;
  real_T rtb_Switch2;
  real_T rtb_TSamp_c;
  boolean_T rtb_NotEqual;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/YawAngle_Now'
   *  Inport: '<Root>/YawAngle_set'
   */
  rtb_SignPreIntegrator = rtU.YawAngle_set - rtU.YawAngle_Now;

  /* Product: '<S48>/PProd Out' incorporates:
   *  Inport: '<Root>/YawP_P'
   */
  rtb_NProdOut = rtb_SignPreIntegrator * rtU.YawP_P;

  /* SampleTimeMath: '<S38>/Tsamp' incorporates:
   *  Inport: '<Root>/YawP_N'
   *
   * About '<S38>/Tsamp':
   *  y = u * K where K = ( w * Ts )
   */
  rtb_PProdOut = rtU.YawP_N * 0.0025;

  /* Math: '<S36>/Reciprocal' incorporates:
   *  Constant: '<S36>/Constant'
   *  Sum: '<S36>/SumDen'
   *
   * About '<S36>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Switch2 = 1.0 / (rtb_PProdOut + 1.0);

  /* DiscreteTransferFcn: '<S36>/Filter Differentiator TF' incorporates:
   *  Constant: '<S36>/Constant'
   *  Inport: '<Root>/YawP_D'
   *  Product: '<S35>/DProd Out'
   *  Product: '<S36>/Divide'
   *  Sum: '<S36>/SumNum'
   */
  FilterDifferentiatorTF_tmp = rtb_SignPreIntegrator * rtU.YawP_D -
    (rtb_PProdOut - 1.0) * rtb_Switch2 * rtDW.FilterDifferentiatorTF_states;

  /* Product: '<S46>/NProd Out' incorporates:
   *  DiscreteTransferFcn: '<S36>/Filter Differentiator TF'
   *  Inport: '<Root>/YawP_N'
   *  Product: '<S36>/DenCoefOut'
   */
  rtb_Switch2 = (FilterDifferentiatorTF_tmp +
                 -rtDW.FilterDifferentiatorTF_states) * rtb_Switch2 * rtU.YawP_N;

  /* Sum: '<S54>/Sum Fdbk' */
  rtb_DProdOut = (rtb_NProdOut + rtDW.Integrator_DSTATE) + rtb_Switch2;

  /* Switch: '<S34>/Switch' incorporates:
   *  Inport: '<Root>/YawP_LO'
   *  Inport: '<Root>/YawP_MO'
   *  RelationalOperator: '<S34>/u_GTE_up'
   *  RelationalOperator: '<S34>/u_GT_lo'
   *  Switch: '<S34>/Switch1'
   */
  if (rtb_DProdOut >= rtU.YawP_MO) {
    rtb_PProdOut = rtU.YawP_MO;
  } else if (rtb_DProdOut > rtU.YawP_LO) {
    /* Switch: '<S34>/Switch1' */
    rtb_PProdOut = rtb_DProdOut;
  } else {
    rtb_PProdOut = rtU.YawP_LO;
  }

  /* End of Switch: '<S34>/Switch' */

  /* Sum: '<S34>/Diff' */
  rtb_PProdOut = rtb_DProdOut - rtb_PProdOut;

  /* RelationalOperator: '<S31>/NotEqual' incorporates:
   *  Gain: '<S31>/ZeroGain'
   */
  rtb_NotEqual = (0.0 * rtb_DProdOut != rtb_PProdOut);

  /* Signum: '<S31>/SignPreSat' */
  if (rtb_PProdOut < 0.0) {
    rtb_PProdOut = -1.0;
  } else if (rtb_PProdOut > 0.0) {
    rtb_PProdOut = 1.0;
  } else if (rtb_PProdOut == 0.0) {
    rtb_PProdOut = 0.0;
  } else {
    rtb_PProdOut = (rtNaN);
  }

  /* End of Signum: '<S31>/SignPreSat' */

  /* Product: '<S40>/IProd Out' incorporates:
   *  Inport: '<Root>/YawP_I'
   */
  rtb_SignPreIntegrator *= rtU.YawP_I;

  /* Signum: '<S31>/SignPreIntegrator' */
  if (rtb_SignPreIntegrator < 0.0) {
    rtb_DProdOut = -1.0;
  } else if (rtb_SignPreIntegrator > 0.0) {
    rtb_DProdOut = 1.0;
  } else if (rtb_SignPreIntegrator == 0.0) {
    rtb_DProdOut = 0.0;
  } else {
    rtb_DProdOut = (rtNaN);
  }

  /* End of Signum: '<S31>/SignPreIntegrator' */

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Constant1'
   *  DataTypeConversion: '<S31>/DataTypeConv1'
   *  DataTypeConversion: '<S31>/DataTypeConv2'
   *  Logic: '<S31>/AND3'
   *  RelationalOperator: '<S31>/Equal1'
   */
  if (rtb_NotEqual && ((int8_T)rtb_PProdOut == (int8_T)rtb_DProdOut)) {
    rtb_DProdOut = 0.0;
  } else {
    rtb_DProdOut = rtb_SignPreIntegrator;
  }

  /* End of Switch: '<S31>/Switch' */

  /* DiscreteIntegrator: '<S43>/Integrator' */
  Integrator = 0.0025 * rtb_DProdOut + rtDW.Integrator_DSTATE;

  /* Sum: '<S53>/Sum' */
  rtb_Sum = (rtb_NProdOut + Integrator) + rtb_Switch2;

  /* Switch: '<S51>/Switch2' incorporates:
   *  Inport: '<Root>/YawP_LO'
   *  Inport: '<Root>/YawP_MO'
   *  RelationalOperator: '<S51>/LowerRelop1'
   *  RelationalOperator: '<S51>/UpperRelop'
   *  Switch: '<S51>/Switch'
   */
  if (rtb_Sum > rtU.YawP_MO) {
    rtb_Switch2 = rtU.YawP_MO;
  } else if (rtb_Sum < rtU.YawP_LO) {
    /* Switch: '<S51>/Switch' incorporates:
     *  Inport: '<Root>/YawP_LO'
     */
    rtb_Switch2 = rtU.YawP_LO;
  } else {
    rtb_Switch2 = rtb_Sum;
  }

  /* End of Switch: '<S51>/Switch2' */

  /* SampleTimeMath: '<S1>/TSamp' incorporates:
   *  Inport: '<Root>/YawAngle_set'
   *  Inport: '<Root>/Yaw_Dif_Gain'
   *  Product: '<Root>/Product'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Sum = rtU.Yaw_Dif_Gain * rtU.YawAngle_set * 200.0;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Inport: '<Root>/YawSpeed_Now'
   *  Sum: '<S1>/Diff'
   *  UnitDelay: '<S1>/UD'
   *
   * Block description for '<S1>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Switch2 = (rtb_Switch2 - rtU.YawSpeed_Now) + (rtb_Sum - rtDW.UD_DSTATE);
//  rtb_Switch2 = 60.0f - rtU.YawSpeed_Now;
  /* Product: '<S156>/PProd Out' incorporates:
   *  Inport: '<Root>/YawS_P'
   */
  rtb_PProdOut = rtb_Switch2 * rtU.YawS_P;

  /* SampleTimeMath: '<S146>/Tsamp' incorporates:
   *  Inport: '<Root>/YawS_N'
   *
   * About '<S146>/Tsamp':
   *  y = u * K where K = ( w * Ts )
   */
  rtb_NProdOut = rtU.YawS_N * 0.0025;

  /* Math: '<S144>/Reciprocal' incorporates:
   *  Constant: '<S144>/Constant'
   *  Sum: '<S144>/SumDen'
   *
   * About '<S144>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_SignPreIntegrator = 1.0 / (rtb_NProdOut + 1.0);

  /* DiscreteTransferFcn: '<S144>/Filter Differentiator TF' incorporates:
   *  Constant: '<S144>/Constant'
   *  Inport: '<Root>/YawS_D'
   *  Product: '<S143>/DProd Out'
   *  Product: '<S144>/Divide'
   *  Sum: '<S144>/SumNum'
   */
  FilterDifferentiatorTF_tmp_i = rtb_Switch2 * rtU.YawS_D - (rtb_NProdOut - 1.0)
    * rtb_SignPreIntegrator * rtDW.FilterDifferentiatorTF_states_b;

  /* Product: '<S154>/NProd Out' incorporates:
   *  DiscreteTransferFcn: '<S144>/Filter Differentiator TF'
   *  Inport: '<Root>/YawS_N'
   *  Product: '<S144>/DenCoefOut'
   */
  rtb_NProdOut = (FilterDifferentiatorTF_tmp_i +
                  -rtDW.FilterDifferentiatorTF_states_b) * rtb_SignPreIntegrator
    * rtU.YawS_N;

  /* Sum: '<S162>/Sum Fdbk' */
  rtb_DProdOut_jq = (rtb_PProdOut + rtDW.Integrator_DSTATE_m) + rtb_NProdOut;

  /* Switch: '<S142>/Switch' incorporates:
   *  Inport: '<Root>/YawS_LO'
   *  Inport: '<Root>/YawS_MO'
   *  RelationalOperator: '<S142>/u_GTE_up'
   *  RelationalOperator: '<S142>/u_GT_lo'
   *  Switch: '<S142>/Switch1'
   */
  if (rtb_DProdOut_jq >= rtU.YawS_MO) {
    rtb_SignPreIntegrator = rtU.YawS_MO;
  } else if (rtb_DProdOut_jq > rtU.YawS_LO) {
    /* Switch: '<S142>/Switch1' */
    rtb_SignPreIntegrator = rtb_DProdOut_jq;
  } else {
    rtb_SignPreIntegrator = rtU.YawS_LO;
  }

  /* End of Switch: '<S142>/Switch' */

  /* Sum: '<S142>/Diff' */
  rtb_SignPreIntegrator = rtb_DProdOut_jq - rtb_SignPreIntegrator;

  /* RelationalOperator: '<S139>/NotEqual' incorporates:
   *  Gain: '<S139>/ZeroGain'
   */
  rtb_NotEqual = (0.0 * rtb_DProdOut_jq != rtb_SignPreIntegrator);

  /* Signum: '<S139>/SignPreSat' */
  if (rtb_SignPreIntegrator < 0.0) {
    rtb_SignPreIntegrator = -1.0;
  } else if (rtb_SignPreIntegrator > 0.0) {
    rtb_SignPreIntegrator = 1.0;
  } else if (rtb_SignPreIntegrator == 0.0) {
    rtb_SignPreIntegrator = 0.0;
  } else {
    rtb_SignPreIntegrator = (rtNaN);
  }

  /* End of Signum: '<S139>/SignPreSat' */

  /* Product: '<S148>/IProd Out' incorporates:
   *  Inport: '<Root>/YawS_I'
   */
  rtb_Switch2 *= rtU.YawS_I;

  /* Signum: '<S139>/SignPreIntegrator' */
  if (rtb_Switch2 < 0.0) {
    rtb_DProdOut_g = -1.0;
  } else if (rtb_Switch2 > 0.0) {
    rtb_DProdOut_g = 1.0;
  } else if (rtb_Switch2 == 0.0) {
    rtb_DProdOut_g = 0.0;
  } else {
    rtb_DProdOut_g = (rtNaN);
  }

  /* End of Signum: '<S139>/SignPreIntegrator' */

  /* Switch: '<S139>/Switch' incorporates:
   *  Constant: '<S139>/Constant1'
   *  DataTypeConversion: '<S139>/DataTypeConv1'
   *  DataTypeConversion: '<S139>/DataTypeConv2'
   *  Logic: '<S139>/AND3'
   *  RelationalOperator: '<S139>/Equal1'
   */
  if (rtb_NotEqual && ((int8_T)rtb_SignPreIntegrator == (int8_T)rtb_DProdOut_g))
  {
    rtb_DProdOut_jq = 0.0;
  } else {
    rtb_DProdOut_jq = rtb_Switch2;
  }

  /* End of Switch: '<S139>/Switch' */

  /* DiscreteIntegrator: '<S151>/Integrator' */
  Integrator_f = 0.0025 * rtb_DProdOut_jq + rtDW.Integrator_DSTATE_m;

  /* Sum: '<S161>/Sum' */
  rtb_NProdOut += rtb_PProdOut + Integrator_f;

  /* Switch: '<S159>/Switch2' incorporates:
   *  Inport: '<Root>/YawS_LO'
   *  Inport: '<Root>/YawS_MO'
   *  RelationalOperator: '<S159>/LowerRelop1'
   *  RelationalOperator: '<S159>/UpperRelop'
   *  Switch: '<S159>/Switch'
   */
  if (rtb_NProdOut > rtU.YawS_MO) {
    /* Outport: '<Root>/YawCurrent' */
    rtY.YawCurrent = rtU.YawS_MO;
  } else if (rtb_NProdOut < rtU.YawS_LO) {
    /* Switch: '<S159>/Switch' incorporates:
     *  Inport: '<Root>/YawS_LO'
     *  Outport: '<Root>/YawCurrent'
     */
    rtY.YawCurrent = rtU.YawS_LO;
  } else {
    /* Outport: '<Root>/YawCurrent' incorporates:
     *  Switch: '<S159>/Switch'
     */
    rtY.YawCurrent = rtb_NProdOut;
  }

  /* End of Switch: '<S159>/Switch2' */

  /* Sum: '<Root>/Sum2' incorporates:
   *  Inport: '<Root>/PihAngle_Now'
   *  Inport: '<Root>/PihAngle_set'
   */
  rtb_Switch2 = rtU.PihAngle_set - rtU.PihAngle_Now;

  /* Product: '<S102>/PProd Out' incorporates:
   *  Inport: '<Root>/PihP_P'
   */
  rtb_PProdOut = rtb_Switch2 * rtU.PihP_P;

  /* SampleTimeMath: '<S92>/Tsamp' incorporates:
   *  Inport: '<Root>/PihP_N'
   *
   * About '<S92>/Tsamp':
   *  y = u * K where K = ( w * Ts )
   */
  rtb_NProdOut = rtU.PihP_N * 0.0025;

  /* Math: '<S90>/Reciprocal' incorporates:
   *  Constant: '<S90>/Constant'
   *  Sum: '<S90>/SumDen'
   *
   * About '<S90>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_SignPreIntegrator = 1.0 / (rtb_NProdOut + 1.0);

  /* DiscreteTransferFcn: '<S90>/Filter Differentiator TF' incorporates:
   *  Constant: '<S90>/Constant'
   *  Inport: '<Root>/PihP_D'
   *  Product: '<S89>/DProd Out'
   *  Product: '<S90>/Divide'
   *  Sum: '<S90>/SumNum'
   */
  FilterDifferentiatorTF_tmp_b = rtb_Switch2 * rtU.PihP_D - (rtb_NProdOut - 1.0)
    * rtb_SignPreIntegrator * rtDW.FilterDifferentiatorTF_states_bz;

  /* Product: '<S100>/NProd Out' incorporates:
   *  DiscreteTransferFcn: '<S90>/Filter Differentiator TF'
   *  Inport: '<Root>/PihP_N'
   *  Product: '<S90>/DenCoefOut'
   */
  rtb_NProdOut = (FilterDifferentiatorTF_tmp_b +
                  -rtDW.FilterDifferentiatorTF_states_bz) *
    rtb_SignPreIntegrator * rtU.PihP_N;

  /* Sum: '<S108>/Sum Fdbk' */
  rtb_DProdOut_j = (rtb_PProdOut + rtDW.Integrator_DSTATE_n) + rtb_NProdOut;

  /* Switch: '<S88>/Switch' incorporates:
   *  Inport: '<Root>/PihP_LO'
   *  Inport: '<Root>/PihP_MO'
   *  RelationalOperator: '<S88>/u_GTE_up'
   *  RelationalOperator: '<S88>/u_GT_lo'
   *  Switch: '<S88>/Switch1'
   */
  if (rtb_DProdOut_j >= rtU.PihP_MO) {
    rtb_SignPreIntegrator = rtU.PihP_MO;
  } else if (rtb_DProdOut_j > rtU.PihP_LO) {
    /* Switch: '<S88>/Switch1' */
    rtb_SignPreIntegrator = rtb_DProdOut_j;
  } else {
    rtb_SignPreIntegrator = rtU.PihP_LO;
  }

  /* End of Switch: '<S88>/Switch' */

  /* Sum: '<S88>/Diff' */
  rtb_SignPreIntegrator = rtb_DProdOut_j - rtb_SignPreIntegrator;

  /* RelationalOperator: '<S85>/NotEqual' incorporates:
   *  Gain: '<S85>/ZeroGain'
   */
  rtb_NotEqual = (0.0 * rtb_DProdOut_j != rtb_SignPreIntegrator);

  /* Signum: '<S85>/SignPreSat' */
  if (rtb_SignPreIntegrator < 0.0) {
    rtb_SignPreIntegrator = -1.0;
  } else if (rtb_SignPreIntegrator > 0.0) {
    rtb_SignPreIntegrator = 1.0;
  } else if (rtb_SignPreIntegrator == 0.0) {
    rtb_SignPreIntegrator = 0.0;
  } else {
    rtb_SignPreIntegrator = (rtNaN);
  }

  /* End of Signum: '<S85>/SignPreSat' */

  /* Product: '<S94>/IProd Out' incorporates:
   *  Inport: '<Root>/PihP_I'
   */
  rtb_Switch2 *= rtU.PihP_I;

  /* Signum: '<S85>/SignPreIntegrator' */
  if (rtb_Switch2 < 0.0) {
    rtb_DProdOut_g = -1.0;
  } else if (rtb_Switch2 > 0.0) {
    rtb_DProdOut_g = 1.0;
  } else if (rtb_Switch2 == 0.0) {
    rtb_DProdOut_g = 0.0;
  } else {
    rtb_DProdOut_g = (rtNaN);
  }

  /* End of Signum: '<S85>/SignPreIntegrator' */

  /* Switch: '<S85>/Switch' incorporates:
   *  Constant: '<S85>/Constant1'
   *  DataTypeConversion: '<S85>/DataTypeConv1'
   *  DataTypeConversion: '<S85>/DataTypeConv2'
   *  Logic: '<S85>/AND3'
   *  RelationalOperator: '<S85>/Equal1'
   */
  if (rtb_NotEqual && ((int8_T)rtb_SignPreIntegrator == (int8_T)rtb_DProdOut_g))
  {
    rtb_DProdOut_j = 0.0;
  } else {
    rtb_DProdOut_j = rtb_Switch2;
  }

  /* End of Switch: '<S85>/Switch' */

  /* DiscreteIntegrator: '<S97>/Integrator' */
  Integrator_b = 0.0025 * rtb_DProdOut_j + rtDW.Integrator_DSTATE_n;

  /* Sum: '<S107>/Sum' */
  rtb_NProdOut += rtb_PProdOut + Integrator_b;

  /* SampleTimeMath: '<S2>/TSamp' incorporates:
   *  Inport: '<Root>/PihAngle_set'
   *  Inport: '<Root>/Pih_Dif_Gain'
   *  Product: '<Root>/Product1'
   *
   * About '<S2>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_c = rtU.Pih_Dif_Gain * rtU.PihAngle_set * 200.0;

  /* Switch: '<S105>/Switch2' incorporates:
   *  Inport: '<Root>/PihP_LO'
   *  Inport: '<Root>/PihP_MO'
   *  RelationalOperator: '<S105>/LowerRelop1'
   *  RelationalOperator: '<S105>/UpperRelop'
   *  Switch: '<S105>/Switch'
   */
  if (rtb_NProdOut > rtU.PihP_MO) {
    rtb_NProdOut = rtU.PihP_MO;
  } else {
    if (rtb_NProdOut < rtU.PihP_LO) {
      /* Switch: '<S105>/Switch' incorporates:
       *  Inport: '<Root>/PihP_LO'
       */
      rtb_NProdOut = rtU.PihP_LO;
    }
  }

  /* End of Switch: '<S105>/Switch2' */

  /* Sum: '<Root>/Sum3' incorporates:
   *  Inport: '<Root>/PihSpeed_Now'
   *  Sum: '<S2>/Diff'
   *  UnitDelay: '<S2>/UD'
   *
   * Block description for '<S2>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Switch2 = (rtb_NProdOut - rtU.PihSpeed_Now) + (rtb_TSamp_c -
    rtDW.UD_DSTATE_o);

  /* Product: '<S210>/PProd Out' incorporates:
   *  Inport: '<Root>/PihS_P'
   */
  rtb_PProdOut = rtb_Switch2 * rtU.PihS_P;

  /* SampleTimeMath: '<S200>/Tsamp' incorporates:
   *  Inport: '<Root>/PihS_N'
   *
   * About '<S200>/Tsamp':
   *  y = u * K where K = ( w * Ts )
   */
  rtb_NProdOut = rtU.PihS_N * 0.0025;

  /* Math: '<S198>/Reciprocal' incorporates:
   *  Constant: '<S198>/Constant'
   *  Sum: '<S198>/SumDen'
   *
   * About '<S198>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_SignPreIntegrator = 1.0 / (rtb_NProdOut + 1.0);

  /* DiscreteTransferFcn: '<S198>/Filter Differentiator TF' incorporates:
   *  Constant: '<S198>/Constant'
   *  Inport: '<Root>/PihS_D'
   *  Product: '<S197>/DProd Out'
   *  Product: '<S198>/Divide'
   *  Sum: '<S198>/SumNum'
   */
  FilterDifferentiatorTF_tmp_j = rtb_Switch2 * rtU.PihS_D - (rtb_NProdOut - 1.0)
    * rtb_SignPreIntegrator * rtDW.FilterDifferentiatorTF_states_a;

  /* Product: '<S208>/NProd Out' incorporates:
   *  DiscreteTransferFcn: '<S198>/Filter Differentiator TF'
   *  Inport: '<Root>/PihS_N'
   *  Product: '<S198>/DenCoefOut'
   */
  rtb_NProdOut = (FilterDifferentiatorTF_tmp_j +
                  -rtDW.FilterDifferentiatorTF_states_a) * rtb_SignPreIntegrator
    * rtU.PihS_N;

  /* Sum: '<S216>/Sum Fdbk' */
  rtb_DProdOut_g = (rtb_PProdOut + rtDW.Integrator_DSTATE_n3) + rtb_NProdOut;

  /* Switch: '<S196>/Switch' incorporates:
   *  Inport: '<Root>/PihS_LO'
   *  Inport: '<Root>/PihS_MO'
   *  RelationalOperator: '<S196>/u_GTE_up'
   *  RelationalOperator: '<S196>/u_GT_lo'
   *  Switch: '<S196>/Switch1'
   */
  if (rtb_DProdOut_g >= rtU.PihS_MO) {
    rtb_SignPreIntegrator = rtU.PihS_MO;
  } else if (rtb_DProdOut_g > rtU.PihS_LO) {
    /* Switch: '<S196>/Switch1' */
    rtb_SignPreIntegrator = rtb_DProdOut_g;
  } else {
    rtb_SignPreIntegrator = rtU.PihS_LO;
  }

  /* End of Switch: '<S196>/Switch' */

  /* Sum: '<S196>/Diff' */
  rtb_SignPreIntegrator = rtb_DProdOut_g - rtb_SignPreIntegrator;

  /* RelationalOperator: '<S193>/NotEqual' incorporates:
   *  Gain: '<S193>/ZeroGain'
   */
  rtb_NotEqual = (0.0 * rtb_DProdOut_g != rtb_SignPreIntegrator);

  /* Signum: '<S193>/SignPreSat' */
  if (rtb_SignPreIntegrator < 0.0) {
    rtb_SignPreIntegrator = -1.0;
  } else if (rtb_SignPreIntegrator > 0.0) {
    rtb_SignPreIntegrator = 1.0;
  } else if (rtb_SignPreIntegrator == 0.0) {
    rtb_SignPreIntegrator = 0.0;
  } else {
    rtb_SignPreIntegrator = (rtNaN);
  }

  /* End of Signum: '<S193>/SignPreSat' */

  /* Product: '<S202>/IProd Out' incorporates:
   *  Inport: '<Root>/PihS_I'
   */
  rtb_Switch2 *= rtU.PihS_I;

  /* Signum: '<S193>/SignPreIntegrator' */
  if (rtb_Switch2 < 0.0) {
    rtb_DProdOut_g = -1.0;
  } else if (rtb_Switch2 > 0.0) {
    rtb_DProdOut_g = 1.0;
  } else if (rtb_Switch2 == 0.0) {
    rtb_DProdOut_g = 0.0;
  } else {
    rtb_DProdOut_g = (rtNaN);
  }

  /* End of Signum: '<S193>/SignPreIntegrator' */

  /* Switch: '<S193>/Switch' incorporates:
   *  Constant: '<S193>/Constant1'
   *  DataTypeConversion: '<S193>/DataTypeConv1'
   *  DataTypeConversion: '<S193>/DataTypeConv2'
   *  Logic: '<S193>/AND3'
   *  RelationalOperator: '<S193>/Equal1'
   */
  if (rtb_NotEqual && ((int8_T)rtb_SignPreIntegrator == (int8_T)rtb_DProdOut_g))
  {
    rtb_SignPreIntegrator = 0.0;
  } else {
    rtb_SignPreIntegrator = rtb_Switch2;
  }

  /* End of Switch: '<S193>/Switch' */

  /* DiscreteIntegrator: '<S205>/Integrator' */
  rtb_DProdOut_g = 0.0025 * rtb_SignPreIntegrator + rtDW.Integrator_DSTATE_n3;

  /* Sum: '<S215>/Sum' */
  rtb_NProdOut += rtb_PProdOut + rtb_DProdOut_g;

  /* Switch: '<S213>/Switch2' incorporates:
   *  Inport: '<Root>/PihS_LO'
   *  Inport: '<Root>/PihS_MO'
   *  RelationalOperator: '<S213>/LowerRelop1'
   *  RelationalOperator: '<S213>/UpperRelop'
   *  Switch: '<S213>/Switch'
   */
  if (rtb_NProdOut > rtU.PihS_MO) {
    rtb_NProdOut = rtU.PihS_MO;
  } else {
    if (rtb_NProdOut < rtU.PihS_LO) {
      /* Switch: '<S213>/Switch' incorporates:
       *  Inport: '<Root>/PihS_LO'
       */
      rtb_NProdOut = rtU.PihS_LO;
    }
  }

  /* End of Switch: '<S213>/Switch2' */

  /* Outport: '<Root>/PihCurrent' incorporates:
   *  Inport: '<Root>/Weight_Gain'
   *  Inport: '<Root>/Weight_Now'
   *  Product: '<Root>/Product2'
   *  Sum: '<Root>/Sum4'
   */
  rtY.PihCurrent = rtU.Weight_Now * rtU.Weight_Gain + rtb_NProdOut;

  /* Update for DiscreteTransferFcn: '<S36>/Filter Differentiator TF' */
  rtDW.FilterDifferentiatorTF_states = FilterDifferentiatorTF_tmp;

  /* Update for DiscreteIntegrator: '<S43>/Integrator' */
  rtDW.Integrator_DSTATE = 0.0025 * rtb_DProdOut + Integrator;

  /* Update for UnitDelay: '<S1>/UD'
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE = rtb_Sum;

  /* Update for DiscreteTransferFcn: '<S144>/Filter Differentiator TF' */
  rtDW.FilterDifferentiatorTF_states_b = FilterDifferentiatorTF_tmp_i;

  /* Update for DiscreteIntegrator: '<S151>/Integrator' */
  rtDW.Integrator_DSTATE_m = 0.0025 * rtb_DProdOut_jq + Integrator_f;

  /* Update for DiscreteTransferFcn: '<S90>/Filter Differentiator TF' */
  rtDW.FilterDifferentiatorTF_states_bz = FilterDifferentiatorTF_tmp_b;

  /* Update for DiscreteIntegrator: '<S97>/Integrator' */
  rtDW.Integrator_DSTATE_n = 0.0025 * rtb_DProdOut_j + Integrator_b;

  /* Update for UnitDelay: '<S2>/UD'
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE_o = rtb_TSamp_c;

  /* Update for DiscreteTransferFcn: '<S198>/Filter Differentiator TF' */
  rtDW.FilterDifferentiatorTF_states_a = FilterDifferentiatorTF_tmp_j;

  /* Update for DiscreteIntegrator: '<S205>/Integrator' */
  rtDW.Integrator_DSTATE_n3 = 0.0025 * rtb_SignPreIntegrator + rtb_DProdOut_g;
}

/* Model step wrapper function for compatibility with a static main program */
void PID_step(int_T tid)
{
  switch (tid) {
   case 0 :
    PID_step0();
    break;

   case 1 :
    PID_step1();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void PID_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] PID.c
 */
