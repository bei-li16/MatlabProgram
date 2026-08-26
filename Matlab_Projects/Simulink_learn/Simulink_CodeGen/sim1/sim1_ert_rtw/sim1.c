/*
 * File: sim1.c
 *
 * Code generated for Simulink model 'sim1'.
 *
 * Model version                  : 1.41
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sun Jun 29 23:35:45 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Passed (10), Warnings (3), Error (0)
 */

#include "sim1.h"

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile double kgain = 5.0;     /* Referenced by: '<Root>/Gain' */

/* External inputs (root inport signals with default storage) */
ExtU_sim1_T sim1_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_sim1_T sim1_Y;

/* Real-time model */
static RT_MODEL_sim1_T sim1_M_;
RT_MODEL_sim1_T *const sim1_M = &sim1_M_;

/* Model step function */
void sim1_step(void)
{
  /* Outport: '<Root>/yout' incorporates:
   *  Constant: '<S1>/Constant'
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/In1'
   *  Sum: '<S1>/Add'
   */
  sim1_Y.yout = (2.0 * sim1_U.xin + 5.0) * 5.0;

  /* Outport: '<Root>/zout' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/In2'
   *  Inport: '<Root>/In3'
   *  Sum: '<Root>/Add'
   */
  sim1_Y.zout = (sim1_U.x + sim1_U.y) * kgain;
}

/* Model initialize function */
void sim1_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void sim1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
