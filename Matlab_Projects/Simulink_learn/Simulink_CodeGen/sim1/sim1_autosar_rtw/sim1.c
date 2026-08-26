/*
 * File: sim1.c
 *
 * Code generated for Simulink model 'sim1'.
 *
 * Model version                  : 1.42
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Jan 22 20:37:47 2026
 *
 * Target selection: autosar.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "sim1.h"
#include "Platform_Types.h"

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile float64 kgain = 5.0;    /* Referenced by: '<Root>/Gain' */

/* Model step function */
void sim1_Step(void)
{
  /* Outport: '<Root>/yout' incorporates:
   *  Constant: '<S1>/Constant'
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/In1'
   *  Sum: '<S1>/Add'
   */
  Rte_IWrite_sim1_Step_Out1_Out1(5.0 * (2.0 * Rte_IRead_sim1_Step_In1_In1() +
    5.0));

  /* Outport: '<Root>/zout' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/In2'
   *  Inport: '<Root>/In3'
   *  Sum: '<Root>/Add'
   */
  Rte_IWrite_sim1_Step_zout_zout(kgain * (Rte_IRead_sim1_Step_In2_In2() +
    Rte_IRead_sim1_Step_In3_In3()));
}

/* Model initialize function */
void sim1_Init(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
