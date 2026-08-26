/*
 * File: sim3.c
 *
 * Code generated for Simulink model 'sim3'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Jan 29 23:45:06 2026
 *
 * Target selection: autosar.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "sim3.h"
#include "Platform_Types.h"
#include "sim3_private.h"

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile uint32 kgain = 2U;      /* Referenced by: '<S1>/Gain' */

/* Model step function */
void sim3_Step(void)
{
  uint8 tmp;

  /* Gain: '<S1>/Gain' */
  if (kgain > 3U) {
    tmp = ((uint8)(255U));
  } else {
    tmp = (uint8)((sint32)kgain << 6);
  }

  /* Outport: '<Root>/zout' incorporates:
   *  Gain: '<S1>/Gain'
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Sum: '<S1>/Add'
   */
  Rte_IWrite_sim3_Step_zout_zout((uint16)((uint32)tmp * (uint8)
    (Rte_IRead_sim3_Step_In1_In1() + Rte_IRead_sim3_Step_In2_In2())));
}

/* Model initialize function */
void sim3_Init(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
