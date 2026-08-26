/*
 * File: sim3.c
 *
 * Code generated for Simulink model 'sim3'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Jan 29 23:52:15 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "sim3.h"
#include <stdint.h>
#include "sim3_private.h"

/* Exported block signals */
uint16_t z;                            /* '<S1>/Gain' */

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile uint8_t kgain = 2U;     /* Referenced by: '<S1>/Gain' */

/* Real-time model */
static RT_MODEL_sim3_T sim3_M_;
RT_MODEL_sim3_T *const sim3_M = &sim3_M_;

/* Model step function */
void sim3_step(void)
{
  uint8_t tmp;

  /* Gain: '<S1>/Gain' */
  if (kgain > 3) {
    tmp = UINT8_MAX;
  } else {
    tmp = (uint8_t)(kgain << 6);
  }

  /* Gain: '<S1>/Gain' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Sum: '<S1>/Add'
   */
  z = (uint16_t)((uint32_t)(uint8_t)(x + y) * tmp);
}

/* Model initialize function */
void sim3_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void sim3_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
