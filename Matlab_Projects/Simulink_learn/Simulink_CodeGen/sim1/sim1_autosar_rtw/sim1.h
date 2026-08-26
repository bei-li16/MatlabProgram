/*
 * File: sim1.h
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

#ifndef sim1_h_
#define sim1_h_
#ifndef sim1_COMMON_INCLUDES_
#define sim1_COMMON_INCLUDES_
#include "Platform_Types.h"
#include "Rte_sim1.h"
#endif                                 /* sim1_COMMON_INCLUDES_ */

#include "sim1_types.h"

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile float64 kgain;   /* Referenced by: '<Root>/Gain' */

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
 * '<Root>' : 'sim1'
 * '<S1>'   : 'sim1/FuncA'
 */
#endif                                 /* sim1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
