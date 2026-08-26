/*
 * File: sim3.h
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

#ifndef sim3_h_
#define sim3_h_
#ifndef sim3_COMMON_INCLUDES_
#define sim3_COMMON_INCLUDES_
#include "Platform_Types.h"
#include "Rte_sim3.h"
#endif                                 /* sim3_COMMON_INCLUDES_ */

#include "sim3_types.h"

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile uint32 kgain;    /* Referenced by: '<S1>/Gain' */

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
 * '<Root>' : 'sim3'
 * '<S1>'   : 'sim3/Subsystem'
 */
#endif                                 /* sim3_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
