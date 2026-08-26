/*
 * File: sim2.h
 *
 * Code generated for Simulink model 'sim2'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sun Jun 29 22:05:17 2025
 *
 * Target selection: autosar.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef sim2_h_
#define sim2_h_
#ifndef sim2_COMMON_INCLUDES_
#define sim2_COMMON_INCLUDES_
#include "Platform_Types.h"
#include "Rte_sim2.h"
#endif                                 /* sim2_COMMON_INCLUDES_ */

#include "sim2_types.h"

/* PublicStructure Variables for Internal Data, for system '<Root>' */
typedef struct {
  uint8 is_active_c1_sim2;             /* '<Root>/Chart1' */
  uint8 is_c1_sim2;                    /* '<Root>/Chart1' */
  uint8 is_D;                          /* '<Root>/Chart1' */
} ARID_DEF_sim2_T;

/* PublicStructure Variables for Internal Data */
extern ARID_DEF_sim2_T sim2_ARID_DEF;  /* '<Root>/Chart1' */

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
 * '<Root>' : 'sim2'
 * '<S1>'   : 'sim2/Chart1'
 */
#endif                                 /* sim2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
