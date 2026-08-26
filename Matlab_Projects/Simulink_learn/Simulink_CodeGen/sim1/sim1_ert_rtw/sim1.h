/*
 * File: sim1.h
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

#ifndef sim1_h_
#define sim1_h_
#ifndef sim1_COMMON_INCLUDES_
#define sim1_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "math.h"
#endif                                 /* sim1_COMMON_INCLUDES_ */

#include "sim1_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  double xin;                          /* '<Root>/In1' */
  double x;                            /* '<Root>/In2' */
  double y;                            /* '<Root>/In3' */
} ExtU_sim1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  double yout;                         /* '<Root>/yout' */
  double zout;                         /* '<Root>/zout' */
} ExtY_sim1_T;

/* Real-time Model Data Structure */
struct tag_RTM_sim1_T {
  const char * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_sim1_T sim1_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_sim1_T sim1_Y;

/* Model entry point functions */
extern void sim1_initialize(void);
extern void sim1_step(void);
extern void sim1_terminate(void);

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile double kgain;    /* Referenced by: '<Root>/Gain' */

/* Real-time Model object */
extern RT_MODEL_sim1_T *const sim1_M;

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
