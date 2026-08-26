/*
 * File: engine_control_codegen.h
 *
 * Code generated for Simulink model 'engine_control_codegen'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Wed Aug 26 20:46:44 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef engine_control_codegen_h_
#define engine_control_codegen_h_
#ifndef engine_control_codegen_COMMON_INCLUDES_
#define engine_control_codegen_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                             /* engine_control_codegen_COMMON_INCLUDES_ */

#include "engine_control_codegen_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T lastSin;                      /* '<Root>/Sine Wave' */
  real_T lastCos;                      /* '<Root>/Sine Wave' */
  int32_T systemEnable;                /* '<Root>/Sine Wave' */
  uint8_T is_active_c3_engine_control_cod;/* '<Root>/Chart' */
  uint8_T is_c3_engine_control_codegen;/* '<Root>/Chart' */
  uint8_T is_D;                        /* '<Root>/Chart' */
} DW_engine_control_codegen_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  uint8_T shaft_sw;                    /* '<Root>/shaft_sw' */
  real32_T speed;                      /* '<Root>/speed' */
} ExtU_engine_control_codegen_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint8_T gear_state;                  /* '<Root>/gear_state' */
} ExtY_engine_control_codegen_T;

/* Parameters (default storage) */
struct P_engine_control_codegen_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T SineWave_Amp;                 /* Expression: 100
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Bias;                /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Freq;                /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Phase;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Hsin;                /* Computed Parameter: SineWave_Hsin
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_HCos;                /* Computed Parameter: SineWave_HCos
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_PSin;                /* Computed Parameter: SineWave_PSin
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_PCos;                /* Computed Parameter: SineWave_PCos
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_engine_control_codege_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block parameters (default storage) */
extern P_engine_control_codegen_T engine_control_codegen_P;

/* Block states (default storage) */
extern DW_engine_control_codegen_T engine_control_codegen_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_engine_control_codegen_T engine_control_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_engine_control_codegen_T engine_control_codegen_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern uint8_T DD;                     /* Variable: DD
                                        * Referenced by: '<Root>/Chart'
                                        */
extern uint8_T NN;                     /* Variable: NN
                                        * Referenced by: '<Root>/Chart'
                                        */
extern uint8_T PP;                     /* Variable: PP
                                        * Referenced by: '<Root>/Chart'
                                        */
extern uint8_T RR;                     /* Variable: RR
                                        * Referenced by: '<Root>/Chart'
                                        */

/* Model entry point functions */
extern void engine_control_codegen_initialize(void);
extern void engine_control_codegen_step(void);
extern void engine_control_codegen_terminate(void);

/* Real-time Model object */
extern RT_MODEL_engine_control_codeg_T *const engine_control_codegen_M;

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
 * '<Root>' : 'engine_control_codegen'
 * '<S1>'   : 'engine_control_codegen/Chart'
 */
#endif                                 /* engine_control_codegen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
