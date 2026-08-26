/*
 * File: engine_control_codegen.c
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

#include "engine_control_codegen.h"
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<Root>/Chart' */
#define engine_contr_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define engine_control_codegen_IN_D    ((uint8_T)1U)
#define engine_control_codegen_IN_D1   ((uint8_T)1U)
#define engine_control_codegen_IN_D2   ((uint8_T)2U)
#define engine_control_codegen_IN_D3   ((uint8_T)3U)
#define engine_control_codegen_IN_D4   ((uint8_T)4U)
#define engine_control_codegen_IN_N    ((uint8_T)2U)
#define engine_control_codegen_IN_P    ((uint8_T)3U)
#define engine_control_codegen_IN_R    ((uint8_T)4U)

/* Exported block parameters */
uint8_T DD = 3U;                       /* Variable: DD
                                        * Referenced by: '<Root>/Chart'
                                        */
uint8_T NN = 2U;                       /* Variable: NN
                                        * Referenced by: '<Root>/Chart'
                                        */
uint8_T PP = 0U;                       /* Variable: PP
                                        * Referenced by: '<Root>/Chart'
                                        */
uint8_T RR = 1U;                       /* Variable: RR
                                        * Referenced by: '<Root>/Chart'
                                        */

/* Block states (default storage) */
DW_engine_control_codegen_T engine_control_codegen_DW;

/* External inputs (root inport signals with default storage) */
ExtU_engine_control_codegen_T engine_control_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_engine_control_codegen_T engine_control_codegen_Y;

/* Real-time model */
static RT_MODEL_engine_control_codeg_T engine_control_codegen_M_;
RT_MODEL_engine_control_codeg_T *const engine_control_codegen_M =
  &engine_control_codegen_M_;

/* Model step function */
void engine_control_codegen_step(void)
{
  real_T lastSin_tmp;

  /* Sin: '<Root>/Sine Wave' */
  if (engine_control_codegen_DW.systemEnable != 0) {
    lastSin_tmp = engine_control_codegen_P.SineWave_Freq *
      ((engine_control_codegen_M->Timing.clockTick0) * 0.01);
    engine_control_codegen_DW.lastSin = sin(lastSin_tmp);
    engine_control_codegen_DW.lastCos = cos(lastSin_tmp);
    engine_control_codegen_DW.systemEnable = 0;
  }

  /* End of Sin: '<Root>/Sine Wave' */

  /* Chart: '<Root>/Chart' incorporates:
   *  Inport: '<Root>/shaft_sw'
   *  Inport: '<Root>/speed'
   */
  if (engine_control_codegen_DW.is_active_c3_engine_control_cod == 0U) {
    engine_control_codegen_DW.is_active_c3_engine_control_cod = 1U;
    engine_control_codegen_DW.is_c3_engine_control_codegen =
      engine_control_codegen_IN_P;

    /* Outport: '<Root>/gear_state' */
    engine_control_codegen_Y.gear_state = PP;
  } else {
    switch (engine_control_codegen_DW.is_c3_engine_control_codegen) {
     case engine_control_codegen_IN_D:
      /* Outport: '<Root>/gear_state' */
      engine_control_codegen_Y.gear_state = DD;
      if (engine_control_codegen_U.shaft_sw == NN) {
        engine_control_codegen_DW.is_D = engine_contr_IN_NO_ACTIVE_CHILD;
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_N;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = NN;
      } else {
        switch (engine_control_codegen_DW.is_D) {
         case engine_control_codegen_IN_D1:
          if (engine_control_codegen_U.speed > 20.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D2;
          }
          break;

         case engine_control_codegen_IN_D2:
          if (engine_control_codegen_U.speed > 40.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D3;
          } else if (engine_control_codegen_U.speed < 15.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D1;
          }
          break;

         case engine_control_codegen_IN_D3:
          if (engine_control_codegen_U.speed > 60.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D4;
          } else if (engine_control_codegen_U.speed < 35.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D2;
          }
          break;

         default:
          /* case IN_D4: */
          if (engine_control_codegen_U.speed < 55.0F) {
            engine_control_codegen_DW.is_D = engine_control_codegen_IN_D3;
          }
          break;
        }
      }
      break;

     case engine_control_codegen_IN_N:
      /* Outport: '<Root>/gear_state' */
      engine_control_codegen_Y.gear_state = NN;
      if (engine_control_codegen_U.shaft_sw == DD) {
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_D;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = DD;
        engine_control_codegen_DW.is_D = engine_control_codegen_IN_D1;
      } else if (engine_control_codegen_U.shaft_sw == RR) {
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_R;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = RR;
      }
      break;

     case engine_control_codegen_IN_P:
      /* Outport: '<Root>/gear_state' */
      engine_control_codegen_Y.gear_state = PP;
      if (engine_control_codegen_U.shaft_sw == RR) {
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_R;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = RR;
      }
      break;

     default:
      /* Outport: '<Root>/gear_state' */
      /* case IN_R: */
      engine_control_codegen_Y.gear_state = RR;
      if (engine_control_codegen_U.shaft_sw == NN) {
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_N;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = NN;
      } else if (engine_control_codegen_U.shaft_sw == PP) {
        engine_control_codegen_DW.is_c3_engine_control_codegen =
          engine_control_codegen_IN_P;

        /* Outport: '<Root>/gear_state' */
        engine_control_codegen_Y.gear_state = PP;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/Chart' */

  /* Update for Sin: '<Root>/Sine Wave' */
  lastSin_tmp = engine_control_codegen_DW.lastSin;
  engine_control_codegen_DW.lastSin = engine_control_codegen_DW.lastSin *
    engine_control_codegen_P.SineWave_HCos + engine_control_codegen_DW.lastCos *
    engine_control_codegen_P.SineWave_Hsin;
  engine_control_codegen_DW.lastCos = engine_control_codegen_DW.lastCos *
    engine_control_codegen_P.SineWave_HCos - lastSin_tmp *
    engine_control_codegen_P.SineWave_Hsin;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  engine_control_codegen_M->Timing.clockTick0++;
}

/* Model initialize function */
void engine_control_codegen_initialize(void)
{
  /* Enable for Sin: '<Root>/Sine Wave' */
  engine_control_codegen_DW.systemEnable = 1;
}

/* Model terminate function */
void engine_control_codegen_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
