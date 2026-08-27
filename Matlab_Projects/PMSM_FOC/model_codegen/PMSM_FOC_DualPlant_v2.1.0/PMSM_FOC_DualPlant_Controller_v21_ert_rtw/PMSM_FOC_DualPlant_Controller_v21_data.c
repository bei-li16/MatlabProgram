/*
 * File: PMSM_FOC_DualPlant_Controller_v21_data.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.17
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Aug 27 21:15:33 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PMSM_FOC_DualPlant_Controller_v21.h"

/* Block parameters (default storage) */
P_PMSM_FOC_DualPlant_Controll_T PMSM_FOC_DualPlant_Controller_P = {
  /* Variable: NATIVE_INV_SQRT3
   * Referenced by: '<S1>/InvSqrt3'
   */
  0.577350259F,

  /* Variable: NATIVE_RPM_TO_RAD_S
   * Referenced by:
   *   '<S2>/Electrical_Speed'
   *   '<S10>/RpmToRad'
   */
  0.104719758F,

  /* Variable: NATIVE_SQRT3_BY2
   * Referenced by:
   *   '<S5>/Vb_Beta'
   *   '<S5>/Vc_Beta'
   */
  0.866025388F,

  /* Expression: single(0.0)
   * Referenced by: '<Root>/Id_Reference_Zero'
   */
  0.0F,

  /* Expression: single(2.0)
   * Referenced by: '<S1>/Ib_x2'
   */
  2.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S4>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-1.0)
   * Referenced by: '<S7>/Negative'
   */
  -1.0F,

  /* Computed Parameter: IqRef_Rate_Transition_InitialCo
   * Referenced by: '<Root>/IqRef_Rate_Transition'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S8>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-0.5)
   * Referenced by: '<S5>/Vb_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S5>/Vc_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S9>/Common_Mode'
   */
  -0.5F,

  /* Expression: single(0.5)
   * Referenced by: '<S9>/Duty_Half'
   */
  0.5F,

  /* Expression: single(0.0)
   * Referenced by: '<S10>/Integrator_State'
   */
  0.0F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
