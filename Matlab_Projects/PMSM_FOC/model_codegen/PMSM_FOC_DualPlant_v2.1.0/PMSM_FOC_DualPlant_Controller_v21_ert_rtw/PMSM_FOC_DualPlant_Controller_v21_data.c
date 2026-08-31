/*
 * File: PMSM_FOC_DualPlant_Controller_v21_data.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Aug 31 04:08:31 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PMSM_FOC_DualPlant_Controller_v21.h"

/* Block parameters (default storage) */
P_PMSM_FOC_DualPlant_Controll_T PMSM_FOC_DualPlant_Controller_P = {
  /* Variable: FOC_Native_CurrentPeriod
   * Referenced by:
   *   '<S6>/KiTs'
   *   '<S13>/KiTs'
   */
  0.0001F,

  /* Variable: FOC_Native_SpeedPeriod
   * Referenced by: '<S16>/KiTs'
   */
  0.001F,

  /* Variable: NATIVE_INV_SQRT3
   * Referenced by: '<S2>/InvSqrt3'
   */
  0.577350259F,

  /* Variable: NATIVE_RPM_TO_RAD_S
   * Referenced by:
   *   '<S4>/Electrical_Speed'
   *   '<S16>/RpmToRad'
   */
  0.104719758F,

  /* Variable: NATIVE_SQRT3_BY2
   * Referenced by:
   *   '<S9>/Vb_Beta'
   *   '<S9>/Vc_Beta'
   */
  0.866025388F,

  /* Variable: PMSM_Alignment_Cos
   * Referenced by: '<S1>/Align_Cos'
   */
  1.0F,

  /* Variable: PMSM_Alignment_Sin
   * Referenced by: '<S1>/Align_Sin'
   */
  0.0F,

  /* Variable: PMSM_SafeDuty
   * Referenced by: '<Root>/Safe_Duty_50pct'
   */
  0.5F,

  /* Expression: single(1.0)
   * Referenced by: '<S3>/One'
   */
  1.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S6>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S13>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S16>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S3>/Count_State'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<Root>/Id_Reference_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S3>/SumA_State'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S3>/Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S3>/SumB_State'
   */
  0.0F,

  /* Expression: single(2.0)
   * Referenced by: '<S2>/Ib_x2'
   */
  2.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S6>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-1.0)
   * Referenced by: '<S12>/Negative'
   */
  -1.0F,

  /* Computed Parameter: IqRef_Rate_Transition_InitialCo
   * Referenced by: '<Root>/IqRef_Rate_Transition'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S13>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-0.5)
   * Referenced by: '<S9>/Vb_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S9>/Vc_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S14>/Common_Mode'
   */
  -0.5F,

  /* Expression: single(0.5)
   * Referenced by: '<S14>/Duty_Half'
   */
  0.5F,

  /* Expression: single(0.0)
   * Referenced by: '<S16>/Integrator_State'
   */
  0.0F,

  /* Expression: false
   * Referenced by: '<S8>/Fault_Latch_State'
   */
  false,

  /* Computed Parameter: StatusFault_To_100us_InitialCon
   * Referenced by: '<Root>/StatusFault_To_100us'
   */
  false,

  /* Expression: false
   * Referenced by: '<Root>/VoltageLimit_Inactive'
   */
  false,

  /* Computed Parameter: StatusState_To_100us_InitialCon
   * Referenced by: '<Root>/StatusState_To_100us'
   */
  0U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
