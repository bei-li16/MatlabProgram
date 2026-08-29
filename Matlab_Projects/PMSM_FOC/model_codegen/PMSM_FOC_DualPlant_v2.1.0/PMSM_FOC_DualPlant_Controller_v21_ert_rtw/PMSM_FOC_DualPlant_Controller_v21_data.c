/*
 * File: PMSM_FOC_DualPlant_Controller_v21_data.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.42
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sat Aug 29 03:10:30 2026
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

  /* Expression: single(1.0)
   * Referenced by: '<S1>/Align_Cos_One'
   */
  1.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S1>/Align_Sin_Zero'
   */
  0.0F,

  /* Expression: single(2.0)
   * Referenced by: '<S1>/Align_Vd_2V'
   */
  2.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S1>/Align_Vq_Zero'
   */
  0.0F,

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

  /* Expression: single(100.0)
   * Referenced by: '<S3>/Sample_Target'
   */
  100.0F,

  /* Expression: single(1.0)
   * Referenced by: '<Root>/Start_Threshold_Rpm'
   */
  1.0F,

  /* Expression: single(12.0)
   * Referenced by: '<S8>/Max_Current_A'
   */
  12.0F,

  /* Expression: single(3000.0)
   * Referenced by: '<S15>/Max_Speed_Rpm'
   */
  3000.0F,

  /* Expression: single(10.0)
   * Referenced by: '<S15>/Min_Vdc'
   */
  10.0F,

  /* Expression: single(60.0)
   * Referenced by: '<S15>/Max_Vdc'
   */
  60.0F,

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

  /* Expression: single(0.5)
   * Referenced by: '<Root>/Safe_Duty_50pct'
   */
  0.5F,

  /* Expression: single(0.0)
   * Referenced by: '<S16>/Integrator_State'
   */
  0.0F,

  /* Expression: false
   * Referenced by: '<S8>/Fault_Latch_State'
   */
  false
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
