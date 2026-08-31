/*
 * File: PMSM_FOC_DualPlant_Controller_v21_data.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.13
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Aug 31 11:44:54 2026
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
   *   '<S7>/KiTs'
   *   '<S16>/KiTs'
   */
  0.0001F,

  /* Variable: FOC_Native_SpeedPeriod
   * Referenced by: '<S19>/KiTs'
   */
  0.001F,

  /* Variable: NATIVE_INV_SQRT3
   * Referenced by: '<S2>/InvSqrt3'
   */
  0.577350259F,

  /* Variable: NATIVE_RPM_TO_RAD_S
   * Referenced by:
   *   '<S5>/Electrical_Speed'
   *   '<S19>/RpmToRad'
   */
  0.104719758F,

  /* Variable: NATIVE_SQRT3_BY2
   * Referenced by:
   *   '<S12>/Vb_Beta'
   *   '<S12>/Vc_Beta'
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

  /* Expression: single(1.0)
   * Referenced by: '<S4>/One'
   */
  1.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S7>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S16>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S19>/Integrator_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<Root>/Id_Reference_Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S4>/SumA_State'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S4>/Count_State'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S4>/Zero'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S4>/SumB_State'
   */
  0.0F,

  /* Expression: single(2.0)
   * Referenced by: '<S2>/Ib_x2'
   */
  2.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S7>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-1.0)
   * Referenced by: '<S15>/Negative'
   */
  -1.0F,

  /* Computed Parameter: IqRef_Rate_Transition_InitialCo
   * Referenced by: '<Root>/IqRef_Rate_Transition'
   */
  0.0F,

  /* Expression: single(0.0)
   * Referenced by: '<S16>/Integrator_State'
   */
  0.0F,

  /* Expression: single(-0.5)
   * Referenced by: '<S12>/Vb_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S12>/Vc_Alpha'
   */
  -0.5F,

  /* Expression: single(-0.5)
   * Referenced by: '<S17>/Common_Mode'
   */
  -0.5F,

  /* Expression: single(0.5)
   * Referenced by: '<S17>/Duty_Half'
   */
  0.5F,

  /* Expression: single(0.0)
   * Referenced by: '<S19>/Integrator_State'
   */
  0.0F,

  /* Computed Parameter: FaultBit6_Weight_Gain
   * Referenced by: '<S10>/FaultBit6_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit5_Weight_Gain
   * Referenced by: '<S10>/FaultBit5_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit4_Weight_Gain
   * Referenced by: '<S10>/FaultBit4_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit3_Weight_Gain
   * Referenced by: '<S10>/FaultBit3_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit2_Weight_Gain
   * Referenced by: '<S10>/FaultBit2_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit1_Weight_Gain
   * Referenced by: '<S10>/FaultBit1_Weight'
   */
  2147483648U,

  /* Computed Parameter: FaultBit0_Weight_Gain
   * Referenced by: '<S10>/FaultBit0_Weight'
   */
  2147483648U,

  /* Expression: uint16(1)
   * Referenced by: '<S10>/FaultCode_1'
   */
  1U,

  /* Expression: uint16(2)
   * Referenced by: '<S10>/FaultCode_2'
   */
  2U,

  /* Expression: uint16(3)
   * Referenced by: '<S10>/FaultCode_3'
   */
  3U,

  /* Expression: uint16(4)
   * Referenced by: '<S10>/FaultCode_4'
   */
  4U,

  /* Expression: uint16(5)
   * Referenced by: '<S10>/FaultCode_5'
   */
  5U,

  /* Expression: uint16(6)
   * Referenced by: '<S10>/FaultCode_6'
   */
  6U,

  /* Expression: uint16(7)
   * Referenced by: '<S10>/FaultCode_7'
   */
  7U,

  /* Expression: uint16(0)
   * Referenced by: '<S10>/FaultCode_0'
   */
  0U,

  /* Computed Parameter: AlignmentEnable_To_100us_Initia
   * Referenced by: '<Root>/AlignmentEnable_To_100us'
   */
  false,

  /* Computed Parameter: Overspeed_To_100us_InitialCondi
   * Referenced by: '<Root>/Overspeed_To_100us'
   */
  false,

  /* Computed Parameter: Undervoltage_To_100us_InitialCo
   * Referenced by: '<Root>/Undervoltage_To_100us'
   */
  false,

  /* Computed Parameter: Overvoltage_To_100us_InitialCon
   * Referenced by: '<Root>/Overvoltage_To_100us'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/SoftwareOvercurrent_Latch_State'
   */
  false,

  /* Computed Parameter: SupervisorPwmRequest_To_100us_I
   * Referenced by: '<Root>/SupervisorPwmRequest_To_100us'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/EmergencyStop_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/DriverFault_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/MeasurementInvalid_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/Overspeed_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/Undervoltage_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<S10>/Overvoltage_Latch_State'
   */
  false,

  /* Expression: false
   * Referenced by: '<Root>/VoltageLimit_Inactive'
   */
  false,

  /* Computed Parameter: CalibrationEnable_To_100us_Init
   * Referenced by: '<Root>/CalibrationEnable_To_100us'
   */
  false,

  /* Computed Parameter: CalibrationReset_To_100us_Initi
   * Referenced by: '<Root>/CalibrationReset_To_100us'
   */
  false,

  /* Computed Parameter: ControllerReset_To_100us_Initia
   * Referenced by: '<Root>/ControllerReset_To_100us'
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
