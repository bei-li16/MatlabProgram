/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
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
#include <math.h>
#include "rtwtypes.h"

/* Exported block parameters */
real32_T FOC_Native_CurrentIntegratorLimit = 30.0F;
                                  /* Variable: FOC_Native_CurrentIntegratorLimit
                                   * Referenced by:
                                   *   '<S4>/Integrator_Limit'
                                   *   '<S8>/Integrator_Limit'
                                   */
real32_T FOC_Native_CurrentPeriod = 0.0001F;/* Variable: FOC_Native_CurrentPeriod
                                             * Referenced by:
                                             *   '<S4>/KiTs'
                                             *   '<S8>/KiTs'
                                             */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S9>/Duty_A_Limit'
                                        *   '<S9>/Duty_B_Limit'
                                        *   '<S9>/Duty_C_Limit'
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S9>/Duty_A_Limit'
                                        *   '<S9>/Duty_B_Limit'
                                        *   '<S9>/Duty_C_Limit'
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S2>/Flux_PM'
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S10>/Integrator_Limit'
                                        *   '<S10>/Iq_Reference_Limit'
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S4>/KiTs'
                                        *   '<S8>/KiTs'
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S10>/KiTs'
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S4>/Kp'
                                        *   '<S8>/Kp'
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S10>/Kp'
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S2>/Ld_x_Id'
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S2>/D_Decoupling'
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S2>/Electrical_Speed'
                                        */
real32_T FOC_Native_SpeedPeriod = 0.001F;/* Variable: FOC_Native_SpeedPeriod
                                          * Referenced by: '<S10>/KiTs'
                                          */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S3>/Vd_Limit'
                                          *   '<S3>/Vq_Limit'
                                          */

/* Block states (default storage) */
DW_PMSM_FOC_DualPlant_Control_T PMSM_FOC_DualPlant_Controlle_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PMSM_FOC_DualPlant_Contr_T PMSM_FOC_DualPlant_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PMSM_FOC_DualPlant_Contr_T PMSM_FOC_DualPlant_Controller_Y;

/* Real-time model */
static RT_MODEL_PMSM_FOC_DualPlant_C_T PMSM_FOC_DualPlant_Controlle_M_;
RT_MODEL_PMSM_FOC_DualPlant_C_T *const PMSM_FOC_DualPlant_Controlle_M =
  &PMSM_FOC_DualPlant_Controlle_M_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1])++;
  if ((PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
void PMSM_FOC_DualPlant_Controller_v21_step(void)
{
  real32_T rtb_Current_Error;
  real32_T rtb_Current_Error_a;
  real32_T rtb_Integrator_Add;
  real32_T rtb_Phase_Vb;
  real32_T rtb_Phase_Vc;
  real32_T rtb_Valpha_Sum;

  /* Trigonometry: '<S6>/CosTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Trigonometry: '<S7>/CosTheta'
   */
  rtb_Phase_Vc = cosf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Trigonometry: '<S7>/SinTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Trigonometry: '<S6>/SinTheta'
   */
  rtb_Integrator_Add = sinf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Sum: '<S7>/Iq_Sum' incorporates:
   *  Gain: '<S1>/Ib_x2'
   *  Gain: '<S1>/InvSqrt3'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Sum: '<S1>/Ia_Plus_2Ib'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured =
    (PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain *
     PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB +
     PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S7>/Id_Sum' incorporates:
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S7>/Id_CosAlpha'
   *  Product: '<S7>/Id_SinBeta'
   *  Trigonometry: '<S7>/SinTheta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IdMeasured = rtb_Phase_Vc *
    PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA + rtb_Integrator_Add *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Sum: '<S4>/Current_Error' incorporates:
   *  Constant: '<Root>/Id_Reference_Zero'
   *
   * Block description for '<Root>/Id_Reference_Zero':
   *  Field-oriented control d-axis current reference: Id*=0 A.
   */
  rtb_Current_Error = PMSM_FOC_DualPlant_Controller_P.Id_Reference_Zero_Value -
    PMSM_FOC_DualPlant_Controller_Y.IdMeasured;

  /* Saturate: '<S3>/Vq_Limit' incorporates:
   *  Gain: '<S2>/Electrical_Speed'
   *  Inport: '<Root>/SpeedRpm'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand =
    PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S * FOC_Native_PolePairs *
    PMSM_FOC_DualPlant_Controller_U.SpeedRpm;

  /* Sum: '<S7>/Iq_Sum' incorporates:
   *  Gain: '<S7>/Negative'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S7>/Iq_CosBeta'
   *  Product: '<S7>/Iq_SinAlpha'
   *  Trigonometry: '<S7>/SinTheta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured = rtb_Integrator_Add *
    PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_Phase_Vc *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Saturate: '<S3>/Vd_Limit' incorporates:
   *  Gain: '<S2>/D_Decoupling'
   *  Gain: '<S4>/Kp'
   *  Product: '<S2>/Omega_x_Iq'
   *  Sum: '<S3>/Vd_Raw'
   *  Sum: '<S4>/PI_Sum'
   *  UnitDelay: '<S4>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.VdCommand = (FOC_Native_KpCurrent *
    rtb_Current_Error + PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE)
    + PMSM_FOC_DualPlant_Controller_Y.VqCommand *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured * -FOC_Native_Lq;

  /* Saturate: '<S3>/Vd_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.VdCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S3>/Vd_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_DualPlant_Controller_Y.VdCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S3>/Vd_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S3>/Vd_Limit' */

  /* RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outport: '<Root>/IqReference' */
    PMSM_FOC_DualPlant_Controller_Y.IqReference =
      PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0;
  }

  /* End of RateTransition: '<Root>/IqRef_Rate_Transition' */

  /* Sum: '<S8>/Current_Error' incorporates:
   *  Outport: '<Root>/IqReference'
   */
  rtb_Current_Error_a = PMSM_FOC_DualPlant_Controller_Y.IqReference -
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Saturate: '<S3>/Vq_Limit' incorporates:
   *  Constant: '<S2>/Flux_PM'
   *  Gain: '<S2>/Ld_x_Id'
   *  Gain: '<S8>/Kp'
   *  Product: '<S2>/Q_Feedforward'
   *  Sum: '<S2>/Flux_Linkage'
   *  Sum: '<S3>/Vq_Raw'
   *  Sum: '<S8>/PI_Sum'
   *  UnitDelay: '<S8>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand = (FOC_Native_Ld *
    PMSM_FOC_DualPlant_Controller_Y.IdMeasured + FOC_Native_FluxPM) *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand + (FOC_Native_KpCurrent *
    rtb_Current_Error_a +
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d);

  /* Saturate: '<S3>/Vq_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.VqCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S3>/Vq_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_DualPlant_Controller_Y.VqCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S3>/Vq_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S3>/Vq_Limit' */

  /* Sum: '<S6>/Valpha_Sum' incorporates:
   *  Product: '<S6>/Valpha_CosVd'
   *  Product: '<S6>/Valpha_SinVq'
   *  Trigonometry: '<S6>/CosTheta'
   */
  rtb_Valpha_Sum = rtb_Phase_Vc * PMSM_FOC_DualPlant_Controller_Y.VdCommand -
    rtb_Integrator_Add * PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S6>/Vbeta_Sum' incorporates:
   *  Product: '<S6>/Vbeta_CosVq'
   *  Product: '<S6>/Vbeta_SinVd'
   *  Trigonometry: '<S6>/CosTheta'
   */
  rtb_Phase_Vc = rtb_Integrator_Add * PMSM_FOC_DualPlant_Controller_Y.VdCommand
    + rtb_Phase_Vc * PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S5>/Phase_Vb' incorporates:
   *  Gain: '<S5>/Vb_Alpha'
   *  Gain: '<S5>/Vb_Beta'
   */
  rtb_Phase_Vb = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain * rtb_Valpha_Sum
    + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Sum: '<S5>/Phase_Vc' incorporates:
   *  Gain: '<S5>/Vc_Alpha'
   *  Gain: '<S5>/Vc_Beta'
   */
  rtb_Phase_Vc = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain * rtb_Valpha_Sum
    + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Gain: '<S9>/Common_Mode' incorporates:
   *  MinMax: '<S9>/Phase_Maximum'
   *  MinMax: '<S9>/Phase_Minimum'
   *  Sum: '<S9>/Max_Plus_Min'
   */
  rtb_Integrator_Add = (fmaxf(fmaxf(rtb_Valpha_Sum, rtb_Phase_Vb), rtb_Phase_Vc)
                        + fminf(fminf(rtb_Valpha_Sum, rtb_Phase_Vb),
    rtb_Phase_Vc)) * PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* Sum: '<S9>/Duty_B_Plus_Half' incorporates:
   *  Constant: '<S9>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S9>/Duty_B_Divide_Vdc'
   *  Sum: '<S9>/Phase_B_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyB = (rtb_Phase_Vb + rtb_Integrator_Add) /
    PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S9>/Duty_B_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyB > FOC_Native_DutyMax) {
    /* Sum: '<S9>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyB < FOC_Native_DutyMin) {
    /* Sum: '<S9>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S9>/Duty_B_Limit' */

  /* Sum: '<S9>/Duty_A_Plus_Half' incorporates:
   *  Constant: '<S9>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S9>/Duty_A_Divide_Vdc'
   *  Sum: '<S9>/Phase_A_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyA = (rtb_Valpha_Sum + rtb_Integrator_Add) /
    PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S9>/Duty_A_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyA > FOC_Native_DutyMax) {
    /* Sum: '<S9>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyA < FOC_Native_DutyMin) {
    /* Sum: '<S9>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S9>/Duty_A_Limit' */

  /* Sum: '<S9>/Duty_C_Plus_Half' incorporates:
   *  Constant: '<S9>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S9>/Duty_C_Divide_Vdc'
   *  Sum: '<S9>/Phase_C_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyC = (rtb_Phase_Vc + rtb_Integrator_Add) /
    PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S9>/Duty_C_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyC > FOC_Native_DutyMax) {
    /* Sum: '<S9>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyC < FOC_Native_DutyMin) {
    /* Sum: '<S9>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S9>/Duty_C_Limit' */

  /* Gain: '<S8>/KiTs' incorporates:
   *  Gain: '<S4>/KiTs'
   */
  rtb_Valpha_Sum = FOC_Native_KiCurrent * FOC_Native_CurrentPeriod;

  /* Sum: '<S8>/Integrator_Add' incorporates:
   *  Gain: '<S8>/KiTs'
   *  UnitDelay: '<S8>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d += rtb_Valpha_Sum *
    rtb_Current_Error_a;

  /* Saturate: '<S8>/Integrator_Limit' */
  if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S8>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S8>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S8>/Integrator_Limit' */

  /* Sum: '<S4>/Integrator_Add' incorporates:
   *  Gain: '<S4>/KiTs'
   *  UnitDelay: '<S4>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_Valpha_Sum *
    rtb_Current_Error;

  /* Saturate: '<S4>/Integrator_Limit' */
  if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S4>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S4>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S4>/Integrator_Limit' */

  /* Update for RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* UnitDelay: '<S10>/Integrator_State' */
    rtb_Current_Error =
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f;

    /* Gain: '<S10>/RpmToRad' incorporates:
     *  Inport: '<Root>/SpeedReferenceRpm'
     *  Inport: '<Root>/SpeedRpm'
     *  Sum: '<S10>/Speed_Error'
     */
    rtb_Current_Error_a = (PMSM_FOC_DualPlant_Controller_U.SpeedReferenceRpm -
      PMSM_FOC_DualPlant_Controller_U.SpeedRpm) *
      PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* Sum: '<S10>/Integrator_Add' incorporates:
     *  Gain: '<S10>/KiTs'
     *  UnitDelay: '<S10>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f +=
      FOC_Native_KiSpeed * FOC_Native_SpeedPeriod * rtb_Current_Error_a;

    /* Saturate: '<S10>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f >
        FOC_Native_IqLimit) {
      /* Sum: '<S10>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f =
        FOC_Native_IqLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f <
               -FOC_Native_IqLimit) {
      /* Sum: '<S10>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f =
        -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S10>/Integrator_Limit' */

    /* Sum: '<S10>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S10>/Kp'
     */
    rtb_Current_Error += FOC_Native_KpSpeed * rtb_Current_Error_a;

    /* Saturate: '<S10>/Iq_Reference_Limit' */
    if (rtb_Current_Error > FOC_Native_IqLimit) {
      rtb_Current_Error = FOC_Native_IqLimit;
    } else if (rtb_Current_Error < -FOC_Native_IqLimit) {
      rtb_Current_Error = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S10>/Iq_Reference_Limit' */
    PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 =
      rtb_Current_Error;
  }

  /* End of Update for RateTransition: '<Root>/IqRef_Rate_Transition' */
  rate_scheduler();
}

/* Model initialize function */
void PMSM_FOC_DualPlant_Controller_v21_initialize(void)
{
  /* Start for Outport: '<Root>/IqReference' incorporates:
   *  RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  PMSM_FOC_DualPlant_Controller_Y.IqReference =
    PMSM_FOC_DualPlant_Controller_P.IqRef_Rate_Transition_InitialCo;

  /* InitializeConditions for Sum: '<S4>/Integrator_Add' incorporates:
   *  UnitDelay: '<S4>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialConditi;

  /* InitializeConditions for RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.IqRef_Rate_Transition_InitialCo;

  /* InitializeConditions for Sum: '<S8>/Integrator_Add' incorporates:
   *  UnitDelay: '<S8>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_m;

  /* InitializeConditions for Sum: '<S10>/Integrator_Add' incorporates:
   *  UnitDelay: '<S10>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_f =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_g;
}

/* Model terminate function */
void PMSM_FOC_DualPlant_Controller_v21_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
