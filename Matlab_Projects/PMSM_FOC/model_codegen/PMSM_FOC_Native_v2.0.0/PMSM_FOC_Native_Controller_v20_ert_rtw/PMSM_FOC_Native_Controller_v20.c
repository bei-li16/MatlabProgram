/*
 * File: PMSM_FOC_Native_Controller_v20.c
 *
 * Code generated for Simulink model 'PMSM_FOC_Native_Controller_v20'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Aug 27 08:58:26 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PMSM_FOC_Native_Controller_v20.h"
#include <math.h>
#include "rtwtypes.h"

/* Exported block parameters */
real32_T FOC_Native_CurrentIntegratorLimit = 30.0F;
                                  /* Variable: FOC_Native_CurrentIntegratorLimit
                                   * Referenced by:
                                   *   '<S1>/Id_Integrator_Limit'
                                   *   '<S1>/Iq_Integrator_Limit'
                                   */
real32_T FOC_Native_CurrentPeriod = 0.0001F;/* Variable: FOC_Native_CurrentPeriod
                                             * Referenced by:
                                             *   '<S1>/Id_KiTs'
                                             *   '<S1>/Iq_KiTs'
                                             */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S1>/Duty_A_Limit'
                                        *   '<S1>/Duty_B_Limit'
                                        *   '<S1>/Duty_C_Limit'
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S1>/Duty_A_Limit'
                                        *   '<S1>/Duty_B_Limit'
                                        *   '<S1>/Duty_C_Limit'
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S1>/Flux_PM'
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S1>/Iq_Reference_Limit'
                                        *   '<S1>/Speed_Integrator_Limit'
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S1>/Id_KiTs'
                                        *   '<S1>/Iq_KiTs'
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S1>/Speed_KiTs'
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S1>/Id_Kp'
                                        *   '<S1>/Iq_Kp'
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S1>/Speed_Kp'
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S1>/Ld_x_Id'
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S1>/D_Decoupling'
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S1>/Electrical_Speed'
                                        */
real32_T FOC_Native_SpeedPeriod = 0.001F;/* Variable: FOC_Native_SpeedPeriod
                                          * Referenced by: '<S1>/Speed_KiTs'
                                          */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S1>/Vd_Limit'
                                          *   '<S1>/Vq_Limit'
                                          */

/* Block signals (default storage) */
B_PMSM_FOC_Native_Controller__T PMSM_FOC_Native_Controller_v2_B;

/* Block states (default storage) */
DW_PMSM_FOC_Native_Controller_T PMSM_FOC_Native_Controller_v_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PMSM_FOC_Native_Controll_T PMSM_FOC_Native_Controller_v2_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PMSM_FOC_Native_Controll_T PMSM_FOC_Native_Controller_v2_Y;

/* Real-time model */
static RT_MODEL_PMSM_FOC_Native_Cont_T PMSM_FOC_Native_Controller_v_M_;
RT_MODEL_PMSM_FOC_Native_Cont_T *const PMSM_FOC_Native_Controller_v_M =
  &PMSM_FOC_Native_Controller_v_M_;
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
  (PMSM_FOC_Native_Controller_v_M->Timing.TaskCounters.TID[1])++;
  if ((PMSM_FOC_Native_Controller_v_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    PMSM_FOC_Native_Controller_v_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
void PMSM_FOC_Native_Controller_v20_step(void)
{
  real32_T rtb_Id_Error;
  real32_T rtb_Iq_Error;
  real32_T rtb_Iq_Integrator_Add;
  real32_T rtb_Phase_Vb;
  real32_T rtb_Phase_Vc;
  real32_T rtb_Speed_RpmToRad;
  real32_T rtb_Valpha_Sum;

  /* Trigonometry: '<S1>/CosTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  rtb_Phase_Vb = cosf(PMSM_FOC_Native_Controller_v2_U.ElectricalAngleRad);

  /* Trigonometry: '<S1>/SinTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  rtb_Phase_Vc = sinf(PMSM_FOC_Native_Controller_v2_U.ElectricalAngleRad);

  /* Sum: '<S1>/Iq_Sum' incorporates:
   *  Gain: '<S1>/Ib_x2'
   *  Gain: '<S1>/InvSqrt3'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Sum: '<S1>/Clarke_Sum'
   */
  PMSM_FOC_Native_Controller_v2_Y.IqMeasured =
    (PMSM_FOC_Native_Controller_v2_P.Ib_x2_Gain *
     PMSM_FOC_Native_Controller_v2_U.PhaseCurrentB +
     PMSM_FOC_Native_Controller_v2_U.PhaseCurrentA) *
    PMSM_FOC_Native_Controller_v2_P.NATIVE_INV_SQRT3;

  /* Sum: '<S1>/Id_Sum' incorporates:
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S1>/Id_CosAlpha'
   *  Product: '<S1>/Id_SinBeta'
   */
  PMSM_FOC_Native_Controller_v2_Y.IdMeasured = rtb_Phase_Vb *
    PMSM_FOC_Native_Controller_v2_U.PhaseCurrentA + rtb_Phase_Vc *
    PMSM_FOC_Native_Controller_v2_Y.IqMeasured;

  /* Saturate: '<S1>/Vd_Limit' incorporates:
   *  Gain: '<S1>/Electrical_Speed'
   *  Inport: '<Root>/SpeedRpm'
   */
  PMSM_FOC_Native_Controller_v2_Y.VdCommand =
    PMSM_FOC_Native_Controller_v2_P.NATIVE_RPM_TO_RAD_S * FOC_Native_PolePairs *
    PMSM_FOC_Native_Controller_v2_U.SpeedRpm;

  /* Sum: '<S1>/Iq_Sum' incorporates:
   *  Gain: '<S1>/Iq_Negative'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S1>/Iq_CosBeta'
   *  Product: '<S1>/Iq_SinAlpha'
   */
  PMSM_FOC_Native_Controller_v2_Y.IqMeasured = rtb_Phase_Vc *
    PMSM_FOC_Native_Controller_v2_U.PhaseCurrentA *
    PMSM_FOC_Native_Controller_v2_P.Iq_Negative_Gain + rtb_Phase_Vb *
    PMSM_FOC_Native_Controller_v2_Y.IqMeasured;

  /* ZeroOrderHold: '<S1>/SpeedRef_1ms' */
  if (PMSM_FOC_Native_Controller_v_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S1>/Speed_RpmToRad' incorporates:
     *  Inport: '<Root>/SpeedReferenceRpm'
     *  Inport: '<Root>/SpeedRpm'
     *  Sum: '<S1>/Speed_Error'
     */
    rtb_Speed_RpmToRad = (PMSM_FOC_Native_Controller_v2_U.SpeedReferenceRpm -
                          PMSM_FOC_Native_Controller_v2_U.SpeedRpm) *
      PMSM_FOC_Native_Controller_v2_P.NATIVE_RPM_TO_RAD_S;

    /* Sum: '<S1>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S1>/Speed_Kp'
     *  UnitDelay: '<S1>/Speed_Integrator_State'
     */
    PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit = FOC_Native_KpSpeed *
      rtb_Speed_RpmToRad +
      PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE;

    /* Saturate: '<S1>/Iq_Reference_Limit' */
    if (PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit > FOC_Native_IqLimit)
    {
      /* Sum: '<S1>/Iq_Reference_Sum' incorporates:
       *  Saturate: '<S1>/Iq_Reference_Limit'
       */
      PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit = FOC_Native_IqLimit;
    } else if (PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit <
               -FOC_Native_IqLimit) {
      /* Sum: '<S1>/Iq_Reference_Sum' incorporates:
       *  Saturate: '<S1>/Iq_Reference_Limit'
       */
      PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S1>/Iq_Reference_Limit' */
  }

  /* End of ZeroOrderHold: '<S1>/SpeedRef_1ms' */

  /* Sum: '<S1>/Iq_Error' incorporates:
   *  ZeroOrderHold: '<S1>/Iq_Reference_100us'
   */
  rtb_Iq_Error = PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit -
    PMSM_FOC_Native_Controller_v2_Y.IqMeasured;

  /* Saturate: '<S1>/Vq_Limit' incorporates:
   *  Constant: '<S1>/Flux_PM'
   *  Gain: '<S1>/Iq_Kp'
   *  Gain: '<S1>/Ld_x_Id'
   *  Product: '<S1>/Q_Feedforward'
   *  Sum: '<S1>/Flux_Linkage'
   *  Sum: '<S1>/Vq_Raw'
   *  UnitDelay: '<S1>/Iq_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v2_Y.VqCommand = (FOC_Native_Ld *
    PMSM_FOC_Native_Controller_v2_Y.IdMeasured + FOC_Native_FluxPM) *
    PMSM_FOC_Native_Controller_v2_Y.VdCommand + (FOC_Native_KpCurrent *
    rtb_Iq_Error + PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE);

  /* Saturate: '<S1>/Vq_Limit' */
  if (PMSM_FOC_Native_Controller_v2_Y.VqCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S1>/Vq_Limit' */
    PMSM_FOC_Native_Controller_v2_Y.VqCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_Native_Controller_v2_Y.VqCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S1>/Vq_Limit' */
    PMSM_FOC_Native_Controller_v2_Y.VqCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S1>/Vq_Limit' */

  /* Sum: '<S1>/Id_Error' incorporates:
   *  Constant: '<S1>/Id_Reference_Zero'
   */
  rtb_Id_Error = PMSM_FOC_Native_Controller_v2_P.Id_Reference_Zero_Value -
    PMSM_FOC_Native_Controller_v2_Y.IdMeasured;

  /* Saturate: '<S1>/Vd_Limit' incorporates:
   *  Gain: '<S1>/D_Decoupling'
   *  Gain: '<S1>/Id_Kp'
   *  Product: '<S1>/Omega_x_Iq'
   *  Sum: '<S1>/Vd_Raw'
   *  UnitDelay: '<S1>/Id_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v2_Y.VdCommand = (FOC_Native_KpCurrent *
    rtb_Id_Error + PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE) +
    PMSM_FOC_Native_Controller_v2_Y.VdCommand *
    PMSM_FOC_Native_Controller_v2_Y.IqMeasured * -FOC_Native_Lq;

  /* Saturate: '<S1>/Vd_Limit' */
  if (PMSM_FOC_Native_Controller_v2_Y.VdCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S1>/Vd_Limit' */
    PMSM_FOC_Native_Controller_v2_Y.VdCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_Native_Controller_v2_Y.VdCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S1>/Vd_Limit' */
    PMSM_FOC_Native_Controller_v2_Y.VdCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S1>/Vd_Limit' */

  /* Sum: '<S1>/Valpha_Sum' incorporates:
   *  Product: '<S1>/Valpha_CosVd'
   *  Product: '<S1>/Valpha_SinVq'
   */
  rtb_Valpha_Sum = rtb_Phase_Vb * PMSM_FOC_Native_Controller_v2_Y.VdCommand -
    rtb_Phase_Vc * PMSM_FOC_Native_Controller_v2_Y.VqCommand;

  /* Sum: '<S1>/Vbeta_Sum' incorporates:
   *  Product: '<S1>/Vbeta_CosVq'
   *  Product: '<S1>/Vbeta_SinVd'
   */
  rtb_Phase_Vc = rtb_Phase_Vc * PMSM_FOC_Native_Controller_v2_Y.VdCommand +
    rtb_Phase_Vb * PMSM_FOC_Native_Controller_v2_Y.VqCommand;

  /* Sum: '<S1>/Phase_Vb' incorporates:
   *  Gain: '<S1>/Vb_Alpha'
   *  Gain: '<S1>/Vb_Beta'
   */
  rtb_Phase_Vb = PMSM_FOC_Native_Controller_v2_P.Vb_Alpha_Gain * rtb_Valpha_Sum
    + PMSM_FOC_Native_Controller_v2_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Sum: '<S1>/Phase_Vc' incorporates:
   *  Gain: '<S1>/Vc_Alpha'
   *  Gain: '<S1>/Vc_Beta'
   */
  rtb_Phase_Vc = PMSM_FOC_Native_Controller_v2_P.Vc_Alpha_Gain * rtb_Valpha_Sum
    + -PMSM_FOC_Native_Controller_v2_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Gain: '<S1>/Common_Mode' incorporates:
   *  MinMax: '<S1>/Phase_Maximum'
   *  MinMax: '<S1>/Phase_Minimum'
   *  Sum: '<S1>/Max_Plus_Min'
   */
  rtb_Iq_Integrator_Add = (fmaxf(fmaxf(rtb_Valpha_Sum, rtb_Phase_Vb),
    rtb_Phase_Vc) + fminf(fminf(rtb_Valpha_Sum, rtb_Phase_Vb), rtb_Phase_Vc)) *
    PMSM_FOC_Native_Controller_v2_P.Common_Mode_Gain;

  /* Sum: '<S1>/Duty_B_Plus_Half' incorporates:
   *  Constant: '<S1>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S1>/Duty_B_Divide_Vdc'
   *  Sum: '<S1>/Phase_B_Plus_Common'
   */
  PMSM_FOC_Native_Controller_v2_Y.DutyB = (rtb_Phase_Vb + rtb_Iq_Integrator_Add)
    / PMSM_FOC_Native_Controller_v2_U.DcBusVoltage +
    PMSM_FOC_Native_Controller_v2_P.Duty_Half_Value;

  /* Saturate: '<S1>/Duty_B_Limit' */
  if (PMSM_FOC_Native_Controller_v2_Y.DutyB > FOC_Native_DutyMax) {
    /* Sum: '<S1>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyB = FOC_Native_DutyMax;
  } else if (PMSM_FOC_Native_Controller_v2_Y.DutyB < FOC_Native_DutyMin) {
    /* Sum: '<S1>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyB = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S1>/Duty_B_Limit' */

  /* Sum: '<S1>/Duty_A_Plus_Half' incorporates:
   *  Constant: '<S1>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S1>/Duty_A_Divide_Vdc'
   *  Sum: '<S1>/Phase_A_Plus_Common'
   */
  PMSM_FOC_Native_Controller_v2_Y.DutyA = (rtb_Valpha_Sum +
    rtb_Iq_Integrator_Add) / PMSM_FOC_Native_Controller_v2_U.DcBusVoltage +
    PMSM_FOC_Native_Controller_v2_P.Duty_Half_Value;

  /* Saturate: '<S1>/Duty_A_Limit' */
  if (PMSM_FOC_Native_Controller_v2_Y.DutyA > FOC_Native_DutyMax) {
    /* Sum: '<S1>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyA = FOC_Native_DutyMax;
  } else if (PMSM_FOC_Native_Controller_v2_Y.DutyA < FOC_Native_DutyMin) {
    /* Sum: '<S1>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyA = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S1>/Duty_A_Limit' */

  /* Sum: '<S1>/Duty_C_Plus_Half' incorporates:
   *  Constant: '<S1>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S1>/Duty_C_Divide_Vdc'
   *  Sum: '<S1>/Phase_C_Plus_Common'
   */
  PMSM_FOC_Native_Controller_v2_Y.DutyC = (rtb_Phase_Vc + rtb_Iq_Integrator_Add)
    / PMSM_FOC_Native_Controller_v2_U.DcBusVoltage +
    PMSM_FOC_Native_Controller_v2_P.Duty_Half_Value;

  /* Saturate: '<S1>/Duty_C_Limit' */
  if (PMSM_FOC_Native_Controller_v2_Y.DutyC > FOC_Native_DutyMax) {
    /* Sum: '<S1>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyC = FOC_Native_DutyMax;
  } else if (PMSM_FOC_Native_Controller_v2_Y.DutyC < FOC_Native_DutyMin) {
    /* Sum: '<S1>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_Native_Controller_v2_Y.DutyC = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S1>/Duty_C_Limit' */

  /* Gain: '<S1>/Id_KiTs' incorporates:
   *  Gain: '<S1>/Iq_KiTs'
   */
  rtb_Phase_Vc = FOC_Native_KiCurrent * FOC_Native_CurrentPeriod;

  /* Sum: '<S1>/Id_Integrator_Add' incorporates:
   *  Gain: '<S1>/Id_KiTs'
   *  UnitDelay: '<S1>/Id_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE += rtb_Phase_Vc *
    rtb_Id_Error;

  /* Saturate: '<S1>/Id_Integrator_Limit' */
  if (PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S1>/Id_Integrator_Add' */
    PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S1>/Id_Integrator_Add' */
    PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S1>/Id_Integrator_Limit' */

  /* Sum: '<S1>/Iq_Integrator_Add' incorporates:
   *  Gain: '<S1>/Iq_KiTs'
   *  UnitDelay: '<S1>/Iq_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE += rtb_Phase_Vc *
    rtb_Iq_Error;

  /* Saturate: '<S1>/Iq_Integrator_Limit' */
  if (PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S1>/Iq_Integrator_Add' */
    PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S1>/Iq_Integrator_Add' */
    PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S1>/Iq_Integrator_Limit' */

  /* Outport: '<Root>/IqReference' incorporates:
   *  ZeroOrderHold: '<S1>/Iq_Reference_100us'
   */
  PMSM_FOC_Native_Controller_v2_Y.IqReference =
    PMSM_FOC_Native_Controller_v2_B.Iq_Reference_Limit;
  if (PMSM_FOC_Native_Controller_v_M->Timing.TaskCounters.TID[1] == 0) {
    /* Sum: '<S1>/Speed_Integrator_Add' incorporates:
     *  Gain: '<S1>/Speed_KiTs'
     *  UnitDelay: '<S1>/Speed_Integrator_State'
     */
    PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE +=
      FOC_Native_KiSpeed * FOC_Native_SpeedPeriod * rtb_Speed_RpmToRad;

    /* Saturate: '<S1>/Speed_Integrator_Limit' */
    if (PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE >
        FOC_Native_IqLimit) {
      /* Sum: '<S1>/Speed_Integrator_Add' */
      PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE =
        FOC_Native_IqLimit;
    } else if (PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE <
               -FOC_Native_IqLimit) {
      /* Sum: '<S1>/Speed_Integrator_Add' */
      PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE =
        -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S1>/Speed_Integrator_Limit' */
  }

  rate_scheduler();
}

/* Model initialize function */
void PMSM_FOC_Native_Controller_v20_initialize(void)
{
  /* InitializeConditions for Sum: '<S1>/Speed_Integrator_Add' incorporates:
   *  UnitDelay: '<S1>/Speed_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v_DW.Speed_Integrator_State_DSTATE =
    PMSM_FOC_Native_Controller_v2_P.Speed_Integrator_State_InitialC;

  /* InitializeConditions for Sum: '<S1>/Iq_Integrator_Add' incorporates:
   *  UnitDelay: '<S1>/Iq_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v_DW.Iq_Integrator_State_DSTATE =
    PMSM_FOC_Native_Controller_v2_P.Iq_Integrator_State_InitialCond;

  /* InitializeConditions for Sum: '<S1>/Id_Integrator_Add' incorporates:
   *  UnitDelay: '<S1>/Id_Integrator_State'
   */
  PMSM_FOC_Native_Controller_v_DW.Id_Integrator_State_DSTATE =
    PMSM_FOC_Native_Controller_v2_P.Id_Integrator_State_InitialCond;
}

/* Model terminate function */
void PMSM_FOC_Native_Controller_v20_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
