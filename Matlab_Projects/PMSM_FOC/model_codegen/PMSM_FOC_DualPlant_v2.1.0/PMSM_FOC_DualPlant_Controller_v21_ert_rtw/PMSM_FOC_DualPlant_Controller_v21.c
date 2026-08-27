/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Aug 27 15:35:17 2026
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
                                   *   '<S5>/Integrator_Limit'
                                   *   '<S9>/Integrator_Limit'
                                   */
real32_T FOC_Native_CurrentPeriod = 0.0001F;/* Variable: FOC_Native_CurrentPeriod
                                             * Referenced by:
                                             *   '<S5>/KiTs'
                                             *   '<S9>/KiTs'
                                             */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S10>/Duty_A_Limit'
                                        *   '<S10>/Duty_B_Limit'
                                        *   '<S10>/Duty_C_Limit'
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S10>/Duty_A_Limit'
                                        *   '<S10>/Duty_B_Limit'
                                        *   '<S10>/Duty_C_Limit'
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S3>/Flux_PM'
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S11>/Integrator_Limit'
                                        *   '<S11>/Iq_Reference_Limit'
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S5>/KiTs'
                                        *   '<S9>/KiTs'
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S11>/KiTs'
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S5>/Kp'
                                        *   '<S9>/Kp'
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S11>/Kp'
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S3>/Ld_x_Id'
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S3>/D_Decoupling'
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S3>/Electrical_Speed'
                                        */
real32_T FOC_Native_SpeedPeriod = 0.001F;/* Variable: FOC_Native_SpeedPeriod
                                          * Referenced by: '<S11>/KiTs'
                                          */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S4>/Vd_Limit'
                                          *   '<S4>/Vq_Limit'
                                          */

/* Block signals (default storage) */
B_PMSM_FOC_DualPlant_Controll_T PMSM_FOC_DualPlant_Controller_B;

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
  real32_T rtb_Current_Error_d;
  real32_T rtb_Integrator_Add_g;
  real32_T rtb_Phase_Vb;
  real32_T rtb_Phase_Vc;
  real32_T rtb_RpmToRad;
  real32_T rtb_Valpha_Sum;

  /* Trigonometry: '<S7>/CosTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Trigonometry: '<S8>/CosTheta'
   */
  rtb_Phase_Vc = cosf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Trigonometry: '<S8>/SinTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Trigonometry: '<S7>/SinTheta'
   */
  rtb_Integrator_Add_g = sinf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Sum: '<S8>/Iq_Sum' incorporates:
   *  Gain: '<S2>/Ib_x2'
   *  Gain: '<S2>/InvSqrt3'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Sum: '<S2>/Ia_Plus_2Ib'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured =
    (PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain *
     PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB +
     PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S8>/Id_Sum' incorporates:
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S8>/Id_CosAlpha'
   *  Product: '<S8>/Id_SinBeta'
   *  Trigonometry: '<S8>/SinTheta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IdMeasured = rtb_Phase_Vc *
    PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA + rtb_Integrator_Add_g *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Sum: '<S5>/Current_Error' incorporates:
   *  Constant: '<S1>/Id_Reference_Zero'
   */
  rtb_Current_Error = PMSM_FOC_DualPlant_Controller_P.Id_Reference_Zero_Value -
    PMSM_FOC_DualPlant_Controller_Y.IdMeasured;

  /* Saturate: '<S4>/Vq_Limit' incorporates:
   *  Gain: '<S3>/Electrical_Speed'
   *  Inport: '<Root>/SpeedRpm'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand =
    PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S * FOC_Native_PolePairs *
    PMSM_FOC_DualPlant_Controller_U.SpeedRpm;

  /* Sum: '<S8>/Iq_Sum' incorporates:
   *  Gain: '<S8>/Negative'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Product: '<S8>/Iq_CosBeta'
   *  Product: '<S8>/Iq_SinAlpha'
   *  Trigonometry: '<S8>/SinTheta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured = rtb_Integrator_Add_g *
    PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_Phase_Vc *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Saturate: '<S4>/Vd_Limit' incorporates:
   *  Gain: '<S3>/D_Decoupling'
   *  Gain: '<S5>/Kp'
   *  Product: '<S3>/Omega_x_Iq'
   *  Sum: '<S4>/Vd_Raw'
   *  Sum: '<S5>/PI_Sum'
   *  UnitDelay: '<S5>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.VdCommand = (FOC_Native_KpCurrent *
    rtb_Current_Error + PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE)
    + PMSM_FOC_DualPlant_Controller_Y.VqCommand *
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured * -FOC_Native_Lq;

  /* Saturate: '<S4>/Vd_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.VdCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S4>/Vd_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_DualPlant_Controller_Y.VdCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S4>/Vd_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S4>/Vd_Limit' */

  /* ZeroOrderHold: '<S11>/SpeedRef_1ms' */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S11>/RpmToRad' incorporates:
     *  Inport: '<Root>/SpeedReferenceRpm'
     *  Inport: '<Root>/SpeedRpm'
     *  Sum: '<S11>/Speed_Error'
     */
    rtb_RpmToRad = (PMSM_FOC_DualPlant_Controller_U.SpeedReferenceRpm -
                    PMSM_FOC_DualPlant_Controller_U.SpeedRpm) *
      PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* Sum: '<S11>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S11>/Kp'
     *  UnitDelay: '<S11>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit = FOC_Native_KpSpeed *
      rtb_RpmToRad + PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m;

    /* Saturate: '<S11>/Iq_Reference_Limit' */
    if (PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit > FOC_Native_IqLimit)
    {
      /* Sum: '<S11>/Iq_Reference_Sum' incorporates:
       *  Saturate: '<S11>/Iq_Reference_Limit'
       */
      PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit = FOC_Native_IqLimit;
    } else if (PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit <
               -FOC_Native_IqLimit) {
      /* Sum: '<S11>/Iq_Reference_Sum' incorporates:
       *  Saturate: '<S11>/Iq_Reference_Limit'
       */
      PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S11>/Iq_Reference_Limit' */
  }

  /* End of ZeroOrderHold: '<S11>/SpeedRef_1ms' */

  /* Sum: '<S9>/Current_Error' incorporates:
   *  ZeroOrderHold: '<S11>/IqRef_100us'
   */
  rtb_Current_Error_d = PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit -
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Saturate: '<S4>/Vq_Limit' incorporates:
   *  Constant: '<S3>/Flux_PM'
   *  Gain: '<S3>/Ld_x_Id'
   *  Gain: '<S9>/Kp'
   *  Product: '<S3>/Q_Feedforward'
   *  Sum: '<S3>/Flux_Linkage'
   *  Sum: '<S4>/Vq_Raw'
   *  Sum: '<S9>/PI_Sum'
   *  UnitDelay: '<S9>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand = (FOC_Native_Ld *
    PMSM_FOC_DualPlant_Controller_Y.IdMeasured + FOC_Native_FluxPM) *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand + (FOC_Native_KpCurrent *
    rtb_Current_Error_d +
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o);

  /* Saturate: '<S4>/Vq_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.VqCommand > FOC_Native_VoltageLimit) {
    /* Saturate: '<S4>/Vq_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = FOC_Native_VoltageLimit;
  } else if (PMSM_FOC_DualPlant_Controller_Y.VqCommand <
             -FOC_Native_VoltageLimit) {
    /* Saturate: '<S4>/Vq_Limit' */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = -FOC_Native_VoltageLimit;
  }

  /* End of Saturate: '<S4>/Vq_Limit' */

  /* Sum: '<S7>/Valpha_Sum' incorporates:
   *  Product: '<S7>/Valpha_CosVd'
   *  Product: '<S7>/Valpha_SinVq'
   *  Trigonometry: '<S7>/CosTheta'
   */
  rtb_Valpha_Sum = rtb_Phase_Vc * PMSM_FOC_DualPlant_Controller_Y.VdCommand -
    rtb_Integrator_Add_g * PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S7>/Vbeta_Sum' incorporates:
   *  Product: '<S7>/Vbeta_CosVq'
   *  Product: '<S7>/Vbeta_SinVd'
   *  Trigonometry: '<S7>/CosTheta'
   */
  rtb_Phase_Vc = rtb_Integrator_Add_g *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand + rtb_Phase_Vc *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S6>/Phase_Vb' incorporates:
   *  Gain: '<S6>/Vb_Alpha'
   *  Gain: '<S6>/Vb_Beta'
   */
  rtb_Phase_Vb = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain * rtb_Valpha_Sum
    + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Sum: '<S6>/Phase_Vc' incorporates:
   *  Gain: '<S6>/Vc_Alpha'
   *  Gain: '<S6>/Vc_Beta'
   */
  rtb_Phase_Vc = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain * rtb_Valpha_Sum
    + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 * rtb_Phase_Vc;

  /* Gain: '<S10>/Common_Mode' incorporates:
   *  MinMax: '<S10>/Phase_Maximum'
   *  MinMax: '<S10>/Phase_Minimum'
   *  Sum: '<S10>/Max_Plus_Min'
   */
  rtb_Integrator_Add_g = (fmaxf(fmaxf(rtb_Valpha_Sum, rtb_Phase_Vb),
    rtb_Phase_Vc) + fminf(fminf(rtb_Valpha_Sum, rtb_Phase_Vb), rtb_Phase_Vc)) *
    PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* Sum: '<S10>/Duty_B_Plus_Half' incorporates:
   *  Constant: '<S10>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S10>/Duty_B_Divide_Vdc'
   *  Sum: '<S10>/Phase_B_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyB = (rtb_Phase_Vb + rtb_Integrator_Add_g) /
    PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S10>/Duty_B_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyB > FOC_Native_DutyMax) {
    /* Sum: '<S10>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyB < FOC_Native_DutyMin) {
    /* Sum: '<S10>/Duty_B_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S10>/Duty_B_Limit' */

  /* Sum: '<S10>/Duty_A_Plus_Half' incorporates:
   *  Constant: '<S10>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S10>/Duty_A_Divide_Vdc'
   *  Sum: '<S10>/Phase_A_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyA = (rtb_Valpha_Sum + rtb_Integrator_Add_g)
    / PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S10>/Duty_A_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyA > FOC_Native_DutyMax) {
    /* Sum: '<S10>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyA < FOC_Native_DutyMin) {
    /* Sum: '<S10>/Duty_A_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S10>/Duty_A_Limit' */

  /* Sum: '<S10>/Duty_C_Plus_Half' incorporates:
   *  Constant: '<S10>/Duty_Half'
   *  Inport: '<Root>/DcBusVoltage'
   *  Product: '<S10>/Duty_C_Divide_Vdc'
   *  Sum: '<S10>/Phase_C_Plus_Common'
   */
  PMSM_FOC_DualPlant_Controller_Y.DutyC = (rtb_Phase_Vc + rtb_Integrator_Add_g) /
    PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S10>/Duty_C_Limit' */
  if (PMSM_FOC_DualPlant_Controller_Y.DutyC > FOC_Native_DutyMax) {
    /* Sum: '<S10>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMax;
  } else if (PMSM_FOC_DualPlant_Controller_Y.DutyC < FOC_Native_DutyMin) {
    /* Sum: '<S10>/Duty_C_Plus_Half' incorporates:
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMin;
  }

  /* End of Saturate: '<S10>/Duty_C_Limit' */

  /* Gain: '<S9>/KiTs' incorporates:
   *  Gain: '<S5>/KiTs'
   */
  rtb_Valpha_Sum = FOC_Native_KiCurrent * FOC_Native_CurrentPeriod;

  /* Sum: '<S9>/Integrator_Add' incorporates:
   *  Gain: '<S9>/KiTs'
   *  UnitDelay: '<S9>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o += rtb_Valpha_Sum *
    rtb_Current_Error_d;

  /* Saturate: '<S9>/Integrator_Limit' */
  if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S9>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S9>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S9>/Integrator_Limit' */

  /* Outport: '<Root>/IqReference' incorporates:
   *  ZeroOrderHold: '<S11>/IqRef_100us'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqReference =
    PMSM_FOC_DualPlant_Controller_B.Iq_Reference_Limit;
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Sum: '<S11>/Integrator_Add' incorporates:
     *  Gain: '<S11>/KiTs'
     *  UnitDelay: '<S11>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m +=
      FOC_Native_KiSpeed * FOC_Native_SpeedPeriod * rtb_RpmToRad;

    /* Saturate: '<S11>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m >
        FOC_Native_IqLimit) {
      /* Sum: '<S11>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
        FOC_Native_IqLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m <
               -FOC_Native_IqLimit) {
      /* Sum: '<S11>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
        -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S11>/Integrator_Limit' */
  }

  /* Sum: '<S5>/Integrator_Add' incorporates:
   *  Gain: '<S5>/KiTs'
   *  UnitDelay: '<S5>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_Valpha_Sum *
    rtb_Current_Error;

  /* Saturate: '<S5>/Integrator_Limit' */
  if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE >
      FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S5>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      FOC_Native_CurrentIntegratorLimit;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE <
             -FOC_Native_CurrentIntegratorLimit) {
    /* Sum: '<S5>/Integrator_Add' */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      -FOC_Native_CurrentIntegratorLimit;
  }

  /* End of Saturate: '<S5>/Integrator_Limit' */
  rate_scheduler();
}

/* Model initialize function */
void PMSM_FOC_DualPlant_Controller_v21_initialize(void)
{
  /* InitializeConditions for Sum: '<S5>/Integrator_Add' incorporates:
   *  UnitDelay: '<S5>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialConditi;

  /* InitializeConditions for Sum: '<S11>/Integrator_Add' incorporates:
   *  UnitDelay: '<S11>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_g;

  /* InitializeConditions for Sum: '<S9>/Integrator_Add' incorporates:
   *  UnitDelay: '<S9>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_b;
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
