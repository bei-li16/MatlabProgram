/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
 *
 * Code generated for Simulink model 'PMSM_FOC_DualPlant_Controller_v21'.
 *
 * Model version                  : 1.33
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Fri Aug 28 17:17:17 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PMSM_FOC_DualPlant_Controller_v21.h"
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<Root>/Motor_State_Machine_100us' */
#define PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define PMSM_FOC_DualPlan_IN_SUPERVISED ((uint8_T)2U)
#define PMSM_FOC_DualPlant_Con_IN_ALIGN ((uint8_T)1U)
#define PMSM_FOC_DualPlant_Con_IN_CALIB ((uint8_T)2U)
#define PMSM_FOC_DualPlant_Con_IN_FAULT ((uint8_T)1U)
#define PMSM_FOC_DualPlant_Con_IN_READY ((uint8_T)4U)
#define PMSM_FOC_DualPlant_Cont_IN_INIT ((uint8_T)3U)
#define PMSM_FOC_DualPlant_Contr_IN_RUN ((uint8_T)5U)

/* Exported block parameters */
real32_T FOC_Native_CurrentIntegratorLimit = 30.0F;
                                  /* Variable: FOC_Native_CurrentIntegratorLimit
                                   * Referenced by:
                                   *   '<S6>/Integrator_Limit'
                                   *   '<S12>/Integrator_Limit'
                                   */
real32_T FOC_Native_CurrentPeriod = 0.0001F;/* Variable: FOC_Native_CurrentPeriod
                                             * Referenced by:
                                             *   '<S6>/KiTs'
                                             *   '<S12>/KiTs'
                                             */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S13>/Duty_A_Limit'
                                        *   '<S13>/Duty_B_Limit'
                                        *   '<S13>/Duty_C_Limit'
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S13>/Duty_A_Limit'
                                        *   '<S13>/Duty_B_Limit'
                                        *   '<S13>/Duty_C_Limit'
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S4>/Flux_PM'
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S14>/Integrator_Limit'
                                        *   '<S14>/Iq_Reference_Limit'
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S6>/KiTs'
                                        *   '<S12>/KiTs'
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S14>/KiTs'
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S6>/Kp'
                                        *   '<S12>/Kp'
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S14>/Kp'
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S4>/Ld_x_Id'
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S4>/D_Decoupling'
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S4>/Electrical_Speed'
                                        */
real32_T FOC_Native_SpeedPeriod = 0.001F;/* Variable: FOC_Native_SpeedPeriod
                                          * Referenced by: '<S14>/KiTs'
                                          */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S5>/Vd_Limit'
                                          *   '<S5>/Vq_Limit'
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
  real32_T rtb_CosTheta;
  real32_T rtb_Current_Error;
  real32_T rtb_Electrical_Speed;
  real32_T rtb_Phase_Maximum;
  real32_T rtb_Phase_Minimum;
  real32_T rtb_SinTheta;
  boolean_T rtb_Any_Fault;
  boolean_T rtb_Start_Command;

  /* RelationalOperator: '<Root>/Start_Command' incorporates:
   *  Constant: '<Root>/Start_Threshold_Rpm'
   *  Inport: '<Root>/SpeedReferenceRpm'
   */
  rtb_Start_Command = (PMSM_FOC_DualPlant_Controller_U.SpeedReferenceRpm >
                       PMSM_FOC_DualPlant_Controller_P.Start_Threshold_Rpm_Value);

  /* Logic: '<S9>/Any_Fault' incorporates:
   *  Abs: '<S9>/Abs_Ia'
   *  Abs: '<S9>/Abs_Ib'
   *  Abs: '<S9>/Abs_Speed'
   *  Constant: '<S9>/Max_Current_A'
   *  Constant: '<S9>/Max_Speed_Rpm'
   *  Constant: '<S9>/Max_Vdc'
   *  Constant: '<S9>/Min_Vdc'
   *  Inport: '<Root>/DcBusVoltage'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Inport: '<Root>/SpeedRpm'
   *  RelationalOperator: '<S9>/OverCurrentA'
   *  RelationalOperator: '<S9>/OverCurrentB'
   *  RelationalOperator: '<S9>/OverSpeed'
   *  RelationalOperator: '<S9>/OverVoltage'
   *  RelationalOperator: '<S9>/UnderVoltage'
   */
  rtb_Any_Fault = ((fabsf(PMSM_FOC_DualPlant_Controller_U.SpeedRpm) >
                    PMSM_FOC_DualPlant_Controller_P.Max_Speed_Rpm_Value) ||
                   (fabsf(PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA) >
                    PMSM_FOC_DualPlant_Controller_P.Max_Current_A_Value) ||
                   (fabsf(PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB) >
                    PMSM_FOC_DualPlant_Controller_P.Max_Current_A_Value) ||
                   (PMSM_FOC_DualPlant_Controller_U.DcBusVoltage <
                    PMSM_FOC_DualPlant_Controller_P.Min_Vdc_Value) ||
                   (PMSM_FOC_DualPlant_Controller_U.DcBusVoltage >
                    PMSM_FOC_DualPlant_Controller_P.Max_Vdc_Value));

  /* Chart: '<Root>/Motor_State_Machine_100us' incorporates:
   *  Constant: '<S3>/Sample_Target'
   *  Inport: '<Root>/FaultResetAck'
   *  RelationalOperator: '<S3>/Count_Complete'
   *  UnitDelay: '<S3>/Count_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 < 255U) {
    PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1++;
  }

  if (PMSM_FOC_DualPlant_Controlle_DW.is_active_c3_PMSM_FOC_DualPlant == 0U) {
    PMSM_FOC_DualPlant_Controlle_DW.is_active_c3_PMSM_FOC_DualPlant = 1U;
    PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
      PMSM_FOC_DualPlan_IN_SUPERVISED;
    PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
    PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
      PMSM_FOC_DualPlant_Cont_IN_INIT;
    PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
    PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
    PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
    PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
    PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
    PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro ==
             PMSM_FOC_DualPlant_Con_IN_FAULT) {
    if (PMSM_FOC_DualPlant_Controller_U.FaultResetAck && (!rtb_Any_Fault) &&
        (!rtb_Start_Command)) {
      PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
        PMSM_FOC_DualPlan_IN_SUPERVISED;
      PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
      PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
        PMSM_FOC_DualPlant_Cont_IN_INIT;
      PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
      PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
      PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
    }

    /* case IN_SUPERVISED: */
  } else if (rtb_Any_Fault) {
    PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
      PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD;
    PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
      PMSM_FOC_DualPlant_Con_IN_FAULT;
    PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
    PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
    PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
    PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
    PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
    PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
  } else {
    switch (PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED) {
     case PMSM_FOC_DualPlant_Con_IN_ALIGN:
      if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >= 200) {
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Contr_IN_RUN;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = true;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = true;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = false;
      } else if (!rtb_Start_Command) {
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Cont_IN_INIT;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }
      break;

     case PMSM_FOC_DualPlant_Con_IN_CALIB:
      if (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >=
          PMSM_FOC_DualPlant_Controller_P.Sample_Target_Value) {
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Con_IN_ALIGN;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = true;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = true;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      } else if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >= 150) {
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD;
        PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
          PMSM_FOC_DualPlant_Con_IN_FAULT;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }
      break;

     case PMSM_FOC_DualPlant_Cont_IN_INIT:
      PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
        PMSM_FOC_DualPlant_Con_IN_READY;
      PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
      PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
      PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      break;

     case PMSM_FOC_DualPlant_Con_IN_READY:
      if (rtb_Start_Command) {
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Con_IN_CALIB;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = true;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }
      break;

     default:
      /* case IN_RUN: */
      if (!rtb_Start_Command) {
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Cont_IN_INIT;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/Motor_State_Machine_100us' */

  /* Switch: '<S1>/Theta_Select' incorporates:
   *  Constant: '<S1>/Align_Theta_Zero'
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    rtb_SinTheta = PMSM_FOC_DualPlant_Controller_P.Align_Theta_Zero_Value;
  } else {
    rtb_SinTheta = PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad;
  }

  /* End of Switch: '<S1>/Theta_Select' */

  /* Trigonometry: '<S8>/CosTheta' */
  rtb_CosTheta = cosf(rtb_SinTheta);

  /* Trigonometry: '<S11>/CosTheta' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  rtb_Phase_Minimum = cosf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Switch: '<S3>/OffsetA_Valid' incorporates:
   *  Constant: '<S3>/Zero'
   *  Product: '<S3>/OffsetA_Calculate'
   *  RelationalOperator: '<S3>/Count_Positive'
   *  Switch: '<S3>/OffsetB_Valid'
   *  UnitDelay: '<S3>/Count_State'
   *  UnitDelay: '<S3>/SumA_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >
      PMSM_FOC_DualPlant_Controller_P.Zero_Value) {
    rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;

    /* Switch: '<S1>/Vd_Select' incorporates:
     *  Product: '<S3>/OffsetA_Calculate'
     *  Product: '<S3>/OffsetB_Calculate'
     *  UnitDelay: '<S3>/SumA_State'
     *  UnitDelay: '<S3>/SumB_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;
  } else {
    rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controller_P.Zero_Value;

    /* Switch: '<S1>/Vd_Select' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
  }

  /* End of Switch: '<S3>/OffsetA_Valid' */

  /* Sum: '<S3>/Correct_Ia' incorporates:
   *  Inport: '<Root>/PhaseCurrentA'
   */
  rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA -
    rtb_Phase_Maximum;

  /* Switch: '<S1>/Vq_Select' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Trigonometry: '<S11>/SinTheta'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand = sinf
    (PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Switch: '<S1>/Vd_Select' incorporates:
   *  Gain: '<S2>/Ib_x2'
   *  Gain: '<S2>/InvSqrt3'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Sum: '<S2>/Ia_Plus_2Ib'
   *  Sum: '<S3>/Correct_Ib'
   */
  PMSM_FOC_DualPlant_Controller_Y.VdCommand =
    ((PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB -
      PMSM_FOC_DualPlant_Controller_Y.VdCommand) *
     PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain + rtb_Phase_Maximum) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S11>/Id_Sum' incorporates:
   *  Product: '<S11>/Id_CosAlpha'
   *  Product: '<S11>/Id_SinBeta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IdMeasured = rtb_Phase_Minimum *
    rtb_Phase_Maximum + PMSM_FOC_DualPlant_Controller_Y.VqCommand *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand;

  /* Sum: '<S6>/Current_Error' incorporates:
   *  Constant: '<Root>/Id_Reference_Zero'
   *
   * Block description for '<Root>/Id_Reference_Zero':
   *  Field-oriented control d-axis current reference: Id*=0 A.
   */
  rtb_Current_Error = PMSM_FOC_DualPlant_Controller_P.Id_Reference_Zero_Value -
    PMSM_FOC_DualPlant_Controller_Y.IdMeasured;

  /* Gain: '<S4>/Electrical_Speed' incorporates:
   *  Inport: '<Root>/SpeedRpm'
   */
  rtb_Electrical_Speed = PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S *
    FOC_Native_PolePairs * PMSM_FOC_DualPlant_Controller_U.SpeedRpm;

  /* Sum: '<S11>/Iq_Sum' incorporates:
   *  Gain: '<S11>/Negative'
   *  Product: '<S11>/Iq_CosBeta'
   *  Product: '<S11>/Iq_SinAlpha'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured =
    PMSM_FOC_DualPlant_Controller_Y.VqCommand * rtb_Phase_Maximum *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_Phase_Minimum *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand;

  /* Switch: '<S1>/Vd_Select' */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    /* Switch: '<S1>/Vd_Select' incorporates:
     *  Constant: '<S1>/Align_Vd_2V'
     */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controller_P.Align_Vd_2V_Value;
  } else {
    /* Switch: '<S1>/Vd_Select' incorporates:
     *  Gain: '<S4>/D_Decoupling'
     *  Gain: '<S6>/Kp'
     *  Product: '<S4>/Omega_x_Iq'
     *  Sum: '<S5>/Vd_Raw'
     *  Sum: '<S6>/PI_Sum'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand = (FOC_Native_KpCurrent *
      rtb_Current_Error +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE) +
      rtb_Electrical_Speed * PMSM_FOC_DualPlant_Controller_Y.IqMeasured *
      -FOC_Native_Lq;

    /* Saturate: '<S5>/Vd_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.VdCommand > FOC_Native_VoltageLimit) {
      /* Switch: '<S1>/Vd_Select' */
      PMSM_FOC_DualPlant_Controller_Y.VdCommand = FOC_Native_VoltageLimit;
    } else if (PMSM_FOC_DualPlant_Controller_Y.VdCommand <
               -FOC_Native_VoltageLimit) {
      /* Switch: '<S1>/Vd_Select' */
      PMSM_FOC_DualPlant_Controller_Y.VdCommand = -FOC_Native_VoltageLimit;
    }

    /* End of Saturate: '<S5>/Vd_Limit' */
  }

  /* End of Switch: '<S1>/Vd_Select' */

  /* Trigonometry: '<S8>/SinTheta' */
  rtb_SinTheta = sinf(rtb_SinTheta);

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

  /* Sum: '<S12>/Current_Error' incorporates:
   *  Outport: '<Root>/IqReference'
   */
  rtb_Phase_Minimum = PMSM_FOC_DualPlant_Controller_Y.IqReference -
    PMSM_FOC_DualPlant_Controller_Y.IqMeasured;

  /* Switch: '<S1>/Vq_Select' */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    /* Switch: '<S1>/Vq_Select' incorporates:
     *  Constant: '<S1>/Align_Vq_Zero'
     */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand =
      PMSM_FOC_DualPlant_Controller_P.Align_Vq_Zero_Value;
  } else {
    /* Switch: '<S1>/Vq_Select' incorporates:
     *  Constant: '<S4>/Flux_PM'
     *  Gain: '<S12>/Kp'
     *  Gain: '<S4>/Ld_x_Id'
     *  Product: '<S4>/Q_Feedforward'
     *  Sum: '<S12>/PI_Sum'
     *  Sum: '<S4>/Flux_Linkage'
     *  Sum: '<S5>/Vq_Raw'
     *  UnitDelay: '<S12>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = (FOC_Native_Ld *
      PMSM_FOC_DualPlant_Controller_Y.IdMeasured + FOC_Native_FluxPM) *
      rtb_Electrical_Speed + (FOC_Native_KpCurrent * rtb_Phase_Minimum +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c);

    /* Saturate: '<S5>/Vq_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.VqCommand > FOC_Native_VoltageLimit) {
      /* Switch: '<S1>/Vq_Select' */
      PMSM_FOC_DualPlant_Controller_Y.VqCommand = FOC_Native_VoltageLimit;
    } else if (PMSM_FOC_DualPlant_Controller_Y.VqCommand <
               -FOC_Native_VoltageLimit) {
      /* Switch: '<S1>/Vq_Select' */
      PMSM_FOC_DualPlant_Controller_Y.VqCommand = -FOC_Native_VoltageLimit;
    }

    /* End of Saturate: '<S5>/Vq_Limit' */
  }

  /* End of Switch: '<S1>/Vq_Select' */

  /* Sum: '<S8>/Valpha_Sum' incorporates:
   *  Product: '<S8>/Valpha_CosVd'
   *  Product: '<S8>/Valpha_SinVq'
   */
  rtb_Electrical_Speed = rtb_CosTheta *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand - rtb_SinTheta *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S8>/Vbeta_Sum' incorporates:
   *  Product: '<S8>/Vbeta_CosVq'
   *  Product: '<S8>/Vbeta_SinVd'
   */
  rtb_Phase_Maximum = rtb_SinTheta * PMSM_FOC_DualPlant_Controller_Y.VdCommand +
    rtb_CosTheta * PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S7>/Phase_Vb' incorporates:
   *  Gain: '<S7>/Vb_Alpha'
   *  Gain: '<S7>/Vb_Beta'
   */
  rtb_CosTheta = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain *
    rtb_Electrical_Speed + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Phase_Maximum;

  /* Sum: '<S7>/Phase_Vc' incorporates:
   *  Gain: '<S7>/Vc_Alpha'
   *  Gain: '<S7>/Vc_Beta'
   */
  rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain *
    rtb_Electrical_Speed + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Phase_Maximum;

  /* Gain: '<S13>/Common_Mode' incorporates:
   *  MinMax: '<S13>/Phase_Maximum'
   *  MinMax: '<S13>/Phase_Minimum'
   *  Sum: '<S13>/Max_Plus_Min'
   */
  rtb_SinTheta = (fmaxf(fmaxf(rtb_Electrical_Speed, rtb_CosTheta),
                        rtb_Phase_Maximum) + fminf(fminf(rtb_Electrical_Speed,
    rtb_CosTheta), rtb_Phase_Maximum)) *
    PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* Switch: '<Root>/Stateflow_PWM_Gate_B' incorporates:
   *  Switch: '<Root>/Stateflow_PWM_Gate_A'
   *  Switch: '<Root>/Stateflow_PWM_Gate_C'
   */
  if (PMSM_FOC_DualPlant_Controller_B.PwmEnable) {
    /* Sum: '<S13>/Duty_B_Plus_Half' incorporates:
     *  Constant: '<S13>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S13>/Duty_B_Divide_Vdc'
     *  Sum: '<S13>/Phase_B_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = (rtb_CosTheta + rtb_SinTheta) /
      PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S13>/Duty_B_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyB > FOC_Native_DutyMax) {
      /* Sum: '<S13>/Duty_B_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyB'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyB < FOC_Native_DutyMin) {
      /* Sum: '<S13>/Duty_B_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyB'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S13>/Duty_B_Limit' */

    /* Sum: '<S13>/Duty_A_Plus_Half' incorporates:
     *  Constant: '<S13>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S13>/Duty_A_Divide_Vdc'
     *  Sum: '<S13>/Phase_A_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = (rtb_Electrical_Speed + rtb_SinTheta)
      / PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S13>/Duty_A_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyA > FOC_Native_DutyMax) {
      /* Sum: '<S13>/Duty_A_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyA'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyA < FOC_Native_DutyMin) {
      /* Sum: '<S13>/Duty_A_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyA'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S13>/Duty_A_Limit' */

    /* Sum: '<S13>/Duty_C_Plus_Half' incorporates:
     *  Constant: '<S13>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S13>/Duty_C_Divide_Vdc'
     *  Sum: '<S13>/Phase_C_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = (rtb_Phase_Maximum + rtb_SinTheta) /
      PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S13>/Duty_C_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyC > FOC_Native_DutyMax) {
      /* Sum: '<S13>/Duty_C_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyC'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyC < FOC_Native_DutyMin) {
      /* Sum: '<S13>/Duty_C_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyC'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S13>/Duty_C_Limit' */
  } else {
    /* Sum: '<S13>/Duty_B_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;

    /* Sum: '<S13>/Duty_A_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;

    /* Sum: '<S13>/Duty_C_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;
  }

  /* End of Switch: '<Root>/Stateflow_PWM_Gate_B' */

  /* Switch: '<S12>/Integrator_Reset_Select' incorporates:
   *  Switch: '<S6>/Integrator_Reset_Select'
   */
  if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
    /* Sum: '<S12>/Integrator_Add' incorporates:
     *  Constant: '<S12>/Integrator_Zero'
     *  UnitDelay: '<S12>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_h;

    /* Sum: '<S6>/Integrator_Add' incorporates:
     *  Constant: '<S6>/Integrator_Zero'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value;
  } else {
    /* Gain: '<S12>/KiTs' incorporates:
     *  Gain: '<S6>/KiTs'
     */
    rtb_SinTheta = FOC_Native_KiCurrent * FOC_Native_CurrentPeriod;

    /* Sum: '<S12>/Integrator_Add' incorporates:
     *  Gain: '<S12>/KiTs'
     *  UnitDelay: '<S12>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c += rtb_SinTheta *
      rtb_Phase_Minimum;

    /* Saturate: '<S12>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S12>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S12>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S12>/Integrator_Limit' */

    /* Sum: '<S6>/Integrator_Add' incorporates:
     *  Gain: '<S6>/KiTs'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_SinTheta *
      rtb_Current_Error;

    /* Saturate: '<S6>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S6>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S6>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S6>/Integrator_Limit' */
  }

  /* End of Switch: '<S12>/Integrator_Reset_Select' */

  /* Switch: '<S3>/SumB_Reset' incorporates:
   *  Constant: '<S3>/Sample_Target'
   *  Constant: '<S3>/Zero'
   *  Logic: '<S3>/Accumulate_Enable'
   *  RelationalOperator: '<S3>/Count_Below_Target'
   *  Switch: '<S3>/Count_Hold'
   *  Switch: '<S3>/Count_Reset'
   *  Switch: '<S3>/SumA_Hold'
   *  Switch: '<S3>/SumA_Reset'
   *  Switch: '<S3>/SumB_Hold'
   *  UnitDelay: '<S3>/Count_State'
   *  UnitDelay: '<S3>/SumA_State'
   *  UnitDelay: '<S3>/SumB_State'
   */
  if (PMSM_FOC_DualPlant_Controller_B.CalibrationReset) {
    PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
    PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
    PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
  } else if (PMSM_FOC_DualPlant_Controller_B.CalibrationEnable &&
             (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE <
              PMSM_FOC_DualPlant_Controller_P.Sample_Target_Value)) {
    /* UnitDelay: '<S3>/SumB_State' incorporates:
     *  Inport: '<Root>/PhaseCurrentB'
     *  Sum: '<S3>/SumB_Add'
     *  Switch: '<S3>/SumB_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB;

    /* UnitDelay: '<S3>/Count_State' incorporates:
     *  Constant: '<S3>/One'
     *  Sum: '<S3>/Count_Add'
     *  Switch: '<S3>/Count_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_P.One_Value;

    /* UnitDelay: '<S3>/SumA_State' incorporates:
     *  Inport: '<Root>/PhaseCurrentA'
     *  Sum: '<S3>/SumA_Add'
     *  Switch: '<S3>/SumA_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA;
  }

  /* End of Switch: '<S3>/SumB_Reset' */

  /* ZeroOrderHold: '<S14>/SpeedRef_1ms' incorporates:
   *  RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<Root>/Stateflow_Speed_Command_Gate' incorporates:
     *  Inport: '<Root>/SpeedReferenceRpm'
     */
    if (PMSM_FOC_DualPlant_Controller_B.ControlEnable) {
      rtb_Current_Error = PMSM_FOC_DualPlant_Controller_U.SpeedReferenceRpm;
    } else {
      rtb_Current_Error = 0.0F;
    }

    /* Gain: '<S14>/RpmToRad' incorporates:
     *  Inport: '<Root>/SpeedRpm'
     *  Product: '<Root>/Stateflow_Speed_Command_Gate'
     *  Sum: '<S14>/Speed_Error'
     */
    rtb_Phase_Minimum = (rtb_Current_Error -
                         PMSM_FOC_DualPlant_Controller_U.SpeedRpm) *
      PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* Sum: '<S14>/Integrator_Add' incorporates:
     *  Gain: '<S14>/KiTs'
     *  UnitDelay: '<S14>/Integrator_State'
     */
    rtb_Current_Error = FOC_Native_KiSpeed * FOC_Native_SpeedPeriod *
      rtb_Phase_Minimum +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d;

    /* Sum: '<S14>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S14>/Kp'
     *  UnitDelay: '<S14>/Integrator_State'
     */
    rtb_Phase_Minimum = FOC_Native_KpSpeed * rtb_Phase_Minimum +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d;

    /* Saturate: '<S14>/Iq_Reference_Limit' */
    if (rtb_Phase_Minimum > FOC_Native_IqLimit) {
      rtb_Phase_Minimum = FOC_Native_IqLimit;
    } else if (rtb_Phase_Minimum < -FOC_Native_IqLimit) {
      rtb_Phase_Minimum = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S14>/Iq_Reference_Limit' */
    PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 =
      rtb_Phase_Minimum;

    /* Switch: '<S14>/Integrator_Reset_Select' incorporates:
     *  Saturate: '<S14>/Integrator_Limit'
     */
    if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
      /* Update for UnitDelay: '<S14>/Integrator_State' incorporates:
       *  Constant: '<S14>/Integrator_Zero'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
        PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_p;
    } else if (rtb_Current_Error > FOC_Native_IqLimit) {
      /* Saturate: '<S14>/Integrator_Limit' incorporates:
       *  UnitDelay: '<S14>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
        FOC_Native_IqLimit;
    } else if (rtb_Current_Error < -FOC_Native_IqLimit) {
      /* Saturate: '<S14>/Integrator_Limit' incorporates:
       *  UnitDelay: '<S14>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
        -FOC_Native_IqLimit;
    } else {
      /* Update for UnitDelay: '<S14>/Integrator_State' incorporates:
       *  Saturate: '<S14>/Integrator_Limit'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
        rtb_Current_Error;
    }

    /* End of Switch: '<S14>/Integrator_Reset_Select' */
  }

  /* End of ZeroOrderHold: '<S14>/SpeedRef_1ms' */
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

  /* InitializeConditions for UnitDelay: '<S3>/Count_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Count_State_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S3>/SumA_State' */
  PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.SumA_State_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S3>/SumB_State' */
  PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.SumB_State_InitialCondition;

  /* InitializeConditions for Sum: '<S6>/Integrator_Add' incorporates:
   *  UnitDelay: '<S6>/Integrator_State'
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

  /* InitializeConditions for Sum: '<S12>/Integrator_Add' incorporates:
   *  UnitDelay: '<S12>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_i;

  /* InitializeConditions for UnitDelay: '<S14>/Integrator_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_d =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_a;
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
