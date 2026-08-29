/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
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
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<Root>/Motor_Supervisor_1ms' */
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
                                   *   '<S13>/Integrator_Limit'
                                   */
real32_T FOC_Native_CurrentPeriod = 0.0001F;/* Variable: FOC_Native_CurrentPeriod
                                             * Referenced by:
                                             *   '<S6>/KiTs'
                                             *   '<S13>/KiTs'
                                             */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S4>/Flux_PM'
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S16>/Integrator_Limit'
                                        *   '<S16>/Iq_Reference_Limit'
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S6>/KiTs'
                                        *   '<S13>/KiTs'
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S16>/KiTs'
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S6>/Kp'
                                        *   '<S13>/Kp'
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S16>/Kp'
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
                                          * Referenced by: '<S16>/KiTs'
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
  real32_T rtb_Cos_Electrical_Angle;
  real32_T rtb_Current_Error;
  real32_T rtb_Electrical_Speed;
  real32_T rtb_Phase_Maximum;
  real32_T rtb_Phase_Minimum;
  real32_T rtb_Phase_Vb;
  boolean_T rtb_PWM_Permit;
  boolean_T rtb_Start_Command;

  /* Logic: '<S8>/Raw_Fast_Fault' incorporates:
   *  Abs: '<S8>/Abs_Ia'
   *  Abs: '<S8>/Abs_Ib'
   *  Constant: '<S8>/Max_Current_A'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  RelationalOperator: '<S8>/OverCurrentA'
   *  RelationalOperator: '<S8>/OverCurrentB'
   */
  rtb_PWM_Permit = ((fabsf(PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA) >
                     PMSM_FOC_DualPlant_Controller_P.Max_Current_A_Value) ||
                    (fabsf(PMSM_FOC_DualPlant_Controller_U.PhaseCurrentB) >
                     PMSM_FOC_DualPlant_Controller_P.Max_Current_A_Value));

  /* Logic: '<S8>/Latched_Fault_Next' incorporates:
   *  Inport: '<Root>/FaultResetAck'
   *  Logic: '<S8>/Keep_Fault'
   *  Logic: '<S8>/No_Raw_Fault'
   *  Logic: '<S8>/Not_Reset_Permitted'
   *  Logic: '<S8>/Reset_Permitted'
   *  UnitDelay: '<S8>/Fault_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE = ((rtb_PWM_Permit ||
    PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE) &&
    ((!PMSM_FOC_DualPlant_Controller_U.FaultResetAck) || rtb_PWM_Permit));

  /* RateTransition: '<Root>/CalibrationDone_To_1ms' incorporates:
   *  RateTransition: '<Root>/FastFault_To_1ms'
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* RelationalOperator: '<Root>/Start_Command' incorporates:
     *  Constant: '<Root>/Start_Threshold_Rpm'
     *  Inport: '<Root>/SpeedReferenceRpm'
     */
    rtb_Start_Command = (PMSM_FOC_DualPlant_Controller_U.SpeedReferenceRpm >
                         PMSM_FOC_DualPlant_Controller_P.Start_Threshold_Rpm_Value);

    /* Logic: '<Root>/Supervisor_Fault_OR' incorporates:
     *  Abs: '<S15>/Abs_Speed'
     *  Constant: '<S15>/Max_Speed_Rpm'
     *  Constant: '<S15>/Max_Vdc'
     *  Constant: '<S15>/Min_Vdc'
     *  Inport: '<Root>/DcBusVoltage'
     *  Inport: '<Root>/SpeedRpm'
     *  Logic: '<S15>/Any_Slow_Fault'
     *  RelationalOperator: '<S15>/OverSpeed'
     *  RelationalOperator: '<S15>/OverVoltage'
     *  RelationalOperator: '<S15>/UnderVoltage'
     *  UnitDelay: '<S8>/Fault_Latch_State'
     */
    rtb_PWM_Permit = (PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE ||
                      ((fabsf(PMSM_FOC_DualPlant_Controller_U.SpeedRpm) >
                        PMSM_FOC_DualPlant_Controller_P.Max_Speed_Rpm_Value) ||
                       (PMSM_FOC_DualPlant_Controller_U.DcBusVoltage <
                        PMSM_FOC_DualPlant_Controller_P.Min_Vdc_Value) ||
                       (PMSM_FOC_DualPlant_Controller_U.DcBusVoltage >
                        PMSM_FOC_DualPlant_Controller_P.Max_Vdc_Value)));

    /* Chart: '<Root>/Motor_Supervisor_1ms' incorporates:
     *  Constant: '<S3>/Sample_Target'
     *  Inport: '<Root>/FaultResetAck'
     *  RelationalOperator: '<S3>/Count_Complete'
     *  UnitDelay: '<S3>/Count_State'
     */
    if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 < 31U) {
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
      if (PMSM_FOC_DualPlant_Controller_U.FaultResetAck && (!rtb_PWM_Permit) &&
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
    } else if (rtb_PWM_Permit) {
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
        if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >= 20) {
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
        } else if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >= 15) {
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

    /* End of Chart: '<Root>/Motor_Supervisor_1ms' */
  }

  /* End of RateTransition: '<Root>/CalibrationDone_To_1ms' */

  /* Trigonometry: '<S7>/Cos_Electrical_Angle' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  rtb_Cos_Electrical_Angle = cosf
    (PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

  /* Switch: '<S1>/Cos_Select' incorporates:
   *  Constant: '<S1>/Align_Cos_One'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    rtb_Phase_Minimum = PMSM_FOC_DualPlant_Controller_P.Align_Cos_One_Value;
  } else {
    rtb_Phase_Minimum = rtb_Cos_Electrical_Angle;
  }

  /* End of Switch: '<S1>/Cos_Select' */

  /* Switch: '<S3>/OffsetA_Valid' incorporates:
   *  Constant: '<S3>/Zero'
   *  RelationalOperator: '<S3>/Count_Positive'
   *  Switch: '<S3>/OffsetB_Valid'
   *  UnitDelay: '<S3>/Count_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >
      PMSM_FOC_DualPlant_Controller_P.Zero_Value) {
    /* Switch: '<S1>/Vq_Select' incorporates:
     *  Product: '<S3>/OffsetA_Calculate'
     *  UnitDelay: '<S3>/SumA_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand =
      PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;

    /* Switch: '<S1>/Vd_Select' incorporates:
     *  Product: '<S3>/OffsetB_Calculate'
     *  UnitDelay: '<S3>/SumB_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;
  } else {
    /* Switch: '<S1>/Vq_Select' */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;

    /* Switch: '<S1>/Vd_Select' */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
  }

  /* End of Switch: '<S3>/OffsetA_Valid' */

  /* Switch: '<S1>/Vq_Select' incorporates:
   *  Inport: '<Root>/PhaseCurrentA'
   *  Sum: '<S3>/Correct_Ia'
   */
  PMSM_FOC_DualPlant_Controller_Y.VqCommand =
    PMSM_FOC_DualPlant_Controller_U.PhaseCurrentA -
    PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Trigonometry: '<S7>/Sin_Electrical_Angle' incorporates:
   *  Inport: '<Root>/ElectricalAngleRad'
   */
  rtb_Phase_Maximum = sinf(PMSM_FOC_DualPlant_Controller_U.ElectricalAngleRad);

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
     PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain +
     PMSM_FOC_DualPlant_Controller_Y.VqCommand) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S12>/Id_Sum' incorporates:
   *  Product: '<S12>/Id_CosAlpha'
   *  Product: '<S12>/Id_SinBeta'
   */
  PMSM_FOC_DualPlant_Controller_Y.IdMeasured = rtb_Cos_Electrical_Angle *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand + rtb_Phase_Maximum *
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

  /* Sum: '<S12>/Iq_Sum' incorporates:
   *  Gain: '<S12>/Negative'
   *  Product: '<S12>/Iq_CosBeta'
   *  Product: '<S12>/Iq_SinAlpha'
   */
  PMSM_FOC_DualPlant_Controller_Y.IqMeasured = rtb_Phase_Maximum *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_Cos_Electrical_Angle *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand;

  /* Switch: '<S1>/Vd_Select' incorporates:
   *  Constant: '<S1>/Align_Sin_Zero'
   *  Switch: '<S1>/Sin_Select'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    /* Switch: '<S1>/Vd_Select' incorporates:
     *  Constant: '<S1>/Align_Vd_2V'
     */
    PMSM_FOC_DualPlant_Controller_Y.VdCommand =
      PMSM_FOC_DualPlant_Controller_P.Align_Vd_2V_Value;
    rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controller_P.Align_Sin_Zero_Value;
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

  /* Sum: '<S13>/Current_Error' incorporates:
   *  Outport: '<Root>/IqReference'
   */
  rtb_Cos_Electrical_Angle = PMSM_FOC_DualPlant_Controller_Y.IqReference -
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
     *  Gain: '<S13>/Kp'
     *  Gain: '<S4>/Ld_x_Id'
     *  Product: '<S4>/Q_Feedforward'
     *  Sum: '<S13>/PI_Sum'
     *  Sum: '<S4>/Flux_Linkage'
     *  Sum: '<S5>/Vq_Raw'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controller_Y.VqCommand = (FOC_Native_Ld *
      PMSM_FOC_DualPlant_Controller_Y.IdMeasured + FOC_Native_FluxPM) *
      rtb_Electrical_Speed + (FOC_Native_KpCurrent * rtb_Cos_Electrical_Angle +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o);

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

  /* Sum: '<S10>/Valpha_Sum' incorporates:
   *  Product: '<S10>/Valpha_CosVd'
   *  Product: '<S10>/Valpha_SinVq'
   */
  rtb_Electrical_Speed = rtb_Phase_Minimum *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand - rtb_Phase_Maximum *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S10>/Vbeta_Sum' incorporates:
   *  Product: '<S10>/Vbeta_CosVq'
   *  Product: '<S10>/Vbeta_SinVd'
   */
  rtb_Phase_Maximum = rtb_Phase_Maximum *
    PMSM_FOC_DualPlant_Controller_Y.VdCommand + rtb_Phase_Minimum *
    PMSM_FOC_DualPlant_Controller_Y.VqCommand;

  /* Sum: '<S9>/Phase_Vb' incorporates:
   *  Gain: '<S9>/Vb_Alpha'
   *  Gain: '<S9>/Vb_Beta'
   */
  rtb_Phase_Vb = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain *
    rtb_Electrical_Speed + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Phase_Maximum;

  /* Sum: '<S9>/Phase_Vc' incorporates:
   *  Gain: '<S9>/Vc_Alpha'
   *  Gain: '<S9>/Vc_Beta'
   */
  rtb_Phase_Maximum = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain *
    rtb_Electrical_Speed + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Phase_Maximum;

  /* Gain: '<S14>/Common_Mode' incorporates:
   *  MinMax: '<S14>/Phase_Maximum'
   *  MinMax: '<S14>/Phase_Minimum'
   *  Sum: '<S14>/Max_Plus_Min'
   */
  rtb_Phase_Minimum = (fmaxf(fmaxf(rtb_Electrical_Speed, rtb_Phase_Vb),
    rtb_Phase_Maximum) + fminf(fminf(rtb_Electrical_Speed, rtb_Phase_Vb),
    rtb_Phase_Maximum)) * PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* Switch: '<Root>/Stateflow_PWM_Gate_B' incorporates:
   *  Logic: '<S8>/No_Fast_Fault'
   *  Logic: '<S8>/PWM_Permit'
   *  Switch: '<Root>/Stateflow_PWM_Gate_A'
   *  Switch: '<Root>/Stateflow_PWM_Gate_C'
   *  UnitDelay: '<S8>/Fault_Latch_State'
   */
  if (PMSM_FOC_DualPlant_Controller_B.PwmEnable &&
      (!PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE)) {
    /* Sum: '<S14>/Duty_B_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S14>/Duty_B_Divide_Vdc'
     *  Sum: '<S14>/Phase_B_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB = (rtb_Phase_Vb + rtb_Phase_Minimum) /
      PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_B_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyB > FOC_Native_DutyMax) {
      /* Sum: '<S14>/Duty_B_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyB'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyB < FOC_Native_DutyMin) {
      /* Sum: '<S14>/Duty_B_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyB'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyB = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S14>/Duty_B_Limit' */

    /* Sum: '<S14>/Duty_A_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S14>/Duty_A_Divide_Vdc'
     *  Sum: '<S14>/Phase_A_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA = (rtb_Electrical_Speed +
      rtb_Phase_Minimum) / PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_A_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyA > FOC_Native_DutyMax) {
      /* Sum: '<S14>/Duty_A_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyA'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyA < FOC_Native_DutyMin) {
      /* Sum: '<S14>/Duty_A_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyA'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyA = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S14>/Duty_A_Limit' */

    /* Sum: '<S14>/Duty_C_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/DcBusVoltage'
     *  Product: '<S14>/Duty_C_Divide_Vdc'
     *  Sum: '<S14>/Phase_C_Plus_Common'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC = (rtb_Phase_Maximum +
      rtb_Phase_Minimum) / PMSM_FOC_DualPlant_Controller_U.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_C_Limit' */
    if (PMSM_FOC_DualPlant_Controller_Y.DutyC > FOC_Native_DutyMax) {
      /* Sum: '<S14>/Duty_C_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyC'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMax;
    } else if (PMSM_FOC_DualPlant_Controller_Y.DutyC < FOC_Native_DutyMin) {
      /* Sum: '<S14>/Duty_C_Plus_Half' incorporates:
       *  Outport: '<Root>/DutyC'
       */
      PMSM_FOC_DualPlant_Controller_Y.DutyC = FOC_Native_DutyMin;
    }

    /* End of Saturate: '<S14>/Duty_C_Limit' */
  } else {
    /* Sum: '<S14>/Duty_B_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyB'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyB =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;

    /* Sum: '<S14>/Duty_A_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyA'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyA =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;

    /* Sum: '<S14>/Duty_C_Plus_Half' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/DutyC'
     */
    PMSM_FOC_DualPlant_Controller_Y.DutyC =
      PMSM_FOC_DualPlant_Controller_P.Safe_Duty_50pct_Value;
  }

  /* End of Switch: '<Root>/Stateflow_PWM_Gate_B' */

  /* Switch: '<S13>/Integrator_Reset_Select' incorporates:
   *  Switch: '<S6>/Integrator_Reset_Select'
   */
  if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
    /* Sum: '<S13>/Integrator_Add' incorporates:
     *  Constant: '<S13>/Integrator_Zero'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_c;

    /* Sum: '<S6>/Integrator_Add' incorporates:
     *  Constant: '<S6>/Integrator_Zero'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value;
  } else {
    /* Gain: '<S13>/KiTs' incorporates:
     *  Gain: '<S6>/KiTs'
     */
    rtb_Phase_Minimum = FOC_Native_KiCurrent * FOC_Native_CurrentPeriod;

    /* Sum: '<S13>/Integrator_Add' incorporates:
     *  Gain: '<S13>/KiTs'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o +=
      rtb_Phase_Minimum * rtb_Cos_Electrical_Angle;

    /* Saturate: '<S13>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S13>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S13>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S13>/Integrator_Limit' */

    /* Sum: '<S6>/Integrator_Add' incorporates:
     *  Gain: '<S6>/KiTs'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_Phase_Minimum
      * rtb_Current_Error;

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

  /* End of Switch: '<S13>/Integrator_Reset_Select' */

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

  /* ZeroOrderHold: '<S16>/SpeedRef_1ms' incorporates:
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

    /* Gain: '<S16>/RpmToRad' incorporates:
     *  Inport: '<Root>/SpeedRpm'
     *  Product: '<Root>/Stateflow_Speed_Command_Gate'
     *  Sum: '<S16>/Speed_Error'
     */
    rtb_Current_Error = (rtb_Current_Error -
                         PMSM_FOC_DualPlant_Controller_U.SpeedRpm) *
      PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* UnitDelay: '<S16>/Integrator_State' */
    rtb_Cos_Electrical_Angle =
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p;

    /* Switch: '<S16>/Integrator_Reset_Select' */
    if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
      /* Sum: '<S16>/Integrator_Add' incorporates:
       *  Constant: '<S16>/Integrator_Zero'
       *  UnitDelay: '<S16>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p =
        PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_g;
    } else {
      /* Sum: '<S16>/Integrator_Add' incorporates:
       *  Gain: '<S16>/KiTs'
       *  UnitDelay: '<S16>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p +=
        FOC_Native_KiSpeed * FOC_Native_SpeedPeriod * rtb_Current_Error;

      /* Saturate: '<S16>/Integrator_Limit' */
      if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p >
          FOC_Native_IqLimit) {
        /* Sum: '<S16>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p =
          FOC_Native_IqLimit;
      } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p <
                 -FOC_Native_IqLimit) {
        /* Sum: '<S16>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p =
          -FOC_Native_IqLimit;
      }

      /* End of Saturate: '<S16>/Integrator_Limit' */
    }

    /* End of Switch: '<S16>/Integrator_Reset_Select' */

    /* Sum: '<S16>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S16>/Kp'
     */
    rtb_Current_Error = FOC_Native_KpSpeed * rtb_Current_Error +
      rtb_Cos_Electrical_Angle;

    /* Saturate: '<S16>/Iq_Reference_Limit' */
    if (rtb_Current_Error > FOC_Native_IqLimit) {
      rtb_Current_Error = FOC_Native_IqLimit;
    } else if (rtb_Current_Error < -FOC_Native_IqLimit) {
      rtb_Current_Error = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S16>/Iq_Reference_Limit' */
    PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 =
      rtb_Current_Error;
  }

  /* End of ZeroOrderHold: '<S16>/SpeedRef_1ms' */
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

  /* InitializeConditions for UnitDelay: '<S8>/Fault_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Fault_Latch_State_InitialCondit;

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

  /* InitializeConditions for Sum: '<S13>/Integrator_Add' incorporates:
   *  UnitDelay: '<S13>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_o =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_n;

  /* InitializeConditions for Sum: '<S16>/Integrator_Add' incorporates:
   *  UnitDelay: '<S16>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_p =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_d;
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
