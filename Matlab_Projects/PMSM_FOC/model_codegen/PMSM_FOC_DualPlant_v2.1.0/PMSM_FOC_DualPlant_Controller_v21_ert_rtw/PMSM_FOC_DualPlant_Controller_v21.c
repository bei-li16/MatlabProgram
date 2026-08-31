/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
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
                                   * D/q PI integrator absolute limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                   */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        * Maximum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        * Minimum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S4>/Flux_PM'
                                        * Permanent-magnet flux linkage Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S16>/Integrator_Limit'
                                        *   '<S16>/Iq_Reference_Limit'
                                        * Speed-loop q-axis current command limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S6>/KiTs'
                                        *   '<S13>/KiTs'
                                        * Current PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S16>/KiTs'
                                        * Speed PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S6>/Kp'
                                        *   '<S13>/Kp'
                                        * Current PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S16>/Kp'
                                        * Speed PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S4>/Ld_x_Id'
                                        * D-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S4>/D_Decoupling'
                                        * Q-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S4>/Electrical_Speed'
                                        * Integer-valued motor pole-pair count stored as single for plant equation compatibility Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S5>/Vd_Limit'
                                          *   '<S5>/Vq_Limit'
                                          * Independent d/q voltage command limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                          */
real32_T PMSM_Alignment_Vd_V = 2.0F;   /* Variable: PMSM_Alignment_Vd_V
                                        * Referenced by: '<S1>/Align_Vd'
                                        * Alignment d-axis voltage Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T PMSM_Alignment_Vq_V = 0.0F;   /* Variable: PMSM_Alignment_Vq_V
                                        * Referenced by: '<S1>/Align_Vq'
                                        * Alignment q-axis voltage Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T PMSM_Protection_MaxCurrent_A = 12.0F;/* Variable: PMSM_Protection_MaxCurrent_A
                                               * Referenced by: '<S8>/Max_Current_A'
                                               * Fast overcurrent trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                               */
real32_T PMSM_Protection_MaxDcBus_V = 60.0F;/* Variable: PMSM_Protection_MaxDcBus_V
                                             * Referenced by: '<S15>/Max_Vdc'
                                             * DC-bus overvoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                             */
real32_T PMSM_Protection_MaxSpeed_Rpm = 3000.0F;/* Variable: PMSM_Protection_MaxSpeed_Rpm
                                                 * Referenced by: '<S15>/Max_Speed_Rpm'
                                                 * Slow overspeed trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                                 */
real32_T PMSM_Protection_MinDcBus_V = 10.0F;/* Variable: PMSM_Protection_MinDcBus_V
                                             * Referenced by: '<S15>/Min_Vdc'
                                             * DC-bus undervoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                             */
real32_T PMSM_StartThreshold_Rpm = 1.0F;/* Variable: PMSM_StartThreshold_Rpm
                                         * Referenced by: '<Root>/Start_Threshold_Rpm'
                                         * Legacy implicit-start threshold pending ARC-003 Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                         */
uint16_T PMSM_Alignment_DurationTicks = 20U;/* Variable: PMSM_Alignment_DurationTicks
                                             * Referenced by: '<Root>/Motor_Supervisor_1ms'
                                             * Alignment duration in 1 ms supervisor ticks Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                             */
uint16_T PMSM_Calibration_SampleCount = 100U;/* Variable: PMSM_Calibration_SampleCount
                                              * Referenced by: '<S3>/Sample_Target'
                                              * Current-offset averaging sample count Owner=Measurement; Version=2.1.0; Class=TunableCalibration.
                                              */
uint16_T PMSM_Calibration_TimeoutTicks = 15U;
                                      /* Variable: PMSM_Calibration_TimeoutTicks
                                       * Referenced by: '<Root>/Motor_Supervisor_1ms'
                                       * Calibration timeout in 1 ms supervisor ticks Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
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
  real32_T rtb_Common_Mode;
  real32_T rtb_Current_Error;
  real32_T rtb_DutyA;
  real32_T rtb_DutyB;
  real32_T rtb_DutyC;
  real32_T rtb_Electrical_Speed;
  real32_T rtb_Id_Sum;
  real32_T rtb_Iq_Sum;
  real32_T rtb_Vd_Select;
  real32_T rtb_Vq_Select;
  uint8_T rtb_StateCode;
  boolean_T rtb_CalibrationDone_To_1ms;
  boolean_T rtb_PWM_Permit;
  boolean_T rtb_Start_Command;
  boolean_T rtb_Supervisor_Fault_OR;

  /* RateTransition: '<Root>/CalibrationDone_To_1ms' incorporates:
   *  Constant: '<S3>/Sample_Target'
   *  RelationalOperator: '<S3>/Count_Complete'
   *  UnitDelay: '<S3>/Count_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    rtb_CalibrationDone_To_1ms =
      (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >=
       PMSM_Calibration_SampleCount);

    /* RelationalOperator: '<Root>/Start_Command' incorporates:
     *  Constant: '<Root>/Start_Threshold_Rpm'
     *  Constant: '<S3>/Sample_Target'
     *  Inport: '<Root>/ControlCommand'
     *  RelationalOperator: '<S3>/Count_Complete'
     *  UnitDelay: '<S3>/Count_State'
     *  ZeroOrderHold: '<Root>/Start_Command_Sample_1ms'
     */
    rtb_Start_Command =
      (PMSM_FOC_DualPlant_Controller_U.ControlCommand.SpeedReferenceRpm >
       PMSM_StartThreshold_Rpm);
  }

  /* End of RateTransition: '<Root>/CalibrationDone_To_1ms' */

  /* Logic: '<S8>/Raw_Fast_Fault' incorporates:
   *  Abs: '<S8>/Abs_Ia'
   *  Abs: '<S8>/Abs_Ib'
   *  Constant: '<S8>/Max_Current_A'
   *  Inport: '<Root>/Measurement'
   *  RelationalOperator: '<S8>/OverCurrentA'
   *  RelationalOperator: '<S8>/OverCurrentB'
   */
  rtb_PWM_Permit = ((fabsf
                     (PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA)
                     > PMSM_Protection_MaxCurrent_A) || (fabsf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB) >
    PMSM_Protection_MaxCurrent_A));

  /* Logic: '<S8>/Latched_Fault_Next' incorporates:
   *  Inport: '<Root>/ControlCommand'
   *  Logic: '<S8>/Keep_Fault'
   *  Logic: '<S8>/No_Raw_Fault'
   *  Logic: '<S8>/Not_Reset_Permitted'
   *  Logic: '<S8>/Reset_Permitted'
   *  UnitDelay: '<S8>/Fault_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE = ((rtb_PWM_Permit ||
    PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE) &&
    ((!PMSM_FOC_DualPlant_Controller_U.ControlCommand.FaultResetRequest) ||
     rtb_PWM_Permit));

  /* RateTransition: '<Root>/FastFault_To_1ms' */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Logic: '<Root>/Supervisor_Fault_OR' incorporates:
     *  Abs: '<S15>/Abs_Speed'
     *  Constant: '<S15>/Max_Speed_Rpm'
     *  Constant: '<S15>/Max_Vdc'
     *  Constant: '<S15>/Min_Vdc'
     *  Inport: '<Root>/Measurement'
     *  Logic: '<S15>/Any_Slow_Fault'
     *  RelationalOperator: '<S15>/OverSpeed'
     *  RelationalOperator: '<S15>/OverVoltage'
     *  RelationalOperator: '<S15>/UnderVoltage'
     *  UnitDelay: '<S8>/Fault_Latch_State'
     *  ZeroOrderHold: '<S15>/Speed_Sample_1ms'
     *  ZeroOrderHold: '<S15>/Vdc_Sample_1ms'
     */
    rtb_Supervisor_Fault_OR =
      (PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE || ((fabsf
         (PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm) >
         PMSM_Protection_MaxSpeed_Rpm) ||
        (PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage <
         PMSM_Protection_MinDcBus_V) ||
        (PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage >
         PMSM_Protection_MaxDcBus_V)));

    /* Chart: '<Root>/Motor_Supervisor_1ms' incorporates:
     *  Inport: '<Root>/ControlCommand'
     */
    if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 < 65535U) {
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
      rtb_StateCode = 1U;
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
      PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro ==
               PMSM_FOC_DualPlant_Con_IN_FAULT) {
      rtb_StateCode = 6U;
      if (PMSM_FOC_DualPlant_Controller_U.ControlCommand.FaultResetRequest &&
          (!rtb_Supervisor_Fault_OR) && (!rtb_Start_Command)) {
        PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
          PMSM_FOC_DualPlan_IN_SUPERVISED;
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Cont_IN_INIT;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
        rtb_StateCode = 1U;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }

      /* case IN_SUPERVISED: */
    } else if (rtb_Supervisor_Fault_OR) {
      PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
        PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD;
      PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
        PMSM_FOC_DualPlant_Con_IN_FAULT;
      PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
      PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
      rtb_StateCode = 6U;
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
      PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
    } else {
      switch (PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED) {
       case PMSM_FOC_DualPlant_Con_IN_ALIGN:
        rtb_StateCode = 4U;
        if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >=
            PMSM_Alignment_DurationTicks) {
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Contr_IN_RUN;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = true;
          PMSM_FOC_DualPlant_Controller_B.PwmEnable = true;
          rtb_StateCode = 5U;
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
          rtb_StateCode = 1U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        }
        break;

       case PMSM_FOC_DualPlant_Con_IN_CALIB:
        rtb_StateCode = 3U;
        if (rtb_CalibrationDone_To_1ms) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Con_IN_ALIGN;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.PwmEnable = true;
          rtb_StateCode = 4U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = true;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        } else if (PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 >=
                   PMSM_Calibration_TimeoutTicks) {
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD;
          PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
            PMSM_FOC_DualPlant_Con_IN_FAULT;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
          rtb_StateCode = 6U;
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
        rtb_StateCode = 2U;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        break;

       case PMSM_FOC_DualPlant_Con_IN_READY:
        rtb_StateCode = 2U;
        if (rtb_Start_Command) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Con_IN_CALIB;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
          rtb_StateCode = 3U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = true;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        }
        break;

       default:
        /* case IN_RUN: */
        rtb_StateCode = 5U;
        if (!rtb_Start_Command) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Cont_IN_INIT;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.PwmEnable = false;
          rtb_StateCode = 1U;
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

  /* End of RateTransition: '<Root>/FastFault_To_1ms' */

  /* Trigonometry: '<S7>/Cos_Electrical_Angle' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_DutyC = cosf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.ElectricalAngleRad);

  /* Switch: '<S1>/Cos_Select' incorporates:
   *  Constant: '<S1>/Align_Cos'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    rtb_DutyA = PMSM_FOC_DualPlant_Controller_P.PMSM_Alignment_Cos;
  } else {
    rtb_DutyA = rtb_DutyC;
  }

  /* End of Switch: '<S1>/Cos_Select' */

  /* Switch: '<S3>/OffsetA_Valid' incorporates:
   *  Constant: '<S3>/Zero'
   *  Product: '<S3>/OffsetA_Calculate'
   *  Product: '<S3>/OffsetB_Calculate'
   *  RelationalOperator: '<S3>/Count_Positive'
   *  Switch: '<S3>/OffsetB_Valid'
   *  UnitDelay: '<S3>/Count_State'
   *  UnitDelay: '<S3>/SumA_State'
   *  UnitDelay: '<S3>/SumB_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >
      PMSM_FOC_DualPlant_Controller_P.Zero_Value) {
    rtb_Vq_Select = PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;
    rtb_Vd_Select = PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE /
      PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE;
  } else {
    rtb_Vq_Select = PMSM_FOC_DualPlant_Controller_P.Zero_Value;
    rtb_Vd_Select = PMSM_FOC_DualPlant_Controller_P.Zero_Value;
  }

  /* End of Switch: '<S3>/OffsetA_Valid' */

  /* Sum: '<S3>/Correct_Ia' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Vq_Select = PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA -
    rtb_Vq_Select;

  /* Trigonometry: '<S7>/Sin_Electrical_Angle' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_DutyB = sinf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.ElectricalAngleRad);

  /* Gain: '<S2>/InvSqrt3' incorporates:
   *  Gain: '<S2>/Ib_x2'
   *  Inport: '<Root>/Measurement'
   *  Sum: '<S2>/Ia_Plus_2Ib'
   *  Sum: '<S3>/Correct_Ib'
   */
  rtb_Vd_Select = ((PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB -
                    rtb_Vd_Select) * PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain
                   + rtb_Vq_Select) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S12>/Id_Sum' incorporates:
   *  Product: '<S12>/Id_CosAlpha'
   *  Product: '<S12>/Id_SinBeta'
   */
  rtb_Id_Sum = rtb_DutyC * rtb_Vq_Select + rtb_DutyB * rtb_Vd_Select;

  /* Sum: '<S6>/Current_Error' incorporates:
   *  Constant: '<Root>/Id_Reference_Zero'
   *
   * Block description for '<Root>/Id_Reference_Zero':
   *  Field-oriented control d-axis current reference: Id*=0 A.
   */
  rtb_Current_Error = PMSM_FOC_DualPlant_Controller_P.Id_Reference_Zero_Value -
    rtb_Id_Sum;

  /* Gain: '<S4>/Electrical_Speed' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Electrical_Speed = PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S *
    FOC_Native_PolePairs *
    PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm;

  /* Sum: '<S12>/Iq_Sum' incorporates:
   *  Gain: '<S12>/Negative'
   *  Product: '<S12>/Iq_CosBeta'
   *  Product: '<S12>/Iq_SinAlpha'
   */
  rtb_Iq_Sum = rtb_DutyB * rtb_Vq_Select *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_DutyC * rtb_Vd_Select;

  /* Switch: '<S1>/Vd_Select' incorporates:
   *  Constant: '<S1>/Align_Sin'
   *  Constant: '<S1>/Align_Vd'
   *  Switch: '<S1>/Sin_Select'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    rtb_Vd_Select = PMSM_Alignment_Vd_V;
    rtb_DutyB = PMSM_FOC_DualPlant_Controller_P.PMSM_Alignment_Sin;
  } else {
    /* Sum: '<S5>/Vd_Raw' incorporates:
     *  Gain: '<S4>/D_Decoupling'
     *  Gain: '<S6>/Kp'
     *  Product: '<S4>/Omega_x_Iq'
     *  Sum: '<S6>/PI_Sum'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    rtb_Vd_Select = (FOC_Native_KpCurrent * rtb_Current_Error +
                     PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE) +
      rtb_Electrical_Speed * rtb_Iq_Sum * -FOC_Native_Lq;

    /* Saturate: '<S5>/Vd_Limit' */
    if (rtb_Vd_Select > FOC_Native_VoltageLimit) {
      rtb_Vd_Select = FOC_Native_VoltageLimit;
    } else if (rtb_Vd_Select < -FOC_Native_VoltageLimit) {
      rtb_Vd_Select = -FOC_Native_VoltageLimit;
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
    /* RateTransition: '<Root>/IqRef_Rate_Transition'
     *
     * Block description for '<Root>/IqRef_Rate_Transition':
     *  Explicit deterministic transfer from 1 ms speed task to 100 us current
     *  task.
     */
    PMSM_FOC_DualPlant_Controller_B.IqReference =
      PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0;
  }

  /* End of RateTransition: '<Root>/IqRef_Rate_Transition' */

  /* Sum: '<S13>/Current_Error' */
  rtb_DutyC = PMSM_FOC_DualPlant_Controller_B.IqReference - rtb_Iq_Sum;

  /* Switch: '<S1>/Vq_Select' incorporates:
   *  Constant: '<S1>/Align_Vq'
   */
  if (PMSM_FOC_DualPlant_Controller_B.AlignmentEnable) {
    rtb_Vq_Select = PMSM_Alignment_Vq_V;
  } else {
    /* Sum: '<S5>/Vq_Raw' incorporates:
     *  Constant: '<S4>/Flux_PM'
     *  Gain: '<S13>/Kp'
     *  Gain: '<S4>/Ld_x_Id'
     *  Product: '<S4>/Q_Feedforward'
     *  Sum: '<S13>/PI_Sum'
     *  Sum: '<S4>/Flux_Linkage'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    rtb_Vq_Select = (FOC_Native_Ld * rtb_Id_Sum + FOC_Native_FluxPM) *
      rtb_Electrical_Speed + (FOC_Native_KpCurrent * rtb_DutyC +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c);

    /* Saturate: '<S5>/Vq_Limit' */
    if (rtb_Vq_Select > FOC_Native_VoltageLimit) {
      rtb_Vq_Select = FOC_Native_VoltageLimit;
    } else if (rtb_Vq_Select < -FOC_Native_VoltageLimit) {
      rtb_Vq_Select = -FOC_Native_VoltageLimit;
    }

    /* End of Saturate: '<S5>/Vq_Limit' */
  }

  /* End of Switch: '<S1>/Vq_Select' */

  /* Sum: '<S10>/Valpha_Sum' incorporates:
   *  Product: '<S10>/Valpha_CosVd'
   *  Product: '<S10>/Valpha_SinVq'
   */
  rtb_Electrical_Speed = rtb_DutyA * rtb_Vd_Select - rtb_DutyB * rtb_Vq_Select;

  /* Sum: '<S10>/Vbeta_Sum' incorporates:
   *  Product: '<S10>/Vbeta_CosVq'
   *  Product: '<S10>/Vbeta_SinVd'
   */
  rtb_DutyB = rtb_DutyB * rtb_Vd_Select + rtb_DutyA * rtb_Vq_Select;

  /* Sum: '<S9>/Phase_Vb' incorporates:
   *  Gain: '<S9>/Vb_Alpha'
   *  Gain: '<S9>/Vb_Beta'
   */
  rtb_DutyA = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain *
    rtb_Electrical_Speed + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_DutyB;

  /* Sum: '<S9>/Phase_Vc' incorporates:
   *  Gain: '<S9>/Vc_Alpha'
   *  Gain: '<S9>/Vc_Beta'
   */
  rtb_DutyB = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain *
    rtb_Electrical_Speed + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_DutyB;

  /* Gain: '<S14>/Common_Mode' incorporates:
   *  MinMax: '<S14>/Phase_Maximum'
   *  MinMax: '<S14>/Phase_Minimum'
   *  Sum: '<S14>/Max_Plus_Min'
   */
  rtb_Common_Mode = (fmaxf(fmaxf(rtb_Electrical_Speed, rtb_DutyA), rtb_DutyB) +
                     fminf(fminf(rtb_Electrical_Speed, rtb_DutyA), rtb_DutyB)) *
    PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* Logic: '<S8>/PWM_Permit' incorporates:
   *  Logic: '<S8>/No_Fast_Fault'
   *  UnitDelay: '<S8>/Fault_Latch_State'
   */
  rtb_PWM_Permit = (PMSM_FOC_DualPlant_Controller_B.PwmEnable &&
                    (!PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE));

  /* Switch: '<Root>/Stateflow_PWM_Gate_B' */
  if (rtb_PWM_Permit) {
    /* Sum: '<S14>/Duty_B_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/Measurement'
     *  Product: '<S14>/Duty_B_Divide_Vdc'
     *  Sum: '<S14>/Phase_B_Plus_Common'
     */
    rtb_DutyA = (rtb_DutyA + rtb_Common_Mode) /
      PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_B_Limit' */
    if (rtb_DutyA > FOC_Native_DutyMax) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = FOC_Native_DutyMax;
    } else if (rtb_DutyA < FOC_Native_DutyMin) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = FOC_Native_DutyMin;
    } else {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = rtb_DutyA;
    }

    /* End of Saturate: '<S14>/Duty_B_Limit' */
  } else {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB =
      PMSM_FOC_DualPlant_Controller_P.PMSM_SafeDuty;
  }

  /* End of Switch: '<Root>/Stateflow_PWM_Gate_B' */

  /* RateTransition: '<Root>/StatusState_To_100us' incorporates:
   *  RateTransition: '<Root>/StatusFault_To_100us'
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.MotorStateCode =
      PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0;

    /* RateTransition: '<Root>/StatusFault_To_100us' */
    PMSM_FOC_DualPlant_Controller_B.StatusFault_To_100us =
      PMSM_FOC_DualPlant_Controlle_DW.StatusFault_To_100us_Buffer0;
  }

  /* End of RateTransition: '<Root>/StatusState_To_100us' */

  /* Switch: '<Root>/Stateflow_PWM_Gate_A' incorporates:
   *  Switch: '<Root>/Stateflow_PWM_Gate_C'
   */
  if (rtb_PWM_Permit) {
    /* Sum: '<S14>/Duty_A_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/Measurement'
     *  Product: '<S14>/Duty_A_Divide_Vdc'
     *  Sum: '<S14>/Phase_A_Plus_Common'
     */
    rtb_DutyA = (rtb_Electrical_Speed + rtb_Common_Mode) /
      PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_A_Limit' */
    if (rtb_DutyA > FOC_Native_DutyMax) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = FOC_Native_DutyMax;
    } else if (rtb_DutyA < FOC_Native_DutyMin) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = FOC_Native_DutyMin;
    } else {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = rtb_DutyA;
    }

    /* End of Saturate: '<S14>/Duty_A_Limit' */

    /* Sum: '<S14>/Duty_C_Plus_Half' incorporates:
     *  Constant: '<S14>/Duty_Half'
     *  Inport: '<Root>/Measurement'
     *  Product: '<S14>/Duty_C_Divide_Vdc'
     *  Sum: '<S14>/Phase_C_Plus_Common'
     */
    rtb_DutyA = (rtb_DutyB + rtb_Common_Mode) /
      PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
      PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

    /* Saturate: '<S14>/Duty_C_Limit' */
    if (rtb_DutyA > FOC_Native_DutyMax) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = FOC_Native_DutyMax;
    } else if (rtb_DutyA < FOC_Native_DutyMin) {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = FOC_Native_DutyMin;
    } else {
      /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
       *  Outport: '<Root>/ControlStatus'
       */
      PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = rtb_DutyA;
    }

    /* End of Saturate: '<S14>/Duty_C_Limit' */
  } else {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Constant: '<Root>/Safe_Duty_50pct'
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA =
      PMSM_FOC_DualPlant_Controller_P.PMSM_SafeDuty;
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC =
      PMSM_FOC_DualPlant_Controller_P.PMSM_SafeDuty;
  }

  /* End of Switch: '<Root>/Stateflow_PWM_Gate_A' */

  /* DataTypeConversion: '<Root>/FaultBits_U32' */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultBits =
    PMSM_FOC_DualPlant_Controller_B.StatusFault_To_100us;

  /* DataTypeConversion: '<Root>/FaultCode_U16' */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
    PMSM_FOC_DualPlant_Controller_B.StatusFault_To_100us;

  /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
   *  Constant: '<Root>/VoltageLimit_Inactive'
   *  Inport: '<Root>/Measurement'
   *  Outport: '<Root>/ControlStatus'
   *  UnitDelay: '<S8>/Fault_Latch_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IqReference =
    PMSM_FOC_DualPlant_Controller_B.IqReference;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IdMeasured = rtb_Id_Sum;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IqMeasured = rtb_Iq_Sum;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VdCommand = rtb_Vd_Select;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VqCommand = rtb_Vq_Select;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.PwmEnable = rtb_PWM_Permit;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.CurrentLimitActive =
    PMSM_FOC_DualPlant_Controlle_DW.Fault_Latch_State_DSTATE;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VoltageLimitActive =
    PMSM_FOC_DualPlant_Controller_P.VoltageLimit_Inactive_Value;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.MeasurementValid =
    PMSM_FOC_DualPlant_Controller_U.Measurement.Valid;

  /* Switch: '<S13>/Integrator_Reset_Select' incorporates:
   *  Switch: '<S6>/Integrator_Reset_Select'
   */
  if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
    /* Sum: '<S13>/Integrator_Add' incorporates:
     *  Constant: '<S13>/Integrator_Zero'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_j;

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
    rtb_Vq_Select = FOC_Native_KiCurrent *
      PMSM_FOC_DualPlant_Controller_P.FOC_Native_CurrentPeriod;

    /* Sum: '<S13>/Integrator_Add' incorporates:
     *  Gain: '<S13>/KiTs'
     *  UnitDelay: '<S13>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c += rtb_Vq_Select *
      rtb_DutyC;

    /* Saturate: '<S13>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S13>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S13>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S13>/Integrator_Limit' */

    /* Sum: '<S6>/Integrator_Add' incorporates:
     *  Gain: '<S6>/KiTs'
     *  UnitDelay: '<S6>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_Vq_Select *
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
              PMSM_Calibration_SampleCount)) {
    /* UnitDelay: '<S3>/SumB_State' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Sum: '<S3>/SumB_Add'
     *  Switch: '<S3>/SumB_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB;

    /* UnitDelay: '<S3>/Count_State' incorporates:
     *  Constant: '<S3>/One'
     *  Sum: '<S3>/Count_Add'
     *  Switch: '<S3>/Count_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_P.One_Value;

    /* UnitDelay: '<S3>/SumA_State' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Sum: '<S3>/SumA_Add'
     *  Switch: '<S3>/SumA_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA;
  }

  /* End of Switch: '<S3>/SumB_Reset' */

  /* ZeroOrderHold: '<S16>/SpeedRef_1ms' incorporates:
   *  RateTransition: '<Root>/IqRef_Rate_Transition'
   *  RateTransition: '<Root>/StatusState_To_100us'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<Root>/Stateflow_Speed_Command_Gate' incorporates:
     *  Inport: '<Root>/ControlCommand'
     */
    if (PMSM_FOC_DualPlant_Controller_B.ControlEnable) {
      rtb_Current_Error =
        PMSM_FOC_DualPlant_Controller_U.ControlCommand.SpeedReferenceRpm;
    } else {
      rtb_Current_Error = 0.0F;
    }

    /* Gain: '<S16>/RpmToRad' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Product: '<Root>/Stateflow_Speed_Command_Gate'
     *  Sum: '<S16>/Speed_Error'
     *  ZeroOrderHold: '<S16>/SpeedFb_1ms'
     */
    rtb_Current_Error = (rtb_Current_Error -
                         PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm)
      * PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* UnitDelay: '<S16>/Integrator_State' */
    rtb_Vq_Select = PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k;

    /* Switch: '<S16>/Integrator_Reset_Select' */
    if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
      /* Sum: '<S16>/Integrator_Add' incorporates:
       *  Constant: '<S16>/Integrator_Zero'
       *  UnitDelay: '<S16>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k =
        PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_c;
    } else {
      /* Sum: '<S16>/Integrator_Add' incorporates:
       *  Gain: '<S16>/KiTs'
       *  UnitDelay: '<S16>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k +=
        FOC_Native_KiSpeed *
        PMSM_FOC_DualPlant_Controller_P.FOC_Native_SpeedPeriod *
        rtb_Current_Error;

      /* Saturate: '<S16>/Integrator_Limit' */
      if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k >
          FOC_Native_IqLimit) {
        /* Sum: '<S16>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k =
          FOC_Native_IqLimit;
      } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k <
                 -FOC_Native_IqLimit) {
        /* Sum: '<S16>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k =
          -FOC_Native_IqLimit;
      }

      /* End of Saturate: '<S16>/Integrator_Limit' */
    }

    /* End of Switch: '<S16>/Integrator_Reset_Select' */

    /* Sum: '<S16>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S16>/Kp'
     */
    rtb_Current_Error = FOC_Native_KpSpeed * rtb_Current_Error + rtb_Vq_Select;

    /* Saturate: '<S16>/Iq_Reference_Limit' */
    if (rtb_Current_Error > FOC_Native_IqLimit) {
      rtb_Current_Error = FOC_Native_IqLimit;
    } else if (rtb_Current_Error < -FOC_Native_IqLimit) {
      rtb_Current_Error = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S16>/Iq_Reference_Limit' */
    PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 =
      rtb_Current_Error;
    PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0 = rtb_StateCode;

    /* Update for RateTransition: '<Root>/StatusFault_To_100us' */
    PMSM_FOC_DualPlant_Controlle_DW.StatusFault_To_100us_Buffer0 =
      rtb_Supervisor_Fault_OR;
  }

  /* End of ZeroOrderHold: '<S16>/SpeedRef_1ms' */
  rate_scheduler();
}

/* Model initialize function */
void PMSM_FOC_DualPlant_Controller_v21_initialize(void)
{
  /* Start for RateTransition: '<Root>/IqRef_Rate_Transition'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   */
  PMSM_FOC_DualPlant_Controller_B.IqReference =
    PMSM_FOC_DualPlant_Controller_P.IqRef_Rate_Transition_InitialCo;

  /* Start for RateTransition: '<Root>/StatusState_To_100us' */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.MotorStateCode =
    PMSM_FOC_DualPlant_Controller_P.StatusState_To_100us_InitialCon;

  /* Start for RateTransition: '<Root>/StatusFault_To_100us' */
  PMSM_FOC_DualPlant_Controller_B.StatusFault_To_100us =
    PMSM_FOC_DualPlant_Controller_P.StatusFault_To_100us_InitialCon;

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
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_c =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_e;

  /* InitializeConditions for RateTransition: '<Root>/StatusState_To_100us' */
  PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.StatusState_To_100us_InitialCon;

  /* InitializeConditions for RateTransition: '<Root>/StatusFault_To_100us' */
  PMSM_FOC_DualPlant_Controlle_DW.StatusFault_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.StatusFault_To_100us_InitialCon;

  /* InitializeConditions for Sum: '<S16>/Integrator_Add' incorporates:
   *  UnitDelay: '<S16>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_k =
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
