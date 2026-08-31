/*
 * File: PMSM_FOC_DualPlant_Controller_v21.c
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
#include "rtwtypes.h"
#include <math.h>
#include "PMSM_FOC_DualPlant_Controller_v21_private.h"

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
                                   *   '<S7>/Integrator_Limit'
                                   *   '<S16>/Integrator_Limit'
                                   * D/q PI integrator absolute limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                   */
real32_T FOC_Native_DutyMax = 0.98F;   /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S17>/Duty_A_Limit'
                                        *   '<S17>/Duty_B_Limit'
                                        *   '<S17>/Duty_C_Limit'
                                        * Maximum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_DutyMin = 0.02F;   /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S17>/Duty_A_Limit'
                                        *   '<S17>/Duty_B_Limit'
                                        *   '<S17>/Duty_C_Limit'
                                        * Minimum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_FluxPM = 0.05F;    /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S5>/Flux_PM'
                                        * Permanent-magnet flux linkage Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_IqLimit = 8.0F;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S19>/Integrator_Limit'
                                        *   '<S19>/Iq_Reference_Limit'
                                        * Speed-loop q-axis current command limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KiCurrent = 500.0F;/* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S7>/KiTs'
                                        *   '<S16>/KiTs'
                                        * Current PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KiSpeed = 0.05F;   /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S19>/KiTs'
                                        * Speed PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KpCurrent = 1.0F;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S7>/Kp'
                                        *   '<S16>/Kp'
                                        * Current PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_KpSpeed = 0.02F;   /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S19>/Kp'
                                        * Speed PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_Ld = 0.001F;       /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S5>/Ld_x_Id'
                                        * D-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_Lq = 0.001F;       /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S5>/D_Decoupling'
                                        * Q-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_PolePairs = 4.0F;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S5>/Electrical_Speed'
                                        * Integer-valued motor pole-pair count stored as single for plant equation compatibility Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
real32_T FOC_Native_VoltageLimit = 26.0F;/* Variable: FOC_Native_VoltageLimit
                                          * Referenced by:
                                          *   '<S6>/Vd_Limit'
                                          *   '<S6>/Vq_Limit'
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
                                               * Referenced by: '<S9>/Max_Current_A'
                                               * Fast overcurrent trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                               */
real32_T PMSM_Protection_MaxDcBus_V = 60.0F;/* Variable: PMSM_Protection_MaxDcBus_V
                                             * Referenced by: '<S18>/Max_Vdc'
                                             * DC-bus overvoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                             */
real32_T PMSM_Protection_MaxSpeed_Rpm = 3000.0F;/* Variable: PMSM_Protection_MaxSpeed_Rpm
                                                 * Referenced by: '<S18>/Max_Speed_Rpm'
                                                 * Slow overspeed trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                                 */
real32_T PMSM_Protection_MinDcBus_V = 10.0F;/* Variable: PMSM_Protection_MinDcBus_V
                                             * Referenced by: '<S18>/Min_Vdc'
                                             * DC-bus undervoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                             */
uint16_T PMSM_Alignment_DurationTicks = 20U;/* Variable: PMSM_Alignment_DurationTicks
                                             * Referenced by: '<Root>/Motor_Supervisor_1ms'
                                             * Alignment duration in 1 ms supervisor ticks Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                             */
uint16_T PMSM_Calibration_SampleCount = 100U;/* Variable: PMSM_Calibration_SampleCount
                                              * Referenced by: '<S4>/Sample_Target'
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
  real32_T rtb_Current_Error;
  real32_T rtb_Current_Error_l;
  real32_T rtb_Duty_A_Limit;
  real32_T rtb_Duty_B_Limit;
  real32_T rtb_Duty_C_Limit;
  real32_T rtb_Electrical_Speed;
  real32_T rtb_Id_Sum;
  real32_T rtb_Iq_Sum;
  real32_T rtb_RpmToRad;
  real32_T rtb_Vd_Select;
  real32_T rtb_Vq_Select;
  uint8_T rtb_MotorStateCode;
  uint8_T rtb_StateCode;
  boolean_T rtb_AlignmentEnable_To_100us;
  boolean_T rtb_Any_Latched_Fault;
  boolean_T rtb_Count_Complete;
  boolean_T rtb_NoStop;
  boolean_T rtb_Overspeed_To_100us;
  boolean_T rtb_Overvoltage_To_100us;
  boolean_T rtb_Undervoltage_To_100us;

  /* RateTransition: '<Root>/AlignmentEnable_To_100us'
   *
   * Block description for '<Root>/AlignmentEnable_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_AlignmentEnable_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.AlignmentEnable_To_100us_Buffer;

  /* Trigonometry: '<S8>/Cos_Electrical_Angle' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Duty_A_Limit = cosf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.ElectricalAngleRad);

  /* Switch: '<S1>/Cos_Select' incorporates:
   *  Constant: '<S1>/Align_Cos'
   */
  if (rtb_AlignmentEnable_To_100us) {
    rtb_Duty_B_Limit = PMSM_FOC_DualPlant_Controller_P.PMSM_Alignment_Cos;
  } else {
    rtb_Duty_B_Limit = rtb_Duty_A_Limit;
  }

  /* End of Switch: '<S1>/Cos_Select' */

  /* Switch: '<S4>/OffsetA_Valid' incorporates:
   *  Constant: '<S4>/Zero'
   *  Product: '<S4>/OffsetA_Calculate'
   *  Product: '<S4>/OffsetB_Calculate'
   *  RelationalOperator: '<S4>/Count_Positive'
   *  Switch: '<S4>/OffsetB_Valid'
   *  UnitDelay: '<S4>/Count_State'
   *  UnitDelay: '<S4>/SumA_State'
   *  UnitDelay: '<S4>/SumB_State'
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

  /* End of Switch: '<S4>/OffsetA_Valid' */

  /* Sum: '<S4>/Correct_Ia' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Vq_Select = PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA -
    rtb_Vq_Select;

  /* Trigonometry: '<S8>/Sin_Electrical_Angle' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Duty_C_Limit = sinf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.ElectricalAngleRad);

  /* Gain: '<S2>/InvSqrt3' incorporates:
   *  Gain: '<S2>/Ib_x2'
   *  Inport: '<Root>/Measurement'
   *  Sum: '<S2>/Ia_Plus_2Ib'
   *  Sum: '<S4>/Correct_Ib'
   */
  rtb_Vd_Select = ((PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB -
                    rtb_Vd_Select) * PMSM_FOC_DualPlant_Controller_P.Ib_x2_Gain
                   + rtb_Vq_Select) *
    PMSM_FOC_DualPlant_Controller_P.NATIVE_INV_SQRT3;

  /* Sum: '<S15>/Id_Sum' incorporates:
   *  Product: '<S15>/Id_CosAlpha'
   *  Product: '<S15>/Id_SinBeta'
   */
  rtb_Id_Sum = rtb_Duty_A_Limit * rtb_Vq_Select + rtb_Duty_C_Limit *
    rtb_Vd_Select;

  /* Sum: '<S7>/Current_Error' incorporates:
   *  Constant: '<Root>/Id_Reference_Zero'
   *
   * Block description for '<Root>/Id_Reference_Zero':
   *  Field-oriented control d-axis current reference: Id*=0 A.
   */
  rtb_Current_Error = PMSM_FOC_DualPlant_Controller_P.Id_Reference_Zero_Value -
    rtb_Id_Sum;

  /* Gain: '<S5>/Electrical_Speed' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Electrical_Speed = PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S *
    FOC_Native_PolePairs *
    PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm;

  /* Sum: '<S15>/Iq_Sum' incorporates:
   *  Gain: '<S15>/Negative'
   *  Product: '<S15>/Iq_CosBeta'
   *  Product: '<S15>/Iq_SinAlpha'
   */
  rtb_Iq_Sum = rtb_Duty_C_Limit * rtb_Vq_Select *
    PMSM_FOC_DualPlant_Controller_P.Negative_Gain + rtb_Duty_A_Limit *
    rtb_Vd_Select;

  /* Switch: '<S1>/Vd_Select' incorporates:
   *  Constant: '<S1>/Align_Sin'
   *  Constant: '<S1>/Align_Vd'
   *  Switch: '<S1>/Sin_Select'
   */
  if (rtb_AlignmentEnable_To_100us) {
    rtb_Vd_Select = PMSM_Alignment_Vd_V;
    rtb_Duty_C_Limit = PMSM_FOC_DualPlant_Controller_P.PMSM_Alignment_Sin;
  } else {
    /* Sum: '<S6>/Vd_Raw' incorporates:
     *  Gain: '<S5>/D_Decoupling'
     *  Gain: '<S7>/Kp'
     *  Product: '<S5>/Omega_x_Iq'
     *  Sum: '<S7>/PI_Sum'
     *  UnitDelay: '<S7>/Integrator_State'
     */
    rtb_Vd_Select = (FOC_Native_KpCurrent * rtb_Current_Error +
                     PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE) +
      rtb_Electrical_Speed * rtb_Iq_Sum * -FOC_Native_Lq;

    /* Saturate: '<S6>/Vd_Limit' */
    if (rtb_Vd_Select > FOC_Native_VoltageLimit) {
      rtb_Vd_Select = FOC_Native_VoltageLimit;
    } else if (rtb_Vd_Select < -FOC_Native_VoltageLimit) {
      rtb_Vd_Select = -FOC_Native_VoltageLimit;
    }

    /* End of Saturate: '<S6>/Vd_Limit' */
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
    PMSM_FOC_DualPlant_Controller_B.IqRef_Rate_Transition =
      PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0;
  }

  /* End of RateTransition: '<Root>/IqRef_Rate_Transition' */

  /* Sum: '<S16>/Current_Error' */
  rtb_Current_Error_l = PMSM_FOC_DualPlant_Controller_B.IqRef_Rate_Transition -
    rtb_Iq_Sum;

  /* Switch: '<S1>/Vq_Select' incorporates:
   *  Constant: '<S1>/Align_Vq'
   */
  if (rtb_AlignmentEnable_To_100us) {
    rtb_Vq_Select = PMSM_Alignment_Vq_V;
  } else {
    /* Sum: '<S6>/Vq_Raw' incorporates:
     *  Constant: '<S5>/Flux_PM'
     *  Gain: '<S16>/Kp'
     *  Gain: '<S5>/Ld_x_Id'
     *  Product: '<S5>/Q_Feedforward'
     *  Sum: '<S16>/PI_Sum'
     *  Sum: '<S5>/Flux_Linkage'
     *  UnitDelay: '<S16>/Integrator_State'
     */
    rtb_Vq_Select = (FOC_Native_Ld * rtb_Id_Sum + FOC_Native_FluxPM) *
      rtb_Electrical_Speed + (FOC_Native_KpCurrent * rtb_Current_Error_l +
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l);

    /* Saturate: '<S6>/Vq_Limit' */
    if (rtb_Vq_Select > FOC_Native_VoltageLimit) {
      rtb_Vq_Select = FOC_Native_VoltageLimit;
    } else if (rtb_Vq_Select < -FOC_Native_VoltageLimit) {
      rtb_Vq_Select = -FOC_Native_VoltageLimit;
    }

    /* End of Saturate: '<S6>/Vq_Limit' */
  }

  /* End of Switch: '<S1>/Vq_Select' */

  /* Sum: '<S13>/Valpha_Sum' incorporates:
   *  Product: '<S13>/Valpha_CosVd'
   *  Product: '<S13>/Valpha_SinVq'
   */
  rtb_Duty_A_Limit = rtb_Duty_B_Limit * rtb_Vd_Select - rtb_Duty_C_Limit *
    rtb_Vq_Select;

  /* Sum: '<S13>/Vbeta_Sum' incorporates:
   *  Product: '<S13>/Vbeta_CosVq'
   *  Product: '<S13>/Vbeta_SinVd'
   */
  rtb_Duty_C_Limit = rtb_Duty_C_Limit * rtb_Vd_Select + rtb_Duty_B_Limit *
    rtb_Vq_Select;

  /* Sum: '<S12>/Phase_Vb' incorporates:
   *  Gain: '<S12>/Vb_Alpha'
   *  Gain: '<S12>/Vb_Beta'
   */
  rtb_Duty_B_Limit = PMSM_FOC_DualPlant_Controller_P.Vb_Alpha_Gain *
    rtb_Duty_A_Limit + PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Duty_C_Limit;

  /* Sum: '<S12>/Phase_Vc' incorporates:
   *  Gain: '<S12>/Vc_Alpha'
   *  Gain: '<S12>/Vc_Beta'
   */
  rtb_Duty_C_Limit = PMSM_FOC_DualPlant_Controller_P.Vc_Alpha_Gain *
    rtb_Duty_A_Limit + -PMSM_FOC_DualPlant_Controller_P.NATIVE_SQRT3_BY2 *
    rtb_Duty_C_Limit;

  /* Gain: '<S17>/Common_Mode' incorporates:
   *  MinMax: '<S17>/Phase_Maximum'
   *  MinMax: '<S17>/Phase_Minimum'
   *  Sum: '<S17>/Max_Plus_Min'
   */
  rtb_Electrical_Speed = (fmaxf(fmaxf(rtb_Duty_A_Limit, rtb_Duty_B_Limit),
    rtb_Duty_C_Limit) + fminf(fminf(rtb_Duty_A_Limit, rtb_Duty_B_Limit),
    rtb_Duty_C_Limit)) * PMSM_FOC_DualPlant_Controller_P.Common_Mode_Gain;

  /* RateTransition: '<Root>/StatusState_To_100us' */
  rtb_MotorStateCode =
    PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0;

  /* RateTransition: '<Root>/HardwareGate_To_1ms' incorporates:
   *  RateTransition: '<Root>/EmergencyStop_To_1ms'
   *
   * Block description for '<Root>/HardwareGate_To_1ms':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   *
   * Block description for '<Root>/EmergencyStop_To_1ms':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* RateTransition: '<Root>/HardwareGate_To_1ms' incorporates:
     *  Inport: '<Root>/ControlCommand'
     *
     * Block description for '<Root>/HardwareGate_To_1ms':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controller_B.HardwareGate_To_1ms =
      PMSM_FOC_DualPlant_Controller_U.ControlCommand.HardwareGate;

    /* Logic: '<S3>/NoEmergency' incorporates:
     *  Inport: '<Root>/ControlCommand'
     */
    PMSM_FOC_DualPlant_Controller_B.NoEmergency =
      !PMSM_FOC_DualPlant_Controller_U.ControlCommand.EmergencyStop;

    /* Logic: '<S3>/NoDriverFault' incorporates:
     *  Inport: '<Root>/ControlCommand'
     *  RateTransition: '<Root>/DriverFault_To_1ms'
     *
     * Block description for '<Root>/DriverFault_To_1ms':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controller_B.NoDriverFault =
      !PMSM_FOC_DualPlant_Controller_U.ControlCommand.DriverFault;
  }

  /* End of RateTransition: '<Root>/HardwareGate_To_1ms' */

  /* Logic: '<S3>/Accept_Reset' incorporates:
   *  Inport: '<Root>/ControlCommand'
   *  Logic: '<S3>/NoStart'
   */
  rtb_AlignmentEnable_To_100us =
    (PMSM_FOC_DualPlant_Controller_U.ControlCommand.FaultResetRequest &&
     PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest &&
     (!PMSM_FOC_DualPlant_Controller_U.ControlCommand.StartRequest) &&
     PMSM_FOC_DualPlant_Controller_B.NoEmergency &&
     PMSM_FOC_DualPlant_Controller_B.NoDriverFault &&
     PMSM_FOC_DualPlant_Controller_B.HardwareGate_To_1ms);

  /* RateTransition: '<Root>/Overspeed_To_100us'
   *
   * Block description for '<Root>/Overspeed_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_Overspeed_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.Overspeed_To_100us_Buffer0;

  /* RateTransition: '<Root>/Undervoltage_To_100us'
   *
   * Block description for '<Root>/Undervoltage_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_Undervoltage_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_To_100us_Buffer0;

  /* RateTransition: '<Root>/Overvoltage_To_100us'
   *
   * Block description for '<Root>/Overvoltage_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_Overvoltage_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_To_100us_Buffer0;

  /* Logic: '<S9>/Any_Overcurrent' incorporates:
   *  Abs: '<S9>/Abs_Ia'
   *  Abs: '<S9>/Abs_Ib'
   *  Constant: '<S9>/Max_Current_A'
   *  Inport: '<Root>/Measurement'
   *  RelationalOperator: '<S9>/OverCurrentA'
   *  RelationalOperator: '<S9>/OverCurrentB'
   */
  rtb_NoStop = ((fabsf(PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA)
                 > PMSM_Protection_MaxCurrent_A) || (fabsf
    (PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB) >
    PMSM_Protection_MaxCurrent_A));

  /* Logic: '<S9>/Invalid_Measurement' incorporates:
   *  Inport: '<Root>/Measurement'
   */
  rtb_Count_Complete = !PMSM_FOC_DualPlant_Controller_U.Measurement.Valid;

  /* Logic: '<S10>/Keep_Latches' incorporates:
   *  Inport: '<Root>/ControlCommand'
   *  Logic: '<S10>/Any_Raw_Fault'
   *  Logic: '<S10>/No_Raw_Fault'
   *  Logic: '<S10>/Reset_Permitted'
   */
  rtb_Any_Latched_Fault = ((!rtb_AlignmentEnable_To_100us) ||
    (PMSM_FOC_DualPlant_Controller_U.ControlCommand.EmergencyStop ||
     PMSM_FOC_DualPlant_Controller_U.ControlCommand.DriverFault || rtb_NoStop ||
     rtb_Count_Complete || rtb_Overspeed_To_100us || rtb_Undervoltage_To_100us ||
     rtb_Overvoltage_To_100us));

  /* Logic: '<S10>/SoftwareOvercurrent_Latched' incorporates:
   *  Logic: '<S10>/SoftwareOvercurrent_Set_Or_Hold'
   *  UnitDelay: '<S10>/SoftwareOvercurrent_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State = ((rtb_NoStop
    || PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State) &&
    rtb_Any_Latched_Fault);

  /* RateTransition: '<Root>/SupervisorPwmRequest_To_100us'
   *
   * Block description for '<Root>/SupervisorPwmRequest_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_NoStop = PMSM_FOC_DualPlant_Controlle_DW.SupervisorPwmRequest_To_100us_B;

  /* Logic: '<S10>/EmergencyStop_Latched' incorporates:
   *  Inport: '<Root>/ControlCommand'
   *  Logic: '<S10>/EmergencyStop_Set_Or_Hold'
   *  UnitDelay: '<S10>/EmergencyStop_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT =
    ((PMSM_FOC_DualPlant_Controller_U.ControlCommand.EmergencyStop ||
      PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/DriverFault_Latched' incorporates:
   *  Inport: '<Root>/ControlCommand'
   *  Logic: '<S10>/DriverFault_Set_Or_Hold'
   *  UnitDelay: '<S10>/DriverFault_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE =
    ((PMSM_FOC_DualPlant_Controller_U.ControlCommand.DriverFault ||
      PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/MeasurementInvalid_Latched' incorporates:
   *  Logic: '<S10>/MeasurementInvalid_Set_Or_Hold'
   *  UnitDelay: '<S10>/MeasurementInvalid_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_ =
    ((rtb_Count_Complete ||
      PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/Overspeed_Latched' incorporates:
   *  Logic: '<S10>/Overspeed_Set_Or_Hold'
   *  UnitDelay: '<S10>/Overspeed_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE =
    ((rtb_Overspeed_To_100us ||
      PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/Undervoltage_Latched' incorporates:
   *  Logic: '<S10>/Undervoltage_Set_Or_Hold'
   *  UnitDelay: '<S10>/Undervoltage_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE =
    ((rtb_Undervoltage_To_100us ||
      PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/Overvoltage_Latched' incorporates:
   *  Logic: '<S10>/Overvoltage_Set_Or_Hold'
   *  UnitDelay: '<S10>/Overvoltage_Latch_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE =
    ((rtb_Overvoltage_To_100us ||
      PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE) &&
     rtb_Any_Latched_Fault);

  /* Logic: '<S10>/Any_Latched_Fault' incorporates:
   *  UnitDelay: '<S10>/DriverFault_Latch_State'
   *  UnitDelay: '<S10>/EmergencyStop_Latch_State'
   *  UnitDelay: '<S10>/MeasurementInvalid_Latch_State'
   *  UnitDelay: '<S10>/Overspeed_Latch_State'
   *  UnitDelay: '<S10>/Overvoltage_Latch_State'
   *  UnitDelay: '<S10>/SoftwareOvercurrent_Latch_State'
   *  UnitDelay: '<S10>/Undervoltage_Latch_State'
   */
  rtb_Any_Latched_Fault =
    (PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT ||
     PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE ||
     PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State ||
     PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_ ||
     PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE ||
     PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE ||
     PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE);

  /* Switch: '<S10>/FaultCode_Priority_1' incorporates:
   *  Constant: '<S10>/FaultCode_1'
   *  Switch: '<S10>/FaultCode_Priority_2'
   *  Switch: '<S10>/FaultCode_Priority_3'
   *  Switch: '<S10>/FaultCode_Priority_4'
   *  Switch: '<S10>/FaultCode_Priority_5'
   *  Switch: '<S10>/FaultCode_Priority_6'
   *  Switch: '<S10>/FaultCode_Priority_7'
   *  UnitDelay: '<S10>/DriverFault_Latch_State'
   *  UnitDelay: '<S10>/EmergencyStop_Latch_State'
   *  UnitDelay: '<S10>/MeasurementInvalid_Latch_State'
   *  UnitDelay: '<S10>/Overspeed_Latch_State'
   *  UnitDelay: '<S10>/Overvoltage_Latch_State'
   *  UnitDelay: '<S10>/SoftwareOvercurrent_Latch_State'
   *  UnitDelay: '<S10>/Undervoltage_Latch_State'
   */
  if (PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT) {
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_1_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE) {
    /* Switch: '<S10>/FaultCode_Priority_2' incorporates:
     *  Constant: '<S10>/FaultCode_2'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_2_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State) {
    /* Switch: '<S10>/FaultCode_Priority_3' incorporates:
     *  Constant: '<S10>/FaultCode_3'
     *  Switch: '<S10>/FaultCode_Priority_2'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_3_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_) {
    /* Switch: '<S10>/FaultCode_Priority_4' incorporates:
     *  Constant: '<S10>/FaultCode_4'
     *  Switch: '<S10>/FaultCode_Priority_2'
     *  Switch: '<S10>/FaultCode_Priority_3'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_4_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE) {
    /* Switch: '<S10>/FaultCode_Priority_5' incorporates:
     *  Constant: '<S10>/FaultCode_5'
     *  Switch: '<S10>/FaultCode_Priority_2'
     *  Switch: '<S10>/FaultCode_Priority_3'
     *  Switch: '<S10>/FaultCode_Priority_4'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_5_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE) {
    /* Switch: '<S10>/FaultCode_Priority_6' incorporates:
     *  Constant: '<S10>/FaultCode_6'
     *  Switch: '<S10>/FaultCode_Priority_2'
     *  Switch: '<S10>/FaultCode_Priority_3'
     *  Switch: '<S10>/FaultCode_Priority_4'
     *  Switch: '<S10>/FaultCode_Priority_5'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_6_Value;
  } else if (PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE) {
    /* Switch: '<S10>/FaultCode_Priority_7' incorporates:
     *  Constant: '<S10>/FaultCode_7'
     *  Switch: '<S10>/FaultCode_Priority_2'
     *  Switch: '<S10>/FaultCode_Priority_3'
     *  Switch: '<S10>/FaultCode_Priority_4'
     *  Switch: '<S10>/FaultCode_Priority_5'
     *  Switch: '<S10>/FaultCode_Priority_6'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_7_Value;
  } else {
    /* Switch: '<S10>/FaultCode_Priority_5' incorporates:
     *  Constant: '<S10>/FaultCode_0'
     *  Switch: '<S10>/FaultCode_Priority_2'
     *  Switch: '<S10>/FaultCode_Priority_3'
     *  Switch: '<S10>/FaultCode_Priority_4'
     *  Switch: '<S10>/FaultCode_Priority_6'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultCode =
      PMSM_FOC_DualPlant_Controller_P.FaultCode_0_Value;
  }

  /* End of Switch: '<S10>/FaultCode_Priority_1' */

  /* Sum: '<S17>/Duty_A_Plus_Half' incorporates:
   *  Constant: '<S17>/Duty_Half'
   *  Inport: '<Root>/Measurement'
   *  Product: '<S17>/Duty_A_Divide_Vdc'
   *  Sum: '<S17>/Phase_A_Plus_Common'
   */
  rtb_Duty_A_Limit = (rtb_Duty_A_Limit + rtb_Electrical_Speed) /
    PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S17>/Duty_A_Limit' */
  if (rtb_Duty_A_Limit > FOC_Native_DutyMax) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = FOC_Native_DutyMax;
  } else if (rtb_Duty_A_Limit < FOC_Native_DutyMin) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = FOC_Native_DutyMin;
  } else {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyA = rtb_Duty_A_Limit;
  }

  /* End of Saturate: '<S17>/Duty_A_Limit' */

  /* Sum: '<S17>/Duty_B_Plus_Half' incorporates:
   *  Constant: '<S17>/Duty_Half'
   *  Inport: '<Root>/Measurement'
   *  Product: '<S17>/Duty_B_Divide_Vdc'
   *  Sum: '<S17>/Phase_B_Plus_Common'
   */
  rtb_Duty_A_Limit = (rtb_Duty_B_Limit + rtb_Electrical_Speed) /
    PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S17>/Duty_B_Limit' */
  if (rtb_Duty_A_Limit > FOC_Native_DutyMax) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = FOC_Native_DutyMax;
  } else if (rtb_Duty_A_Limit < FOC_Native_DutyMin) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = FOC_Native_DutyMin;
  } else {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyB = rtb_Duty_A_Limit;
  }

  /* End of Saturate: '<S17>/Duty_B_Limit' */

  /* Sum: '<S17>/Duty_C_Plus_Half' incorporates:
   *  Constant: '<S17>/Duty_Half'
   *  Inport: '<Root>/Measurement'
   *  Product: '<S17>/Duty_C_Divide_Vdc'
   *  Sum: '<S17>/Phase_C_Plus_Common'
   */
  rtb_Duty_A_Limit = (rtb_Duty_C_Limit + rtb_Electrical_Speed) /
    PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage +
    PMSM_FOC_DualPlant_Controller_P.Duty_Half_Value;

  /* Saturate: '<S17>/Duty_C_Limit' */
  if (rtb_Duty_A_Limit > FOC_Native_DutyMax) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = FOC_Native_DutyMax;
  } else if (rtb_Duty_A_Limit < FOC_Native_DutyMin) {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = FOC_Native_DutyMin;
  } else {
    /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
     *  Outport: '<Root>/ControlStatus'
     */
    PMSM_FOC_DualPlant_Controller_Y.ControlStatus.DutyC = rtb_Duty_A_Limit;
  }

  /* End of Saturate: '<S17>/Duty_C_Limit' */

  /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
   *  Outport: '<Root>/ControlStatus'
   */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IqReference =
    PMSM_FOC_DualPlant_Controller_B.IqRef_Rate_Transition;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IdMeasured = rtb_Id_Sum;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.IqMeasured = rtb_Iq_Sum;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VdCommand = rtb_Vd_Select;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VqCommand = rtb_Vq_Select;

  /* Logic: '<S10>/Interlock_OK' incorporates:
   *  Chart: '<Root>/Motor_Supervisor_1ms'
   */
  rtb_Overspeed_To_100us = !rtb_Any_Latched_Fault;

  /* BusCreator generated from: '<Root>/ControlStatus' incorporates:
   *  Constant: '<Root>/VoltageLimit_Inactive'
   *  DataTypeConversion: '<S10>/FaultBit0_U32'
   *  DataTypeConversion: '<S10>/FaultBit1_U32'
   *  DataTypeConversion: '<S10>/FaultBit2_U32'
   *  DataTypeConversion: '<S10>/FaultBit3_U32'
   *  DataTypeConversion: '<S10>/FaultBit4_U32'
   *  DataTypeConversion: '<S10>/FaultBit5_U32'
   *  DataTypeConversion: '<S10>/FaultBit6_U32'
   *  Gain: '<S10>/FaultBit0_Weight'
   *  Gain: '<S10>/FaultBit1_Weight'
   *  Gain: '<S10>/FaultBit2_Weight'
   *  Gain: '<S10>/FaultBit3_Weight'
   *  Gain: '<S10>/FaultBit4_Weight'
   *  Gain: '<S10>/FaultBit5_Weight'
   *  Gain: '<S10>/FaultBit6_Weight'
   *  Inport: '<Root>/ControlCommand'
   *  Inport: '<Root>/Measurement'
   *  Logic: '<S10>/Interlock_OK'
   *  Logic: '<S11>/Three_Level_AND'
   *  Outport: '<Root>/ControlStatus'
   *  Sum: '<S10>/Pack_Fault_Bits'
   *  UnitDelay: '<S10>/DriverFault_Latch_State'
   *  UnitDelay: '<S10>/EmergencyStop_Latch_State'
   *  UnitDelay: '<S10>/MeasurementInvalid_Latch_State'
   *  UnitDelay: '<S10>/Overspeed_Latch_State'
   *  UnitDelay: '<S10>/Overvoltage_Latch_State'
   *  UnitDelay: '<S10>/SoftwareOvercurrent_Latch_State'
   *  UnitDelay: '<S10>/Undervoltage_Latch_State'
   */
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.PwmEnable = (rtb_NoStop &&
    rtb_Overspeed_To_100us &&
    PMSM_FOC_DualPlant_Controller_U.ControlCommand.HardwareGate);
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.MotorStateCode =
    rtb_MotorStateCode;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.FaultBits =
    (((((((PMSM_FOC_DualPlant_Controller_P.FaultBit0_Weight_Gain *
           PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT) >>
          31) + ((PMSM_FOC_DualPlant_Controller_P.FaultBit1_Weight_Gain *
                  PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE)
                 >> 30)) +
        ((PMSM_FOC_DualPlant_Controller_P.FaultBit2_Weight_Gain *
          PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State) >> 29))
       + ((PMSM_FOC_DualPlant_Controller_P.FaultBit3_Weight_Gain *
           PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_) >>
          28)) + ((PMSM_FOC_DualPlant_Controller_P.FaultBit4_Weight_Gain *
                   PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE)
                  >> 27)) +
     ((PMSM_FOC_DualPlant_Controller_P.FaultBit5_Weight_Gain *
       PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE) >> 26))
    + ((PMSM_FOC_DualPlant_Controller_P.FaultBit6_Weight_Gain *
        PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE) >> 25);
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.CurrentLimitActive =
    PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.VoltageLimitActive =
    PMSM_FOC_DualPlant_Controller_P.VoltageLimit_Inactive_Value;
  PMSM_FOC_DualPlant_Controller_Y.ControlStatus.MeasurementValid =
    PMSM_FOC_DualPlant_Controller_U.Measurement.Valid;

  /* RateTransition: '<Root>/AnyFault_To_1ms' incorporates:
   *  RateTransition: '<Root>/CalibrationDone_To_1ms'
   *  ZeroOrderHold: '<S19>/SpeedRef_1ms'
   *
   * Block description for '<Root>/AnyFault_To_1ms':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   *
   * Block description for '<Root>/CalibrationDone_To_1ms':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* Chart: '<Root>/Motor_Supervisor_1ms' incorporates:
     *  Constant: '<S4>/Sample_Target'
     *  Inport: '<Root>/ControlCommand'
     *  Logic: '<S3>/Accept_Start'
     *  Logic: '<S3>/NoStop'
     *  RelationalOperator: '<S4>/Count_Complete'
     *  UnitDelay: '<S4>/Count_State'
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
      PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
      rtb_StateCode = 1U;
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
      PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro ==
               PMSM_FOC_DualPlant_Con_IN_FAULT) {
      rtb_StateCode = 6U;
      if (rtb_AlignmentEnable_To_100us && rtb_Overspeed_To_100us &&
          PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest) {
        PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
          PMSM_FOC_DualPlan_IN_SUPERVISED;
        PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
        PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
          PMSM_FOC_DualPlant_Cont_IN_INIT;
        PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
        PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
        rtb_StateCode = 1U;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
      }

      /* case IN_SUPERVISED: */
    } else if (rtb_Any_Latched_Fault) {
      PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
        PMSM_FOC_Dua_IN_NO_ACTIVE_CHILD;
      PMSM_FOC_DualPlant_Controlle_DW.is_c3_PMSM_FOC_DualPlant_Contro =
        PMSM_FOC_DualPlant_Con_IN_FAULT;
      PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
      PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
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
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = true;
          rtb_StateCode = 5U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = false;
        } else if (PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Cont_IN_INIT;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
          rtb_StateCode = 1U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        }
        break;

       case PMSM_FOC_DualPlant_Con_IN_CALIB:
        rtb_StateCode = 3U;
        if (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE >=
            PMSM_Calibration_SampleCount) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Con_IN_ALIGN;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = true;
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
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
          rtb_StateCode = 6U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        } else if (PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Cont_IN_INIT;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
          rtb_StateCode = 1U;
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
        PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
        rtb_StateCode = 2U;
        PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
        PMSM_FOC_DualPlant_Controller_B.CalibrationReset = false;
        PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
        PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        break;

       case PMSM_FOC_DualPlant_Con_IN_READY:
        rtb_StateCode = 2U;

        /* Logic: '<S3>/NoStop' incorporates:
         *  Inport: '<Root>/ControlCommand'
         */
        rtb_AlignmentEnable_To_100us =
          !PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest;
        if (PMSM_FOC_DualPlant_Controller_U.ControlCommand.StartRequest &&
            rtb_AlignmentEnable_To_100us &&
            PMSM_FOC_DualPlant_Controller_B.NoEmergency &&
            PMSM_FOC_DualPlant_Controller_B.NoDriverFault &&
            PMSM_FOC_DualPlant_Controller_B.HardwareGate_To_1ms &&
            rtb_AlignmentEnable_To_100us) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Con_IN_CALIB;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
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
        if (PMSM_FOC_DualPlant_Controller_U.ControlCommand.StopRequest) {
          PMSM_FOC_DualPlant_Controlle_DW.temporalCounter_i1 = 0U;
          PMSM_FOC_DualPlant_Controlle_DW.is_SUPERVISED =
            PMSM_FOC_DualPlant_Cont_IN_INIT;
          PMSM_FOC_DualPlant_Controller_B.ControlEnable = false;
          PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest = false;
          rtb_StateCode = 1U;
          PMSM_FOC_DualPlant_Controller_B.CalibrationEnable = false;
          PMSM_FOC_DualPlant_Controller_B.CalibrationReset = true;
          PMSM_FOC_DualPlant_Controller_B.AlignmentEnable = false;
          PMSM_FOC_DualPlant_Controller_B.ControllerReset = true;
        }
        break;
      }
    }

    /* Product: '<Root>/Stateflow_Speed_Command_Gate' incorporates:
     *  Inport: '<Root>/ControlCommand'
     */
    if (PMSM_FOC_DualPlant_Controller_B.ControlEnable) {
      rtb_RpmToRad =
        PMSM_FOC_DualPlant_Controller_U.ControlCommand.SpeedReferenceRpm;
    } else {
      rtb_RpmToRad = 0.0F;
    }

    /* Gain: '<S19>/RpmToRad' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Product: '<Root>/Stateflow_Speed_Command_Gate'
     *  Sum: '<S19>/Speed_Error'
     *  ZeroOrderHold: '<S19>/SpeedFb_1ms'
     */
    rtb_RpmToRad = (rtb_RpmToRad -
                    PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm)
      * PMSM_FOC_DualPlant_Controller_P.NATIVE_RPM_TO_RAD_S;

    /* UnitDelay: '<S19>/Integrator_State' */
    rtb_Duty_B_Limit = PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m;

    /* Switch: '<S19>/Integrator_Reset_Select' */
    if (PMSM_FOC_DualPlant_Controller_B.ControllerReset) {
      /* Sum: '<S19>/Integrator_Add' incorporates:
       *  Constant: '<S19>/Integrator_Zero'
       *  UnitDelay: '<S19>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
        PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_f;
    } else {
      /* Sum: '<S19>/Integrator_Add' incorporates:
       *  Gain: '<S19>/KiTs'
       *  UnitDelay: '<S19>/Integrator_State'
       */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m +=
        FOC_Native_KiSpeed *
        PMSM_FOC_DualPlant_Controller_P.FOC_Native_SpeedPeriod * rtb_RpmToRad;

      /* Saturate: '<S19>/Integrator_Limit' */
      if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m >
          FOC_Native_IqLimit) {
        /* Sum: '<S19>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
          FOC_Native_IqLimit;
      } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m <
                 -FOC_Native_IqLimit) {
        /* Sum: '<S19>/Integrator_Add' */
        PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
          -FOC_Native_IqLimit;
      }

      /* End of Saturate: '<S19>/Integrator_Limit' */
    }

    /* End of Switch: '<S19>/Integrator_Reset_Select' */

    /* Sum: '<S19>/Iq_Reference_Sum' incorporates:
     *  Gain: '<S19>/Kp'
     */
    rtb_RpmToRad = FOC_Native_KpSpeed * rtb_RpmToRad + rtb_Duty_B_Limit;

    /* Saturate: '<S19>/Iq_Reference_Limit' */
    if (rtb_RpmToRad > FOC_Native_IqLimit) {
      rtb_RpmToRad = FOC_Native_IqLimit;
    } else if (rtb_RpmToRad < -FOC_Native_IqLimit) {
      rtb_RpmToRad = -FOC_Native_IqLimit;
    }

    /* End of Saturate: '<S19>/Iq_Reference_Limit' */
  }

  /* End of RateTransition: '<Root>/AnyFault_To_1ms' */

  /* RateTransition: '<Root>/CalibrationEnable_To_100us'
   *
   * Block description for '<Root>/CalibrationEnable_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_AlignmentEnable_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.CalibrationEnable_To_100us_Buff;

  /* RateTransition: '<Root>/CalibrationReset_To_100us'
   *
   * Block description for '<Root>/CalibrationReset_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_Any_Latched_Fault =
    PMSM_FOC_DualPlant_Controlle_DW.CalibrationReset_To_100us_Buffe;

  /* Switch: '<S4>/Count_Reset' incorporates:
   *  Constant: '<S4>/Sample_Target'
   *  Constant: '<S4>/Zero'
   *  Logic: '<S4>/Accumulate_Enable'
   *  RelationalOperator: '<S4>/Count_Below_Target'
   *  Switch: '<S4>/Count_Hold'
   *  Switch: '<S4>/SumA_Hold'
   *  Switch: '<S4>/SumA_Reset'
   *  Switch: '<S4>/SumB_Hold'
   *  Switch: '<S4>/SumB_Reset'
   *  UnitDelay: '<S4>/Count_State'
   *  UnitDelay: '<S4>/SumA_State'
   *  UnitDelay: '<S4>/SumB_State'
   */
  if (rtb_Any_Latched_Fault) {
    PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
    PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
    PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Zero_Value;
  } else if (rtb_AlignmentEnable_To_100us &&
             (PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE <
              PMSM_Calibration_SampleCount)) {
    /* UnitDelay: '<S4>/Count_State' incorporates:
     *  Constant: '<S4>/One'
     *  Sum: '<S4>/Count_Add'
     *  Switch: '<S4>/Count_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_P.One_Value;

    /* UnitDelay: '<S4>/SumA_State' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Sum: '<S4>/SumA_Add'
     *  Switch: '<S4>/SumA_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentA;

    /* UnitDelay: '<S4>/SumB_State' incorporates:
     *  Inport: '<Root>/Measurement'
     *  Sum: '<S4>/SumB_Add'
     *  Switch: '<S4>/SumB_Hold'
     */
    PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE +=
      PMSM_FOC_DualPlant_Controller_U.Measurement.PhaseCurrentB;
  }

  /* End of Switch: '<S4>/Count_Reset' */

  /* RateTransition: '<Root>/ControllerReset_To_100us'
   *
   * Block description for '<Root>/ControllerReset_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  rtb_AlignmentEnable_To_100us =
    PMSM_FOC_DualPlant_Controlle_DW.ControllerReset_To_100us_Buffer;

  /* Switch: '<S16>/Integrator_Reset_Select' incorporates:
   *  Switch: '<S7>/Integrator_Reset_Select'
   */
  if (rtb_AlignmentEnable_To_100us) {
    /* Sum: '<S16>/Integrator_Add' incorporates:
     *  Constant: '<S16>/Integrator_Zero'
     *  UnitDelay: '<S16>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value_d;

    /* Sum: '<S7>/Integrator_Add' incorporates:
     *  Constant: '<S7>/Integrator_Zero'
     *  UnitDelay: '<S7>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
      PMSM_FOC_DualPlant_Controller_P.Integrator_Zero_Value;
  } else {
    /* Gain: '<S16>/KiTs' incorporates:
     *  Gain: '<S7>/KiTs'
     */
    rtb_Duty_B_Limit = FOC_Native_KiCurrent *
      PMSM_FOC_DualPlant_Controller_P.FOC_Native_CurrentPeriod;

    /* Sum: '<S16>/Integrator_Add' incorporates:
     *  Gain: '<S16>/KiTs'
     *  UnitDelay: '<S16>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l +=
      rtb_Duty_B_Limit * rtb_Current_Error_l;

    /* Saturate: '<S16>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S16>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S16>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S16>/Integrator_Limit' */

    /* Sum: '<S7>/Integrator_Add' incorporates:
     *  Gain: '<S7>/KiTs'
     *  UnitDelay: '<S7>/Integrator_State'
     */
    PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE += rtb_Duty_B_Limit *
      rtb_Current_Error;

    /* Saturate: '<S7>/Integrator_Limit' */
    if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE >
        FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S7>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
        FOC_Native_CurrentIntegratorLimit;
    } else if (PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE <
               -FOC_Native_CurrentIntegratorLimit) {
      /* Sum: '<S7>/Integrator_Add' */
      PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE =
        -FOC_Native_CurrentIntegratorLimit;
    }

    /* End of Saturate: '<S7>/Integrator_Limit' */
  }

  /* End of Switch: '<S16>/Integrator_Reset_Select' */

  /* ZeroOrderHold: '<S18>/Speed_Sample_1ms' incorporates:
   *  RateTransition: '<Root>/IqRef_Rate_Transition'
   *  RateTransition: '<Root>/StatusState_To_100us'
   *  RateTransition: '<Root>/SupervisorPwmRequest_To_100us'
   *
   * Block description for '<Root>/IqRef_Rate_Transition':
   *  Explicit deterministic transfer from 1 ms speed task to 100 us current
   *  task.
   *
   * Block description for '<Root>/SupervisorPwmRequest_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  if (PMSM_FOC_DualPlant_Controlle_M->Timing.TaskCounters.TID[1] == 0) {
    /* RelationalOperator: '<S18>/OverSpeed' incorporates:
     *  Abs: '<S18>/Abs_Speed'
     *  Constant: '<S18>/Max_Speed_Rpm'
     *  Inport: '<Root>/Measurement'
     */
    rtb_AlignmentEnable_To_100us = (fabsf
      (PMSM_FOC_DualPlant_Controller_U.Measurement.MechanicalSpeedRpm) >
      PMSM_Protection_MaxSpeed_Rpm);

    /* Update for RateTransition: '<Root>/AlignmentEnable_To_100us'
     *
     * Block description for '<Root>/AlignmentEnable_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.AlignmentEnable_To_100us_Buffer =
      PMSM_FOC_DualPlant_Controller_B.AlignmentEnable;
    PMSM_FOC_DualPlant_Controlle_DW.IqRef_Rate_Transition_Buffer0 = rtb_RpmToRad;
    PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0 = rtb_StateCode;

    /* Update for RateTransition: '<Root>/Overspeed_To_100us'
     *
     * Block description for '<Root>/Overspeed_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.Overspeed_To_100us_Buffer0 =
      rtb_AlignmentEnable_To_100us;

    /* Update for RateTransition: '<Root>/Undervoltage_To_100us' incorporates:
     *  Constant: '<S18>/Min_Vdc'
     *  Inport: '<Root>/Measurement'
     *  RelationalOperator: '<S18>/UnderVoltage'
     *  ZeroOrderHold: '<S18>/Vdc_Sample_1ms'
     *
     * Block description for '<Root>/Undervoltage_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_To_100us_Buffer0 =
      (PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage <
       PMSM_Protection_MinDcBus_V);

    /* Update for RateTransition: '<Root>/Overvoltage_To_100us' incorporates:
     *  Constant: '<S18>/Max_Vdc'
     *  Inport: '<Root>/Measurement'
     *  RelationalOperator: '<S18>/OverVoltage'
     *  ZeroOrderHold: '<S18>/Vdc_Sample_1ms'
     *
     * Block description for '<Root>/Overvoltage_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_To_100us_Buffer0 =
      (PMSM_FOC_DualPlant_Controller_U.Measurement.DcBusVoltage >
       PMSM_Protection_MaxDcBus_V);
    PMSM_FOC_DualPlant_Controlle_DW.SupervisorPwmRequest_To_100us_B =
      PMSM_FOC_DualPlant_Controller_B.SupervisorPwmRequest;

    /* Update for RateTransition: '<Root>/CalibrationEnable_To_100us'
     *
     * Block description for '<Root>/CalibrationEnable_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.CalibrationEnable_To_100us_Buff =
      PMSM_FOC_DualPlant_Controller_B.CalibrationEnable;

    /* Update for RateTransition: '<Root>/CalibrationReset_To_100us'
     *
     * Block description for '<Root>/CalibrationReset_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.CalibrationReset_To_100us_Buffe =
      PMSM_FOC_DualPlant_Controller_B.CalibrationReset;

    /* Update for RateTransition: '<Root>/ControllerReset_To_100us'
     *
     * Block description for '<Root>/ControllerReset_To_100us':
     *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
     */
    PMSM_FOC_DualPlant_Controlle_DW.ControllerReset_To_100us_Buffer =
      PMSM_FOC_DualPlant_Controller_B.ControllerReset;
  }

  /* End of ZeroOrderHold: '<S18>/Speed_Sample_1ms' */
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
  PMSM_FOC_DualPlant_Controller_B.IqRef_Rate_Transition =
    PMSM_FOC_DualPlant_Controller_P.IqRef_Rate_Transition_InitialCo;

  /* InitializeConditions for RateTransition: '<Root>/AlignmentEnable_To_100us'
   *
   * Block description for '<Root>/AlignmentEnable_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.AlignmentEnable_To_100us_Buffer =
    PMSM_FOC_DualPlant_Controller_P.AlignmentEnable_To_100us_Initia;

  /* InitializeConditions for UnitDelay: '<S4>/SumA_State' */
  PMSM_FOC_DualPlant_Controlle_DW.SumA_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.SumA_State_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S4>/Count_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Count_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Count_State_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S4>/SumB_State' */
  PMSM_FOC_DualPlant_Controlle_DW.SumB_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.SumB_State_InitialCondition;

  /* InitializeConditions for Sum: '<S7>/Integrator_Add' incorporates:
   *  UnitDelay: '<S7>/Integrator_State'
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

  /* InitializeConditions for Sum: '<S16>/Integrator_Add' incorporates:
   *  UnitDelay: '<S16>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_l =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_k;

  /* InitializeConditions for RateTransition: '<Root>/StatusState_To_100us' */
  PMSM_FOC_DualPlant_Controlle_DW.StatusState_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.StatusState_To_100us_InitialCon;

  /* InitializeConditions for RateTransition: '<Root>/Overspeed_To_100us'
   *
   * Block description for '<Root>/Overspeed_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.Overspeed_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.Overspeed_To_100us_InitialCondi;

  /* InitializeConditions for RateTransition: '<Root>/Undervoltage_To_100us'
   *
   * Block description for '<Root>/Undervoltage_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.Undervoltage_To_100us_InitialCo;

  /* InitializeConditions for RateTransition: '<Root>/Overvoltage_To_100us'
   *
   * Block description for '<Root>/Overvoltage_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_To_100us_Buffer0 =
    PMSM_FOC_DualPlant_Controller_P.Overvoltage_To_100us_InitialCon;

  /* InitializeConditions for UnitDelay: '<S10>/SoftwareOvercurrent_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.SoftwareOvercurrent_Latch_State =
    PMSM_FOC_DualPlant_Controller_P.SoftwareOvercurrent_Latch_State;

  /* InitializeConditions for RateTransition: '<Root>/SupervisorPwmRequest_To_100us'
   *
   * Block description for '<Root>/SupervisorPwmRequest_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.SupervisorPwmRequest_To_100us_B =
    PMSM_FOC_DualPlant_Controller_P.SupervisorPwmRequest_To_100us_I;

  /* InitializeConditions for UnitDelay: '<S10>/EmergencyStop_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.EmergencyStop_Latch_State_DSTAT =
    PMSM_FOC_DualPlant_Controller_P.EmergencyStop_Latch_State_Initi;

  /* InitializeConditions for UnitDelay: '<S10>/DriverFault_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.DriverFault_Latch_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.DriverFault_Latch_State_Initial;

  /* InitializeConditions for UnitDelay: '<S10>/MeasurementInvalid_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.MeasurementInvalid_Latch_State_ =
    PMSM_FOC_DualPlant_Controller_P.MeasurementInvalid_Latch_State_;

  /* InitializeConditions for UnitDelay: '<S10>/Overspeed_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Overspeed_Latch_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Overspeed_Latch_State_InitialCo;

  /* InitializeConditions for UnitDelay: '<S10>/Undervoltage_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Undervoltage_Latch_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Undervoltage_Latch_State_Initia;

  /* InitializeConditions for UnitDelay: '<S10>/Overvoltage_Latch_State' */
  PMSM_FOC_DualPlant_Controlle_DW.Overvoltage_Latch_State_DSTATE =
    PMSM_FOC_DualPlant_Controller_P.Overvoltage_Latch_State_Initial;

  /* InitializeConditions for Sum: '<S19>/Integrator_Add' incorporates:
   *  UnitDelay: '<S19>/Integrator_State'
   */
  PMSM_FOC_DualPlant_Controlle_DW.Integrator_State_DSTATE_m =
    PMSM_FOC_DualPlant_Controller_P.Integrator_State_InitialCondi_c;

  /* InitializeConditions for RateTransition: '<Root>/CalibrationEnable_To_100us'
   *
   * Block description for '<Root>/CalibrationEnable_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.CalibrationEnable_To_100us_Buff =
    PMSM_FOC_DualPlant_Controller_P.CalibrationEnable_To_100us_Init;

  /* InitializeConditions for RateTransition: '<Root>/CalibrationReset_To_100us'
   *
   * Block description for '<Root>/CalibrationReset_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.CalibrationReset_To_100us_Buffe =
    PMSM_FOC_DualPlant_Controller_P.CalibrationReset_To_100us_Initi;

  /* InitializeConditions for RateTransition: '<Root>/ControllerReset_To_100us'
   *
   * Block description for '<Root>/ControllerReset_To_100us':
   *  Explicit dual-rate snapshot/hold boundary required by ARC-005.
   */
  PMSM_FOC_DualPlant_Controlle_DW.ControllerReset_To_100us_Buffer =
    PMSM_FOC_DualPlant_Controller_P.ControllerReset_To_100us_Initia;
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
