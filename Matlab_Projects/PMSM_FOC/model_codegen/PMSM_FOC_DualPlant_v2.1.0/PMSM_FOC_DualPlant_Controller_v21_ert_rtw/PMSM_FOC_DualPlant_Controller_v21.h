/*
 * File: PMSM_FOC_DualPlant_Controller_v21.h
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

#ifndef PMSM_FOC_DualPlant_Controller_v21_h_
#define PMSM_FOC_DualPlant_Controller_v21_h_
#ifndef PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_
#define PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                  /* PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_ */

#include "PMSM_FOC_DualPlant_Controller_v21_types.h"
#include "ControlCommandBus.h"
#include "MeasurementBus.h"
#include "ControlStatusBus.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T IqReference;                /* '<Root>/IqRef_Rate_Transition' */
  boolean_T StatusFault_To_100us;      /* '<Root>/StatusFault_To_100us' */
  boolean_T ControlEnable;             /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T PwmEnable;                 /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T CalibrationEnable;         /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T CalibrationReset;          /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T AlignmentEnable;           /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T ControllerReset;           /* '<Root>/Motor_Supervisor_1ms' */
} B_PMSM_FOC_DualPlant_Controll_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Count_State_DSTATE;         /* '<S3>/Count_State' */
  real32_T SumA_State_DSTATE;          /* '<S3>/SumA_State' */
  real32_T SumB_State_DSTATE;          /* '<S3>/SumB_State' */
  real32_T Integrator_State_DSTATE;    /* '<S6>/Integrator_State' */
  real32_T Integrator_State_DSTATE_c;  /* '<S13>/Integrator_State' */
  real32_T Integrator_State_DSTATE_k;  /* '<S16>/Integrator_State' */
  real32_T IqRef_Rate_Transition_Buffer0;/* '<Root>/IqRef_Rate_Transition' */
  uint16_T temporalCounter_i1;         /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T Fault_Latch_State_DSTATE;  /* '<S8>/Fault_Latch_State' */
  uint8_T StatusState_To_100us_Buffer0;/* '<Root>/StatusState_To_100us' */
  uint8_T is_active_c3_PMSM_FOC_DualPlant;/* '<Root>/Motor_Supervisor_1ms' */
  uint8_T is_c3_PMSM_FOC_DualPlant_Contro;/* '<Root>/Motor_Supervisor_1ms' */
  uint8_T is_SUPERVISED;               /* '<Root>/Motor_Supervisor_1ms' */
  boolean_T StatusFault_To_100us_Buffer0;/* '<Root>/StatusFault_To_100us' */
} DW_PMSM_FOC_DualPlant_Control_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  ControlCommandBus ControlCommand;    /* '<Root>/ControlCommand' */
  MeasurementBus Measurement;          /* '<Root>/Measurement' */
} ExtU_PMSM_FOC_DualPlant_Contr_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  ControlStatusBus ControlStatus;      /* '<Root>/ControlStatus' */
} ExtY_PMSM_FOC_DualPlant_Contr_T;

/* Parameters (default storage) */
struct P_PMSM_FOC_DualPlant_Controll_T_ {
  real32_T FOC_Native_CurrentPeriod;   /* Variable: FOC_Native_CurrentPeriod
                                        * Referenced by:
                                        *   '<S6>/KiTs'
                                        *   '<S13>/KiTs'
                                        */
  real32_T FOC_Native_SpeedPeriod;     /* Variable: FOC_Native_SpeedPeriod
                                        * Referenced by: '<S16>/KiTs'
                                        */
  real32_T NATIVE_INV_SQRT3;           /* Variable: NATIVE_INV_SQRT3
                                        * Referenced by: '<S2>/InvSqrt3'
                                        */
  real32_T NATIVE_RPM_TO_RAD_S;        /* Variable: NATIVE_RPM_TO_RAD_S
                                        * Referenced by:
                                        *   '<S4>/Electrical_Speed'
                                        *   '<S16>/RpmToRad'
                                        */
  real32_T NATIVE_SQRT3_BY2;           /* Variable: NATIVE_SQRT3_BY2
                                        * Referenced by:
                                        *   '<S9>/Vb_Beta'
                                        *   '<S9>/Vc_Beta'
                                        */
  real32_T PMSM_Alignment_Cos;         /* Variable: PMSM_Alignment_Cos
                                        * Referenced by: '<S1>/Align_Cos'
                                        */
  real32_T PMSM_Alignment_Sin;         /* Variable: PMSM_Alignment_Sin
                                        * Referenced by: '<S1>/Align_Sin'
                                        */
  real32_T PMSM_SafeDuty;              /* Variable: PMSM_SafeDuty
                                        * Referenced by: '<Root>/Safe_Duty_50pct'
                                        */
  real32_T One_Value;                  /* Expression: single(1.0)
                                        * Referenced by: '<S3>/One'
                                        */
  real32_T Integrator_Zero_Value;      /* Expression: single(0.0)
                                        * Referenced by: '<S6>/Integrator_Zero'
                                        */
  real32_T Integrator_Zero_Value_j;    /* Expression: single(0.0)
                                        * Referenced by: '<S13>/Integrator_Zero'
                                        */
  real32_T Integrator_Zero_Value_c;    /* Expression: single(0.0)
                                        * Referenced by: '<S16>/Integrator_Zero'
                                        */
  real32_T Count_State_InitialCondition;/* Expression: single(0.0)
                                         * Referenced by: '<S3>/Count_State'
                                         */
  real32_T Id_Reference_Zero_Value;    /* Expression: single(0.0)
                                        * Referenced by: '<Root>/Id_Reference_Zero'
                                        */
  real32_T SumA_State_InitialCondition;/* Expression: single(0.0)
                                        * Referenced by: '<S3>/SumA_State'
                                        */
  real32_T Zero_Value;                 /* Expression: single(0.0)
                                        * Referenced by: '<S3>/Zero'
                                        */
  real32_T SumB_State_InitialCondition;/* Expression: single(0.0)
                                        * Referenced by: '<S3>/SumB_State'
                                        */
  real32_T Ib_x2_Gain;                 /* Expression: single(2.0)
                                        * Referenced by: '<S2>/Ib_x2'
                                        */
  real32_T Integrator_State_InitialConditi;/* Expression: single(0.0)
                                            * Referenced by: '<S6>/Integrator_State'
                                            */
  real32_T Negative_Gain;              /* Expression: single(-1.0)
                                        * Referenced by: '<S12>/Negative'
                                        */
  real32_T IqRef_Rate_Transition_InitialCo;
                          /* Computed Parameter: IqRef_Rate_Transition_InitialCo
                           * Referenced by: '<Root>/IqRef_Rate_Transition'
                           */
  real32_T Integrator_State_InitialCondi_e;/* Expression: single(0.0)
                                            * Referenced by: '<S13>/Integrator_State'
                                            */
  real32_T Vb_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S9>/Vb_Alpha'
                                        */
  real32_T Vc_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S9>/Vc_Alpha'
                                        */
  real32_T Common_Mode_Gain;           /* Expression: single(-0.5)
                                        * Referenced by: '<S14>/Common_Mode'
                                        */
  real32_T Duty_Half_Value;            /* Expression: single(0.5)
                                        * Referenced by: '<S14>/Duty_Half'
                                        */
  real32_T Integrator_State_InitialCondi_a;/* Expression: single(0.0)
                                            * Referenced by: '<S16>/Integrator_State'
                                            */
  boolean_T Fault_Latch_State_InitialCondit;/* Expression: false
                                             * Referenced by: '<S8>/Fault_Latch_State'
                                             */
  boolean_T StatusFault_To_100us_InitialCon;
                          /* Computed Parameter: StatusFault_To_100us_InitialCon
                           * Referenced by: '<Root>/StatusFault_To_100us'
                           */
  boolean_T VoltageLimit_Inactive_Value;/* Expression: false
                                         * Referenced by: '<Root>/VoltageLimit_Inactive'
                                         */
  uint8_T StatusState_To_100us_InitialCon;
                          /* Computed Parameter: StatusState_To_100us_InitialCon
                           * Referenced by: '<Root>/StatusState_To_100us'
                           */
};

/* Real-time Model Data Structure */
struct tag_RTM_PMSM_FOC_DualPlant_Co_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block parameters (default storage) */
extern P_PMSM_FOC_DualPlant_Controll_T PMSM_FOC_DualPlant_Controller_P;

/* Block signals (default storage) */
extern B_PMSM_FOC_DualPlant_Controll_T PMSM_FOC_DualPlant_Controller_B;

/* Block states (default storage) */
extern DW_PMSM_FOC_DualPlant_Control_T PMSM_FOC_DualPlant_Controlle_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PMSM_FOC_DualPlant_Contr_T PMSM_FOC_DualPlant_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PMSM_FOC_DualPlant_Contr_T PMSM_FOC_DualPlant_Controller_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T FOC_Native_CurrentIntegratorLimit;
                                  /* Variable: FOC_Native_CurrentIntegratorLimit
                                   * Referenced by:
                                   *   '<S6>/Integrator_Limit'
                                   *   '<S13>/Integrator_Limit'
                                   * D/q PI integrator absolute limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                   */
extern real32_T FOC_Native_DutyMax;    /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        * Maximum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_DutyMin;    /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S14>/Duty_A_Limit'
                                        *   '<S14>/Duty_B_Limit'
                                        *   '<S14>/Duty_C_Limit'
                                        * Minimum SVPWM duty Owner=Modulation; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_FluxPM;     /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S4>/Flux_PM'
                                        * Permanent-magnet flux linkage Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_IqLimit;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S16>/Integrator_Limit'
                                        *   '<S16>/Iq_Reference_Limit'
                                        * Speed-loop q-axis current command limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_KiCurrent;  /* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S6>/KiTs'
                                        *   '<S13>/KiTs'
                                        * Current PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_KiSpeed;    /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S16>/KiTs'
                                        * Speed PI integral gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_KpCurrent;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S6>/Kp'
                                        *   '<S13>/Kp'
                                        * Current PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_KpSpeed;    /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S16>/Kp'
                                        * Speed PI proportional gain Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_Ld;         /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S4>/Ld_x_Id'
                                        * D-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_Lq;         /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S4>/D_Decoupling'
                                        * Q-axis inductance Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_PolePairs;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S4>/Electrical_Speed'
                                        * Integer-valued motor pole-pair count stored as single for plant equation compatibility Owner=MotorCalibration; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T FOC_Native_VoltageLimit;/* Variable: FOC_Native_VoltageLimit
                                         * Referenced by:
                                         *   '<S5>/Vd_Limit'
                                         *   '<S5>/Vq_Limit'
                                         * Independent d/q voltage command limit Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                         */
extern real32_T PMSM_Alignment_Vd_V;   /* Variable: PMSM_Alignment_Vd_V
                                        * Referenced by: '<S1>/Align_Vd'
                                        * Alignment d-axis voltage Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T PMSM_Alignment_Vq_V;   /* Variable: PMSM_Alignment_Vq_V
                                        * Referenced by: '<S1>/Align_Vq'
                                        * Alignment q-axis voltage Owner=Control; Version=2.1.0; Class=TunableCalibration.
                                        */
extern real32_T PMSM_Protection_MaxCurrent_A;/* Variable: PMSM_Protection_MaxCurrent_A
                                              * Referenced by: '<S8>/Max_Current_A'
                                              * Fast overcurrent trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                              */
extern real32_T PMSM_Protection_MaxDcBus_V;/* Variable: PMSM_Protection_MaxDcBus_V
                                            * Referenced by: '<S15>/Max_Vdc'
                                            * DC-bus overvoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                            */
extern real32_T PMSM_Protection_MaxSpeed_Rpm;/* Variable: PMSM_Protection_MaxSpeed_Rpm
                                              * Referenced by: '<S15>/Max_Speed_Rpm'
                                              * Slow overspeed trip threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                              */
extern real32_T PMSM_Protection_MinDcBus_V;/* Variable: PMSM_Protection_MinDcBus_V
                                            * Referenced by: '<S15>/Min_Vdc'
                                            * DC-bus undervoltage threshold Owner=Safety; Version=2.1.0; Class=TunableCalibration.
                                            */
extern real32_T PMSM_StartThreshold_Rpm;/* Variable: PMSM_StartThreshold_Rpm
                                         * Referenced by: '<Root>/Start_Threshold_Rpm'
                                         * Legacy implicit-start threshold pending ARC-003 Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                         */
extern uint16_T PMSM_Alignment_DurationTicks;/* Variable: PMSM_Alignment_DurationTicks
                                              * Referenced by: '<Root>/Motor_Supervisor_1ms'
                                              * Alignment duration in 1 ms supervisor ticks Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                              */
extern uint16_T PMSM_Calibration_SampleCount;/* Variable: PMSM_Calibration_SampleCount
                                              * Referenced by: '<S3>/Sample_Target'
                                              * Current-offset averaging sample count Owner=Measurement; Version=2.1.0; Class=TunableCalibration.
                                              */
extern uint16_T PMSM_Calibration_TimeoutTicks;
                                      /* Variable: PMSM_Calibration_TimeoutTicks
                                       * Referenced by: '<Root>/Motor_Supervisor_1ms'
                                       * Calibration timeout in 1 ms supervisor ticks Owner=StateManager; Version=2.1.0; Class=TunableCalibration.
                                       */

/* Model entry point functions */
extern void PMSM_FOC_DualPlant_Controller_v21_initialize(void);
extern void PMSM_FOC_DualPlant_Controller_v21_step(void);
extern void PMSM_FOC_DualPlant_Controller_v21_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PMSM_FOC_DualPlant_C_T *const PMSM_FOC_DualPlant_Controlle_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/AlignmentEnable_To_100us' : Eliminate redundant data type conversion
 * Block '<Root>/CalibrationEnable_To_100us' : Eliminate redundant data type conversion
 * Block '<Root>/CalibrationReset_To_100us' : Eliminate redundant data type conversion
 * Block '<Root>/ControllerReset_To_100us' : Eliminate redundant data type conversion
 * Block '<Root>/MeasurementValid_Echo' : Eliminate redundant data type conversion
 * Block '<Root>/SupervisorPwmEnable_To_100us' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PMSM_FOC_DualPlant_Controller_v21'
 * '<S1>'   : 'PMSM_FOC_DualPlant_Controller_v21/Alignment_DQ_Override_100us'
 * '<S2>'   : 'PMSM_FOC_DualPlant_Controller_v21/Clarke_Transform'
 * '<S3>'   : 'PMSM_FOC_DualPlant_Controller_v21/Current_Offset_Calibration_100us'
 * '<S4>'   : 'PMSM_FOC_DualPlant_Controller_v21/DQ_Decoupling_Feedforward'
 * '<S5>'   : 'PMSM_FOC_DualPlant_Controller_v21/DQ_Voltage_Command'
 * '<S6>'   : 'PMSM_FOC_DualPlant_Controller_v21/D_Axis_Current_PI'
 * '<S7>'   : 'PMSM_FOC_DualPlant_Controller_v21/Electrical_Angle_Trig_100us'
 * '<S8>'   : 'PMSM_FOC_DualPlant_Controller_v21/Fast_Safety_Gate_100us'
 * '<S9>'   : 'PMSM_FOC_DualPlant_Controller_v21/Inverse_Clarke_Transform'
 * '<S10>'  : 'PMSM_FOC_DualPlant_Controller_v21/Inverse_Park_Transform'
 * '<S11>'  : 'PMSM_FOC_DualPlant_Controller_v21/Motor_Supervisor_1ms'
 * '<S12>'  : 'PMSM_FOC_DualPlant_Controller_v21/Park_Transform'
 * '<S13>'  : 'PMSM_FOC_DualPlant_Controller_v21/Q_Axis_Current_PI'
 * '<S14>'  : 'PMSM_FOC_DualPlant_Controller_v21/SVPWM_Duty_Calculation'
 * '<S15>'  : 'PMSM_FOC_DualPlant_Controller_v21/Slow_Safety_Monitor_1ms'
 * '<S16>'  : 'PMSM_FOC_DualPlant_Controller_v21/Speed_PI_Controller_1ms'
 */
#endif                                /* PMSM_FOC_DualPlant_Controller_v21_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
