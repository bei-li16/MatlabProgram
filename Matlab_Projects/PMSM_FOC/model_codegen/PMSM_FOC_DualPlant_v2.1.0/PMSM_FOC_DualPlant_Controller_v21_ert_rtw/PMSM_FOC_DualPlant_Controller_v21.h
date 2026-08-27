/*
 * File: PMSM_FOC_DualPlant_Controller_v21.h
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

#ifndef PMSM_FOC_DualPlant_Controller_v21_h_
#define PMSM_FOC_DualPlant_Controller_v21_h_
#ifndef PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_
#define PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                  /* PMSM_FOC_DualPlant_Controller_v21_COMMON_INCLUDES_ */

#include "PMSM_FOC_DualPlant_Controller_v21_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_State_DSTATE;    /* '<S4>/Integrator_State' */
  real32_T Integrator_State_DSTATE_d;  /* '<S8>/Integrator_State' */
  real32_T Integrator_State_DSTATE_f;  /* '<S10>/Integrator_State' */
  real32_T IqRef_Rate_Transition_Buffer0;/* '<Root>/IqRef_Rate_Transition' */
} DW_PMSM_FOC_DualPlant_Control_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T SpeedReferenceRpm;          /* '<Root>/SpeedReferenceRpm' */
  real32_T SpeedRpm;                   /* '<Root>/SpeedRpm' */
  real32_T PhaseCurrentA;              /* '<Root>/PhaseCurrentA' */
  real32_T PhaseCurrentB;              /* '<Root>/PhaseCurrentB' */
  real32_T ElectricalAngleRad;         /* '<Root>/ElectricalAngleRad' */
  real32_T DcBusVoltage;               /* '<Root>/DcBusVoltage' */
} ExtU_PMSM_FOC_DualPlant_Contr_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T DutyA;                      /* '<Root>/DutyA' */
  real32_T DutyB;                      /* '<Root>/DutyB' */
  real32_T DutyC;                      /* '<Root>/DutyC' */
  real32_T IqReference;                /* '<Root>/IqReference' */
  real32_T IdMeasured;                 /* '<Root>/IdMeasured' */
  real32_T IqMeasured;                 /* '<Root>/IqMeasured' */
  real32_T VdCommand;                  /* '<Root>/VdCommand' */
  real32_T VqCommand;                  /* '<Root>/VqCommand' */
} ExtY_PMSM_FOC_DualPlant_Contr_T;

/* Parameters (default storage) */
struct P_PMSM_FOC_DualPlant_Controll_T_ {
  real32_T NATIVE_INV_SQRT3;           /* Variable: NATIVE_INV_SQRT3
                                        * Referenced by: '<S1>/InvSqrt3'
                                        */
  real32_T NATIVE_RPM_TO_RAD_S;        /* Variable: NATIVE_RPM_TO_RAD_S
                                        * Referenced by:
                                        *   '<S2>/Electrical_Speed'
                                        *   '<S10>/RpmToRad'
                                        */
  real32_T NATIVE_SQRT3_BY2;           /* Variable: NATIVE_SQRT3_BY2
                                        * Referenced by:
                                        *   '<S5>/Vb_Beta'
                                        *   '<S5>/Vc_Beta'
                                        */
  real32_T Id_Reference_Zero_Value;    /* Expression: single(0.0)
                                        * Referenced by: '<Root>/Id_Reference_Zero'
                                        */
  real32_T Ib_x2_Gain;                 /* Expression: single(2.0)
                                        * Referenced by: '<S1>/Ib_x2'
                                        */
  real32_T Integrator_State_InitialConditi;/* Expression: single(0.0)
                                            * Referenced by: '<S4>/Integrator_State'
                                            */
  real32_T Negative_Gain;              /* Expression: single(-1.0)
                                        * Referenced by: '<S7>/Negative'
                                        */
  real32_T IqRef_Rate_Transition_InitialCo;
                          /* Computed Parameter: IqRef_Rate_Transition_InitialCo
                           * Referenced by: '<Root>/IqRef_Rate_Transition'
                           */
  real32_T Integrator_State_InitialCondi_m;/* Expression: single(0.0)
                                            * Referenced by: '<S8>/Integrator_State'
                                            */
  real32_T Vb_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S5>/Vb_Alpha'
                                        */
  real32_T Vc_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S5>/Vc_Alpha'
                                        */
  real32_T Common_Mode_Gain;           /* Expression: single(-0.5)
                                        * Referenced by: '<S9>/Common_Mode'
                                        */
  real32_T Duty_Half_Value;            /* Expression: single(0.5)
                                        * Referenced by: '<S9>/Duty_Half'
                                        */
  real32_T Integrator_State_InitialCondi_g;/* Expression: single(0.0)
                                            * Referenced by: '<S10>/Integrator_State'
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
                                   *   '<S4>/Integrator_Limit'
                                   *   '<S8>/Integrator_Limit'
                                   */
extern real32_T FOC_Native_CurrentPeriod;/* Variable: FOC_Native_CurrentPeriod
                                          * Referenced by:
                                          *   '<S4>/KiTs'
                                          *   '<S8>/KiTs'
                                          */
extern real32_T FOC_Native_DutyMax;    /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S9>/Duty_A_Limit'
                                        *   '<S9>/Duty_B_Limit'
                                        *   '<S9>/Duty_C_Limit'
                                        */
extern real32_T FOC_Native_DutyMin;    /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S9>/Duty_A_Limit'
                                        *   '<S9>/Duty_B_Limit'
                                        *   '<S9>/Duty_C_Limit'
                                        */
extern real32_T FOC_Native_FluxPM;     /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S2>/Flux_PM'
                                        */
extern real32_T FOC_Native_IqLimit;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S10>/Integrator_Limit'
                                        *   '<S10>/Iq_Reference_Limit'
                                        */
extern real32_T FOC_Native_KiCurrent;  /* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S4>/KiTs'
                                        *   '<S8>/KiTs'
                                        */
extern real32_T FOC_Native_KiSpeed;    /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S10>/KiTs'
                                        */
extern real32_T FOC_Native_KpCurrent;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S4>/Kp'
                                        *   '<S8>/Kp'
                                        */
extern real32_T FOC_Native_KpSpeed;    /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S10>/Kp'
                                        */
extern real32_T FOC_Native_Ld;         /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S2>/Ld_x_Id'
                                        */
extern real32_T FOC_Native_Lq;         /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S2>/D_Decoupling'
                                        */
extern real32_T FOC_Native_PolePairs;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S2>/Electrical_Speed'
                                        */
extern real32_T FOC_Native_SpeedPeriod;/* Variable: FOC_Native_SpeedPeriod
                                        * Referenced by: '<S10>/KiTs'
                                        */
extern real32_T FOC_Native_VoltageLimit;/* Variable: FOC_Native_VoltageLimit
                                         * Referenced by:
                                         *   '<S3>/Vd_Limit'
                                         *   '<S3>/Vq_Limit'
                                         */

/* Model entry point functions */
extern void PMSM_FOC_DualPlant_Controller_v21_initialize(void);
extern void PMSM_FOC_DualPlant_Controller_v21_step(void);
extern void PMSM_FOC_DualPlant_Controller_v21_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PMSM_FOC_DualPlant_C_T *const PMSM_FOC_DualPlant_Controlle_M;

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
 * '<S1>'   : 'PMSM_FOC_DualPlant_Controller_v21/Clarke_Transform'
 * '<S2>'   : 'PMSM_FOC_DualPlant_Controller_v21/DQ_Decoupling_Feedforward'
 * '<S3>'   : 'PMSM_FOC_DualPlant_Controller_v21/DQ_Voltage_Command'
 * '<S4>'   : 'PMSM_FOC_DualPlant_Controller_v21/D_Axis_Current_PI'
 * '<S5>'   : 'PMSM_FOC_DualPlant_Controller_v21/Inverse_Clarke_Transform'
 * '<S6>'   : 'PMSM_FOC_DualPlant_Controller_v21/Inverse_Park_Transform'
 * '<S7>'   : 'PMSM_FOC_DualPlant_Controller_v21/Park_Transform'
 * '<S8>'   : 'PMSM_FOC_DualPlant_Controller_v21/Q_Axis_Current_PI'
 * '<S9>'   : 'PMSM_FOC_DualPlant_Controller_v21/SVPWM_Duty_Calculation'
 * '<S10>'  : 'PMSM_FOC_DualPlant_Controller_v21/Speed_PI_Controller_1ms'
 */
#endif                                /* PMSM_FOC_DualPlant_Controller_v21_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
