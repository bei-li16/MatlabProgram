/*
 * File: PMSM_FOC_Native_Controller_v20.h
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

#ifndef PMSM_FOC_Native_Controller_v20_h_
#define PMSM_FOC_Native_Controller_v20_h_
#ifndef PMSM_FOC_Native_Controller_v20_COMMON_INCLUDES_
#define PMSM_FOC_Native_Controller_v20_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                     /* PMSM_FOC_Native_Controller_v20_COMMON_INCLUDES_ */

#include "PMSM_FOC_Native_Controller_v20_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T Iq_Reference_Limit;         /* '<S1>/Iq_Reference_Limit' */
} B_PMSM_FOC_Native_Controller__T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Speed_Integrator_State_DSTATE;/* '<S1>/Speed_Integrator_State' */
  real32_T Iq_Integrator_State_DSTATE; /* '<S1>/Iq_Integrator_State' */
  real32_T Id_Integrator_State_DSTATE; /* '<S1>/Id_Integrator_State' */
} DW_PMSM_FOC_Native_Controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T SpeedReferenceRpm;          /* '<Root>/SpeedReferenceRpm' */
  real32_T SpeedRpm;                   /* '<Root>/SpeedRpm' */
  real32_T PhaseCurrentA;              /* '<Root>/PhaseCurrentA' */
  real32_T PhaseCurrentB;              /* '<Root>/PhaseCurrentB' */
  real32_T ElectricalAngleRad;         /* '<Root>/ElectricalAngleRad' */
  real32_T DcBusVoltage;               /* '<Root>/DcBusVoltage' */
} ExtU_PMSM_FOC_Native_Controll_T;

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
} ExtY_PMSM_FOC_Native_Controll_T;

/* Parameters (default storage) */
struct P_PMSM_FOC_Native_Controller__T_ {
  real32_T NATIVE_INV_SQRT3;           /* Variable: NATIVE_INV_SQRT3
                                        * Referenced by: '<S1>/InvSqrt3'
                                        */
  real32_T NATIVE_RPM_TO_RAD_S;        /* Variable: NATIVE_RPM_TO_RAD_S
                                        * Referenced by:
                                        *   '<S1>/Electrical_Speed'
                                        *   '<S1>/Speed_RpmToRad'
                                        */
  real32_T NATIVE_SQRT3_BY2;           /* Variable: NATIVE_SQRT3_BY2
                                        * Referenced by:
                                        *   '<S1>/Vb_Beta'
                                        *   '<S1>/Vc_Beta'
                                        */
  real32_T Ib_x2_Gain;                 /* Expression: single(2.0)
                                        * Referenced by: '<S1>/Ib_x2'
                                        */
  real32_T Iq_Negative_Gain;           /* Expression: single(-1.0)
                                        * Referenced by: '<S1>/Iq_Negative'
                                        */
  real32_T Speed_Integrator_State_InitialC;/* Expression: single(0.0)
                                            * Referenced by: '<S1>/Speed_Integrator_State'
                                            */
  real32_T Iq_Integrator_State_InitialCond;/* Expression: single(0.0)
                                            * Referenced by: '<S1>/Iq_Integrator_State'
                                            */
  real32_T Id_Reference_Zero_Value;    /* Expression: single(0.0)
                                        * Referenced by: '<S1>/Id_Reference_Zero'
                                        */
  real32_T Id_Integrator_State_InitialCond;/* Expression: single(0.0)
                                            * Referenced by: '<S1>/Id_Integrator_State'
                                            */
  real32_T Vb_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S1>/Vb_Alpha'
                                        */
  real32_T Vc_Alpha_Gain;              /* Expression: single(-0.5)
                                        * Referenced by: '<S1>/Vc_Alpha'
                                        */
  real32_T Common_Mode_Gain;           /* Expression: single(-0.5)
                                        * Referenced by: '<S1>/Common_Mode'
                                        */
  real32_T Duty_Half_Value;            /* Expression: single(0.5)
                                        * Referenced by: '<S1>/Duty_Half'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_PMSM_FOC_Native_Contr_T {
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
extern P_PMSM_FOC_Native_Controller__T PMSM_FOC_Native_Controller_v2_P;

/* Block signals (default storage) */
extern B_PMSM_FOC_Native_Controller__T PMSM_FOC_Native_Controller_v2_B;

/* Block states (default storage) */
extern DW_PMSM_FOC_Native_Controller_T PMSM_FOC_Native_Controller_v_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PMSM_FOC_Native_Controll_T PMSM_FOC_Native_Controller_v2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PMSM_FOC_Native_Controll_T PMSM_FOC_Native_Controller_v2_Y;

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
                                   *   '<S1>/Id_Integrator_Limit'
                                   *   '<S1>/Iq_Integrator_Limit'
                                   */
extern real32_T FOC_Native_CurrentPeriod;/* Variable: FOC_Native_CurrentPeriod
                                          * Referenced by:
                                          *   '<S1>/Id_KiTs'
                                          *   '<S1>/Iq_KiTs'
                                          */
extern real32_T FOC_Native_DutyMax;    /* Variable: FOC_Native_DutyMax
                                        * Referenced by:
                                        *   '<S1>/Duty_A_Limit'
                                        *   '<S1>/Duty_B_Limit'
                                        *   '<S1>/Duty_C_Limit'
                                        */
extern real32_T FOC_Native_DutyMin;    /* Variable: FOC_Native_DutyMin
                                        * Referenced by:
                                        *   '<S1>/Duty_A_Limit'
                                        *   '<S1>/Duty_B_Limit'
                                        *   '<S1>/Duty_C_Limit'
                                        */
extern real32_T FOC_Native_FluxPM;     /* Variable: FOC_Native_FluxPM
                                        * Referenced by: '<S1>/Flux_PM'
                                        */
extern real32_T FOC_Native_IqLimit;    /* Variable: FOC_Native_IqLimit
                                        * Referenced by:
                                        *   '<S1>/Iq_Reference_Limit'
                                        *   '<S1>/Speed_Integrator_Limit'
                                        */
extern real32_T FOC_Native_KiCurrent;  /* Variable: FOC_Native_KiCurrent
                                        * Referenced by:
                                        *   '<S1>/Id_KiTs'
                                        *   '<S1>/Iq_KiTs'
                                        */
extern real32_T FOC_Native_KiSpeed;    /* Variable: FOC_Native_KiSpeed
                                        * Referenced by: '<S1>/Speed_KiTs'
                                        */
extern real32_T FOC_Native_KpCurrent;  /* Variable: FOC_Native_KpCurrent
                                        * Referenced by:
                                        *   '<S1>/Id_Kp'
                                        *   '<S1>/Iq_Kp'
                                        */
extern real32_T FOC_Native_KpSpeed;    /* Variable: FOC_Native_KpSpeed
                                        * Referenced by: '<S1>/Speed_Kp'
                                        */
extern real32_T FOC_Native_Ld;         /* Variable: FOC_Native_Ld
                                        * Referenced by: '<S1>/Ld_x_Id'
                                        */
extern real32_T FOC_Native_Lq;         /* Variable: FOC_Native_Lq
                                        * Referenced by: '<S1>/D_Decoupling'
                                        */
extern real32_T FOC_Native_PolePairs;  /* Variable: FOC_Native_PolePairs
                                        * Referenced by: '<S1>/Electrical_Speed'
                                        */
extern real32_T FOC_Native_SpeedPeriod;/* Variable: FOC_Native_SpeedPeriod
                                        * Referenced by: '<S1>/Speed_KiTs'
                                        */
extern real32_T FOC_Native_VoltageLimit;/* Variable: FOC_Native_VoltageLimit
                                         * Referenced by:
                                         *   '<S1>/Vd_Limit'
                                         *   '<S1>/Vq_Limit'
                                         */

/* Model entry point functions */
extern void PMSM_FOC_Native_Controller_v20_initialize(void);
extern void PMSM_FOC_Native_Controller_v20_step(void);
extern void PMSM_FOC_Native_Controller_v20_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PMSM_FOC_Native_Cont_T *const PMSM_FOC_Native_Controller_v_M;

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
 * '<Root>' : 'PMSM_FOC_Native_Controller_v20'
 * '<S1>'   : 'PMSM_FOC_Native_Controller_v20/Native_FOC_Controller_100us'
 */
#endif                                 /* PMSM_FOC_Native_Controller_v20_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
