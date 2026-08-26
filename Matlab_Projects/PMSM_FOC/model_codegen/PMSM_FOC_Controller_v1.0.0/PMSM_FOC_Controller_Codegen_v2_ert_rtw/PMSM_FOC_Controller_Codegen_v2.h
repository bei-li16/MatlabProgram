/*
 * File: PMSM_FOC_Controller_Codegen_v2.h
 *
 * Code generated for Simulink model 'PMSM_FOC_Controller_Codegen_v2'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Wed Aug 26 23:29:48 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef PMSM_FOC_Controller_Codegen_v2_h_
#define PMSM_FOC_Controller_Codegen_v2_h_
#ifndef PMSM_FOC_Controller_Codegen_v2_COMMON_INCLUDES_
#define PMSM_FOC_Controller_Codegen_v2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                     /* PMSM_FOC_Controller_Codegen_v2_COMMON_INCLUDES_ */

#include "PMSM_FOC_Controller_Codegen_v2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T speedIntegrator;            /* '<Root>/FOC_Controller_100us' */
  real32_T dIntegrator;                /* '<Root>/FOC_Controller_100us' */
  real32_T qIntegrator;                /* '<Root>/FOC_Controller_100us' */
  real32_T iqReferenceMemory;          /* '<Root>/FOC_Controller_100us' */
  uint8_T speedDivider;                /* '<Root>/FOC_Controller_100us' */
} DW_PMSM_FOC_Controller_Codege_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T SpeedReferenceRpm;          /* '<Root>/SpeedReferenceRpm' */
  real32_T SpeedRpm;                   /* '<Root>/SpeedRpm' */
  real32_T PhaseCurrentA;              /* '<Root>/PhaseCurrentA' */
  real32_T PhaseCurrentB;              /* '<Root>/PhaseCurrentB' */
  real32_T ElectricalAngleRad;         /* '<Root>/ElectricalAngleRad' */
  real32_T DcBusVoltage;               /* '<Root>/DcBusVoltage' */
} ExtU_PMSM_FOC_Controller_Code_T;

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
} ExtY_PMSM_FOC_Controller_Code_T;

/* Real-time Model Data Structure */
struct tag_RTM_PMSM_FOC_Controller_C_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PMSM_FOC_Controller_Codege_T PMSM_FOC_Controller_Codegen__DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PMSM_FOC_Controller_Code_T PMSM_FOC_Controller_Codegen_v_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PMSM_FOC_Controller_Code_T PMSM_FOC_Controller_Codegen_v_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T FOC_AntiWindupGain;    /* Variable: FOC_AntiWindupGain
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_CurrentPeriod;     /* Variable: FOC_CurrentPeriod
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_DutyMax;           /* Variable: FOC_DutyMax
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_DutyMin;           /* Variable: FOC_DutyMin
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_FluxPM;            /* Variable: FOC_FluxPM
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_IqLimit;           /* Variable: FOC_IqLimit
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_KiCurrent;         /* Variable: FOC_KiCurrent
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_KiSpeed;           /* Variable: FOC_KiSpeed
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_KpCurrent;         /* Variable: FOC_KpCurrent
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_KpSpeed;           /* Variable: FOC_KpSpeed
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_Ld;                /* Variable: FOC_Ld
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_Lq;                /* Variable: FOC_Lq
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_PolePairs;         /* Variable: FOC_PolePairs
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_SpeedPeriod;       /* Variable: FOC_SpeedPeriod
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
extern real32_T FOC_VoltageUtilization;/* Variable: FOC_VoltageUtilization
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */

/* Model entry point functions */
extern void PMSM_FOC_Controller_Codegen_v2_initialize(void);
extern void PMSM_FOC_Controller_Codegen_v2_step(void);
extern void PMSM_FOC_Controller_Codegen_v2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PMSM_FOC_Controller__T *const PMSM_FOC_Controller_Codegen__M;

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
 * '<Root>' : 'PMSM_FOC_Controller_Codegen_v2'
 * '<S1>'   : 'PMSM_FOC_Controller_Codegen_v2/FOC_Controller_100us'
 */
#endif                                 /* PMSM_FOC_Controller_Codegen_v2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
