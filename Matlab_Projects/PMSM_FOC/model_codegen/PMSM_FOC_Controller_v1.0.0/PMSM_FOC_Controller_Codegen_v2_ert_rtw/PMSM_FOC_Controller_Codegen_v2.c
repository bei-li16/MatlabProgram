/*
 * File: PMSM_FOC_Controller_Codegen_v2.c
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

#include "PMSM_FOC_Controller_Codegen_v2.h"
#include <math.h>
#include "rtwtypes.h"

/* Exported block parameters */
real32_T FOC_AntiWindupGain = 0.2F;    /* Variable: FOC_AntiWindupGain
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_CurrentPeriod = 0.0001F;  /* Variable: FOC_CurrentPeriod
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_DutyMax = 0.98F;          /* Variable: FOC_DutyMax
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_DutyMin = 0.02F;          /* Variable: FOC_DutyMin
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_FluxPM = 0.05F;           /* Variable: FOC_FluxPM
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_IqLimit = 8.0F;           /* Variable: FOC_IqLimit
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_KiCurrent = 500.0F;       /* Variable: FOC_KiCurrent
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_KiSpeed = 3.0F;           /* Variable: FOC_KiSpeed
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_KpCurrent = 1.0F;         /* Variable: FOC_KpCurrent
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_KpSpeed = 0.2F;           /* Variable: FOC_KpSpeed
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_Ld = 0.001F;              /* Variable: FOC_Ld
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_Lq = 0.001F;              /* Variable: FOC_Lq
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_PolePairs = 4.0F;         /* Variable: FOC_PolePairs
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_SpeedPeriod = 0.001F;     /* Variable: FOC_SpeedPeriod
                                        * Referenced by: '<Root>/FOC_Controller_100us'
                                        */
real32_T FOC_VoltageUtilization = 0.95F;/* Variable: FOC_VoltageUtilization
                                         * Referenced by: '<Root>/FOC_Controller_100us'
                                         */

/* Block states (default storage) */
DW_PMSM_FOC_Controller_Codege_T PMSM_FOC_Controller_Codegen__DW;

/* External inputs (root inport signals with default storage) */
ExtU_PMSM_FOC_Controller_Code_T PMSM_FOC_Controller_Codegen_v_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PMSM_FOC_Controller_Code_T PMSM_FOC_Controller_Codegen_v_Y;

/* Real-time model */
static RT_MODEL_PMSM_FOC_Controller__T PMSM_FOC_Controller_Codegen__M_;
RT_MODEL_PMSM_FOC_Controller__T *const PMSM_FOC_Controller_Codegen__M =
  &PMSM_FOC_Controller_Codegen__M_;

/* Model step function */
void PMSM_FOC_Controller_Codegen_v2_step(void)
{
  real32_T cosTheta;
  real32_T iBeta;
  real32_T iqUnsaturated;
  real32_T magnitudeSquared;
  real32_T omegaElectrical;
  real32_T sinTheta;
  real32_T vdc;
  real32_T voltageLimit;

  /* MATLAB Function: '<Root>/FOC_Controller_100us' incorporates:
   *  Inport: '<Root>/DcBusVoltage'
   *  Inport: '<Root>/ElectricalAngleRad'
   *  Inport: '<Root>/PhaseCurrentA'
   *  Inport: '<Root>/PhaseCurrentB'
   *  Inport: '<Root>/SpeedReferenceRpm'
   *  Inport: '<Root>/SpeedRpm'
   */
  vdc = fmaxf(PMSM_FOC_Controller_Codegen_v_U.DcBusVoltage, 1.0F);
  iBeta = (2.0F * PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentB +
           PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentA) * 0.577350259F;
  cosTheta = cosf(PMSM_FOC_Controller_Codegen_v_U.ElectricalAngleRad);
  sinTheta = sinf(PMSM_FOC_Controller_Codegen_v_U.ElectricalAngleRad);
  PMSM_FOC_Controller_Codegen_v_Y.IdMeasured = cosTheta *
    PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentA + sinTheta * iBeta;
  PMSM_FOC_Controller_Codegen_v_Y.IqMeasured = -sinTheta *
    PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentA + cosTheta * iBeta;
  if (PMSM_FOC_Controller_Codegen__DW.speedDivider >= 9) {
    PMSM_FOC_Controller_Codegen__DW.speedDivider = 0U;
    iBeta = (PMSM_FOC_Controller_Codegen_v_U.SpeedReferenceRpm -
             PMSM_FOC_Controller_Codegen_v_U.SpeedRpm) * 0.104719758F;
    iqUnsaturated = FOC_KpSpeed * iBeta +
      PMSM_FOC_Controller_Codegen__DW.speedIntegrator;
    PMSM_FOC_Controller_Codegen__DW.iqReferenceMemory = fminf(fmaxf
      (iqUnsaturated, -FOC_IqLimit), FOC_IqLimit);
    PMSM_FOC_Controller_Codegen__DW.speedIntegrator = (FOC_KiSpeed *
      FOC_SpeedPeriod * iBeta + PMSM_FOC_Controller_Codegen__DW.speedIntegrator)
      + (PMSM_FOC_Controller_Codegen__DW.iqReferenceMemory - iqUnsaturated) *
      FOC_AntiWindupGain;
    PMSM_FOC_Controller_Codegen__DW.speedIntegrator = fminf(fmaxf
      (PMSM_FOC_Controller_Codegen__DW.speedIntegrator, -FOC_IqLimit),
      FOC_IqLimit);
  } else {
    PMSM_FOC_Controller_Codegen__DW.speedDivider++;
  }

  omegaElectrical = PMSM_FOC_Controller_Codegen_v_U.SpeedRpm * 0.104719758F *
    FOC_PolePairs;
  iBeta = PMSM_FOC_Controller_Codegen__DW.iqReferenceMemory -
    PMSM_FOC_Controller_Codegen_v_Y.IqMeasured;
  iqUnsaturated = (FOC_KpCurrent * -PMSM_FOC_Controller_Codegen_v_Y.IdMeasured +
                   PMSM_FOC_Controller_Codegen__DW.dIntegrator) -
    omegaElectrical * FOC_Lq * PMSM_FOC_Controller_Codegen_v_Y.IqMeasured;
  omegaElectrical = (FOC_Ld * PMSM_FOC_Controller_Codegen_v_Y.IdMeasured +
                     FOC_FluxPM) * omegaElectrical + (FOC_KpCurrent * iBeta +
    PMSM_FOC_Controller_Codegen__DW.qIntegrator);
  voltageLimit = FOC_VoltageUtilization * vdc * 0.577350259F;
  magnitudeSquared = iqUnsaturated * iqUnsaturated + omegaElectrical *
    omegaElectrical;
  if (magnitudeSquared > voltageLimit * voltageLimit) {
    magnitudeSquared = voltageLimit / sqrtf(magnitudeSquared);
  } else {
    magnitudeSquared = 1.0F;
  }

  PMSM_FOC_Controller_Codegen_v_Y.VdCommand = iqUnsaturated * magnitudeSquared;
  PMSM_FOC_Controller_Codegen_v_Y.VqCommand = omegaElectrical * magnitudeSquared;
  magnitudeSquared = FOC_KiCurrent * FOC_CurrentPeriod;
  PMSM_FOC_Controller_Codegen__DW.dIntegrator = (magnitudeSquared *
    -PMSM_FOC_Controller_Codegen_v_Y.IdMeasured +
    PMSM_FOC_Controller_Codegen__DW.dIntegrator) +
    (PMSM_FOC_Controller_Codegen_v_Y.VdCommand - iqUnsaturated) *
    FOC_AntiWindupGain;
  PMSM_FOC_Controller_Codegen__DW.qIntegrator = (magnitudeSquared * iBeta +
    PMSM_FOC_Controller_Codegen__DW.qIntegrator) +
    (PMSM_FOC_Controller_Codegen_v_Y.VqCommand - omegaElectrical) *
    FOC_AntiWindupGain;
  PMSM_FOC_Controller_Codegen__DW.dIntegrator = fminf(fmaxf
    (PMSM_FOC_Controller_Codegen__DW.dIntegrator, -voltageLimit), voltageLimit);
  PMSM_FOC_Controller_Codegen__DW.qIntegrator = fminf(fmaxf
    (PMSM_FOC_Controller_Codegen__DW.qIntegrator, -voltageLimit), voltageLimit);
  iBeta = cosTheta * PMSM_FOC_Controller_Codegen_v_Y.VdCommand - sinTheta *
    PMSM_FOC_Controller_Codegen_v_Y.VqCommand;
  sinTheta = sinTheta * PMSM_FOC_Controller_Codegen_v_Y.VdCommand + cosTheta *
    PMSM_FOC_Controller_Codegen_v_Y.VqCommand;
  cosTheta = -0.5F * iBeta + 0.866025388F * sinTheta;
  sinTheta = -0.5F * iBeta - 0.866025388F * sinTheta;
  iqUnsaturated = (fmaxf(iBeta, fmaxf(cosTheta, sinTheta)) + fminf(iBeta, fminf
    (cosTheta, sinTheta))) * -0.5F;

  /* Outport: '<Root>/DutyA' incorporates:
   *  MATLAB Function: '<Root>/FOC_Controller_100us'
   */
  PMSM_FOC_Controller_Codegen_v_Y.DutyA = fminf(fmaxf((iBeta + iqUnsaturated) /
    vdc + 0.5F, FOC_DutyMin), FOC_DutyMax);

  /* Outport: '<Root>/DutyB' incorporates:
   *  MATLAB Function: '<Root>/FOC_Controller_100us'
   */
  PMSM_FOC_Controller_Codegen_v_Y.DutyB = fminf(fmaxf((cosTheta + iqUnsaturated)
    / vdc + 0.5F, FOC_DutyMin), FOC_DutyMax);

  /* Outport: '<Root>/DutyC' incorporates:
   *  MATLAB Function: '<Root>/FOC_Controller_100us'
   */
  PMSM_FOC_Controller_Codegen_v_Y.DutyC = fminf(fmaxf((sinTheta + iqUnsaturated)
    / vdc + 0.5F, FOC_DutyMin), FOC_DutyMax);

  /* Outport: '<Root>/IqReference' incorporates:
   *  MATLAB Function: '<Root>/FOC_Controller_100us'
   */
  PMSM_FOC_Controller_Codegen_v_Y.IqReference =
    PMSM_FOC_Controller_Codegen__DW.iqReferenceMemory;
}

/* Model initialize function */
void PMSM_FOC_Controller_Codegen_v2_initialize(void)
{
  /* SystemInitialize for MATLAB Function: '<Root>/FOC_Controller_100us' */
  PMSM_FOC_Controller_Codegen__DW.speedDivider = 9U;
}

/* Model terminate function */
void PMSM_FOC_Controller_Codegen_v2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
