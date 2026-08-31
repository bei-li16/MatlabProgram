/*
 * File: ControlCommandBus.h
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

#ifndef ControlCommandBus_h_
#define ControlCommandBus_h_
#include "rtwtypes.h"

/* ControlCommandBus v2.1.0 interface contract */
typedef struct {
  /* Ts=1 ms; Class=RuntimeSignal; Explicit start request; consumed by ARC-003 */
  boolean_T StartRequest;

  /* Ts=1 ms; Class=RuntimeSignal; Explicit stop request; consumed by ARC-003 */
  boolean_T StopRequest;

  /* Ts=100 us; Class=RuntimeSignal; Highest-priority emergency-stop request; consumed by ARC-003 */
  boolean_T EmergencyStop;

  /* Ts=1 ms; Class=RuntimeSignal; Requested direction: -1, 0 or +1 */
  int8_T Direction;

  /* Ts=1 ms; Class=RuntimeSignal; Mechanical speed reference */
  real32_T SpeedReferenceRpm;

  /* Ts=1 ms; Class=RuntimeSignal; Torque reference reserved for torque mode */
  real32_T TorqueReferenceNm;

  /* Ts=1 ms; Class=RuntimeSignal; Acknowledged fault-reset request */
  boolean_T FaultResetRequest;
} ControlCommandBus;

#endif                                 /* ControlCommandBus_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
