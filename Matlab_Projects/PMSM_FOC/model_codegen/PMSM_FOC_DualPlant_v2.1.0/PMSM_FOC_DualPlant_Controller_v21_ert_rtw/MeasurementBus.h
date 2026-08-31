/*
 * File: MeasurementBus.h
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

#ifndef MeasurementBus_h_
#define MeasurementBus_h_
#include "rtwtypes.h"

/* MeasurementBus v2.1.0 interface contract */
typedef struct {
  /* Ts=100 us; Class=RuntimeSignal; Measured phase-A current */
  real32_T PhaseCurrentA;

  /* Ts=100 us; Class=RuntimeSignal; Measured phase-B current */
  real32_T PhaseCurrentB;

  /* Ts=100 us; Class=RuntimeSignal; Measured phase-C current; reserved for plausibility checks */
  real32_T PhaseCurrentC;

  /* Ts=100 us; Class=RuntimeSignal; Measured DC-bus voltage */
  real32_T DcBusVoltage;

  /* Ts=100 us; Class=RuntimeSignal; Electrical rotor angle */
  real32_T ElectricalAngleRad;

  /* Ts=1 ms; Class=RuntimeSignal; Mechanical rotor speed */
  real32_T MechanicalSpeedRpm;

  /* Ts=100 us; Class=RuntimeSignal; Aggregate measurement validity */
  boolean_T Valid;

  /* Ts=100 us; Class=RuntimeSignal; Acquisition timestamp */
  real32_T TimestampSeconds;

  /* Ts=100 us; Class=RuntimeSignal; Age since latest valid acquisition */
  uint16_T FreshnessTicks;
} MeasurementBus;

#endif                                 /* MeasurementBus_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
