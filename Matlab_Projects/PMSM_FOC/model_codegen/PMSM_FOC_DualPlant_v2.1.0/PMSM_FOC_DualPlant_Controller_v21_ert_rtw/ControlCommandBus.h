/*
 * File: ControlCommandBus.h
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

#ifndef ControlCommandBus_h_
#define ControlCommandBus_h_
#include "rtwtypes.h"

/* ControlCommandBus v2.1.0 interface contract */
typedef struct {
  /* Ts=1 ms; Class=RuntimeSignal; Level request sampled by the 1 ms command arbiter; hold for at least one slow tick */
  boolean_T StartRequest;

  /* Ts=1 ms; Class=RuntimeSignal; Stop level; has priority over StartRequest and must be asserted for at least one slow tick */
  boolean_T StopRequest;

  /* Ts=100 us; Class=RuntimeSignal; Fast emergency-stop level; directly latches fault bit 0 and inhibits PWM */
  boolean_T EmergencyStop;

  /* Ts=100 us; Class=RuntimeSignal; Fast gate-driver fault level; directly latches fault bit 1 and inhibits PWM */
  boolean_T DriverFault;

  /* Ts=100 us; Class=RuntimeSignal; External hardware permission; false independently inhibits final PWM without software substitution */
  boolean_T HardwareGate;

  /* Ts=1 ms; Class=RuntimeSignal; Requested direction: -1, 0 or +1 */
  int8_T Direction;

  /* Ts=1 ms; Class=RuntimeSignal; Mechanical speed reference */
  real32_T SpeedReferenceRpm;

  /* Ts=1 ms; Class=RuntimeSignal; Torque reference reserved for torque mode */
  real32_T TorqueReferenceNm;

  /* Ts=1 ms; Class=RuntimeSignal; Reset level accepted only while stopped, start is clear, raw faults are clear, and HardwareGate is true */
  boolean_T FaultResetRequest;
} ControlCommandBus;

#endif                                 /* ControlCommandBus_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
