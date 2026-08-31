/*
 * File: ControlStatusBus.h
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

#ifndef ControlStatusBus_h_
#define ControlStatusBus_h_
#include "rtwtypes.h"

/* ControlStatusBus v2.1.0 interface contract */
typedef struct {
  /* Ts=100 us; Class=ReadOnlyDiagnostic; Raw SVPWM phase-A duty request; actuator must also honor PwmEnable */
  real32_T DutyA;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Raw SVPWM phase-B duty request; actuator must also honor PwmEnable */
  real32_T DutyB;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Raw SVPWM phase-C duty request; actuator must also honor PwmEnable */
  real32_T DutyC;

  /* Ts=1 ms; Class=ReadOnlyDiagnostic; Q-axis current reference */
  real32_T IqReference;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Measured d-axis current */
  real32_T IdMeasured;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Measured q-axis current */
  real32_T IqMeasured;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Applied d-axis voltage command */
  real32_T VdCommand;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Applied q-axis voltage command */
  real32_T VqCommand;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Single final permission: SupervisorPwmRequest AND FastInterlock AND HardwareGate */
  boolean_T PwmEnable;

  /* Ts=1 ms; Class=ReadOnlyDiagnostic; Supervisor state code */
  uint8_T MotorStateCode;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Latched fault bitmap: bits 0..6 = EStop, driver, overcurrent, invalid measurement, overspeed, undervoltage, overvoltage */
  uint32_T FaultBits;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Primary latched fault code 1..7 in bitmap priority order; zero means no fault */
  uint16_T FaultCode;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Latched software-overcurrent status (fault bit 2) */
  boolean_T CurrentLimitActive;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Voltage saturation status; detailed detection belongs to CTL-001 */
  boolean_T VoltageLimitActive;

  /* Ts=100 us; Class=ReadOnlyDiagnostic; Echo of aggregate measurement validity */
  boolean_T MeasurementValid;
} ControlStatusBus;

#endif                                 /* ControlStatusBus_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
