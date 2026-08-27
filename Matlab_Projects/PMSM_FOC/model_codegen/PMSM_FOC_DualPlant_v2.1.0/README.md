# PMSM FOC Dual Plant v2.1.0

This package adds a selectable motor plant to the native Simulink FOC model.
The original `PMSM_FOC_Native_v2.0.0` package is not modified.

## Models

- `PMSM_FOC_DualPlant_ClosedLoop_v21.slx`: closed-loop simulation harness.
- `PMSM_FOC_DualPlant_Controller_v21.slx`: controller-only ERT code-generation model.
- `Selectable_PMSM_Plant`: Variant Subsystem with a common 3-input/7-output interface.

Plant choices:

1. `Native_Discrete_PMSM`: the retained Simulink-block implementation.
2. `MathWorks_MCB_PMSM_HDL`: wrapper around the official
   `mcbhdlplantlib/PMSM HDL` Motor Control Blockset block.

The default selection is 2. Both choices use the same electrical/mechanical
parameters and a 100 us discrete sample time.

## Switching the plant

The model workspace variable `PMSM_PLANT_SELECTION` controls the Variant:

- `1`: native discrete PMSM
- `2`: MathWorks Motor Control Blockset PMSM HDL

Use the `ONE-CLICK PMSM PLANT SWITCH` hyperlink in the model to toggle the
selection while simulation is stopped. The adjacent Dashboard Callback Button
invokes the same script. In MATLAB R2024a a Dashboard button on the canvas must
first be selected to activate it, so the hyperlink is the true one-click path.
The selected choice is compiled when the next simulation starts.

## Common plant interface

Inputs:

- `VAlpha`, `VBeta`: stator voltage command in the stationary alpha-beta frame
- `LoadTorque`: load torque in N*m

Outputs:

- `SpeedRpm`
- `ThetaElectrical`
- `Ia`, `Ib`
- `TorqueNm`
- `Id`, `Iq`

The MathWorks wrapper performs alpha-beta to three-phase voltage conversion and
converts the official plant outputs back to the native interface.

## Verification and generated code

Run `build_pmsm_foc_dualplant_v21` to rebuild the models, simulate both plant
choices, generate the comparison plot, and rebuild controller C code.

Latest verification:

- Native final speed: 993.090393 rpm, simulation PASS
- MathWorks plant final speed: 993.090332 rpm, simulation PASS
- Variant choices detected: 2
- Official block reference verified: PASS
- Controller ERT code interfaces and no-S-Function check: PASS

Artifacts:

- `verification_report.txt`
- `PMSM_FOC_DualPlant_v21_results.png`
- `PMSM_FOC_DualPlant_Controller_v21_ert_rtw/`
- `PMSM_FOC_DualPlant_Controller_v21.exe`

The generated C code is for the controller model. The two motor plants are the
simulation environment and are intentionally not part of the controller target.

Required products include Simulink, Motor Control Blockset, Simulink Coder, and
Embedded Coder. The package was built with MATLAB R2024a.
