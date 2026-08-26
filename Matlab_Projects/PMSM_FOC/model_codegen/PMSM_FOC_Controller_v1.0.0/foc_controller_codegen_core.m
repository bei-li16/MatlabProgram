function [dutyA, dutyB, dutyC, iqReference, idMeasured, iqMeasured, vdCommand, vqCommand] = ...
    foc_controller_codegen_core(speedReferenceRpm, speedRpm, ia, ib, thetaElectrical, vdc)
%#codegen
% Code-generation implementation of the basic PMSM FOC controller.

% These states belong to the deployable controller. The generated
% initialize function resets them when the application starts.
persistent speedIntegrator dIntegrator qIntegrator iqReferenceMemory speedDivider
if isempty(speedIntegrator)
    speedIntegrator = single(0.0);
    dIntegrator = single(0.0);
    qIntegrator = single(0.0);
    iqReferenceMemory = single(0.0);
    speedDivider = uint8(9); % Execute the 1 ms speed loop on the first call.
end

vdc = max(vdc, single(1.0));
sqrtThree = single(1.7320508075688772);
inverseSqrtThree = single(0.5773502691896258);
rpmToRadians = single(0.1047197551196598);

% Clarke and Park transforms.
iAlpha = ia;
iBeta = (ia + single(2.0) * ib) * inverseSqrtThree;
cosTheta = cos(thetaElectrical);
sinTheta = sin(thetaElectrical);
idMeasured = cosTheta * iAlpha + sinTheta * iBeta;
iqMeasured = -sinTheta * iAlpha + cosTheta * iBeta;

% The controller step runs at 100 us; execute the speed PI every ten calls.
if speedDivider >= uint8(9)
    speedDivider = uint8(0);
    speedError = (speedReferenceRpm - speedRpm) * rpmToRadians;
    iqUnsaturated = FOC_KpSpeed * speedError + speedIntegrator;
    iqReference = min(max(iqUnsaturated, -FOC_IqLimit), FOC_IqLimit);
    speedIntegrator = speedIntegrator + ...
        FOC_KiSpeed * FOC_SpeedPeriod * speedError + ...
        FOC_AntiWindupGain * (iqReference - iqUnsaturated);
    speedIntegrator = min(max(speedIntegrator, -FOC_IqLimit), FOC_IqLimit);
else
    speedDivider = speedDivider + uint8(1);
    iqReference = iqReferenceMemory;
end
iqReferenceMemory = iqReference;

% Current PI, dq decoupling and back-EMF feed-forward.
omegaElectrical = speedRpm * rpmToRadians * FOC_PolePairs;
dError = -idMeasured;
qError = iqReference - iqMeasured;
vdRaw = FOC_KpCurrent * dError + dIntegrator - ...
    omegaElectrical * FOC_Lq * iqMeasured;
vqRaw = FOC_KpCurrent * qError + qIntegrator + ...
    omegaElectrical * (FOC_Ld * idMeasured + FOC_FluxPM);

voltageLimit = FOC_VoltageUtilization * vdc * inverseSqrtThree;
magnitudeSquared = vdRaw * vdRaw + vqRaw * vqRaw;
if magnitudeSquared > voltageLimit * voltageLimit
    voltageScale = voltageLimit / sqrt(magnitudeSquared);
else
    voltageScale = single(1.0);
end
vdCommand = vdRaw * voltageScale;
vqCommand = vqRaw * voltageScale;

dIntegrator = dIntegrator + FOC_KiCurrent * FOC_CurrentPeriod * dError + ...
    FOC_AntiWindupGain * (vdCommand - vdRaw);
qIntegrator = qIntegrator + FOC_KiCurrent * FOC_CurrentPeriod * qError + ...
    FOC_AntiWindupGain * (vqCommand - vqRaw);
dIntegrator = min(max(dIntegrator, -voltageLimit), voltageLimit);
qIntegrator = min(max(qIntegrator, -voltageLimit), voltageLimit);

% Inverse Park and centered SVPWM duty ratios.
vAlpha = cosTheta * vdCommand - sinTheta * vqCommand;
vBeta = sinTheta * vdCommand + cosTheta * vqCommand;
va = vAlpha;
vb = single(-0.5) * vAlpha + single(0.5) * sqrtThree * vBeta;
vc = single(-0.5) * vAlpha - single(0.5) * sqrtThree * vBeta;
maximumPhase = max(va, max(vb, vc));
minimumPhase = min(va, min(vb, vc));
commonMode = single(-0.5) * (maximumPhase + minimumPhase);
dutyA = single(0.5) + (va + commonMode) / vdc;
dutyB = single(0.5) + (vb + commonMode) / vdc;
dutyC = single(0.5) + (vc + commonMode) / vdc;
dutyA = min(max(dutyA, FOC_DutyMin), FOC_DutyMax);
dutyB = min(max(dutyB, FOC_DutyMin), FOC_DutyMax);
dutyC = min(max(dutyC, FOC_DutyMin), FOC_DutyMax);
end
