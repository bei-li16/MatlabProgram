function foc_controller_sfun(block)
%FOC_CONTROLLER_SFUN Discrete speed and dq-current controller for basic FOC.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 6;
block.NumOutputPorts = 8;

for index = 1:block.NumInputPorts
    block.InputPort(index).Dimensions = 1;
    block.InputPort(index).DatatypeID = 1; % single
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end
for index = 1:block.NumOutputPorts
    block.OutputPort(index).Dimensions = 1;
    block.OutputPort(index).DatatypeID = 1; % single
    block.OutputPort(index).Complexity = 'Real';
end

block.SampleTimes = [1.0e-4 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @postPropagationSetup);
block.RegBlockMethod('InitializeConditions', @initializeConditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @update);
end

function postPropagationSetup(block)
stateNames = {'speedIntegrator','dIntegrator','qIntegrator', ...
    'iqRefMemory','speedDivider'};
block.NumDworks = numel(stateNames);
for index = 1:numel(stateNames)
    block.Dwork(index).Name = stateNames{index};
    block.Dwork(index).Dimensions = 1;
    block.Dwork(index).DatatypeID = 1; % single
    block.Dwork(index).Complexity = 'Real';
    block.Dwork(index).UsedAsDiscState = true;
end
end

function initializeConditions(block)
for index = 1:block.NumDworks
    block.Dwork(index).Data = single(0);
end
block.Dwork(5).Data = single(9); % Run the 1 ms loop on the first call.
end

function outputs(block)
[outputValues, ~] = calculateController(block);
for index = 1:block.NumOutputPorts
    block.OutputPort(index).Data = outputValues(index);
end
end

function update(block)
[~, nextStates] = calculateController(block);
for index = 1:block.NumDworks
    block.Dwork(index).Data = nextStates(index);
end
end

function [outputValues, nextStates] = calculateController(block)
speedReferenceRpm = single(block.InputPort(1).Data);
speedRpm = single(block.InputPort(2).Data);
ia = single(block.InputPort(3).Data);
ib = single(block.InputPort(4).Data);
thetaElectrical = single(block.InputPort(5).Data);
vdc = max(single(block.InputPort(6).Data), single(1.0));

speedIntegrator = single(block.Dwork(1).Data);
dIntegrator = single(block.Dwork(2).Data);
qIntegrator = single(block.Dwork(3).Data);
iqReferenceMemory = single(block.Dwork(4).Data);
speedDivider = single(block.Dwork(5).Data);

currentPeriod = single(1.0e-4);
speedPeriod = single(1.0e-3);
polePairs = single(4.0);
ld = single(1.0e-3);
lq = single(1.0e-3);
fluxPM = single(0.05);
kpCurrent = single(1.0);
kiCurrent = single(500.0);
kpSpeed = single(0.20);
kiSpeed = single(3.0);
iqLimit = single(8.0);
sqrtThree = single(1.7320508075688772);
inverseSqrtThree = single(0.5773502691896258);
rpmToRadians = single(0.1047197551196598);

% Clarke and Park transforms.
iAlpha = ia;
iBeta = (ia + single(2.0) * ib) * inverseSqrtThree;
cosTheta = single(cos(double(thetaElectrical)));
sinTheta = single(sin(double(thetaElectrical)));
idMeasured = cosTheta * iAlpha + sinTheta * iBeta;
iqMeasured = -sinTheta * iAlpha + cosTheta * iBeta;

% Execute the speed PI once every ten 100 us calls.
if speedDivider >= single(9.0)
    speedDividerNext = single(0);
    speedError = (speedReferenceRpm - speedRpm) * rpmToRadians;
    iqUnsaturated = kpSpeed * speedError + speedIntegrator;
    iqReference = min(max(iqUnsaturated, -iqLimit), iqLimit);
    speedIntegratorNext = speedIntegrator + ...
        kiSpeed * speedPeriod * speedError + ...
        single(0.2) * (iqReference - iqUnsaturated);
    speedIntegratorNext = min(max(speedIntegratorNext, -iqLimit), iqLimit);
else
    speedDividerNext = speedDivider + single(1.0);
    iqReference = iqReferenceMemory;
    speedIntegratorNext = speedIntegrator;
end

% Current PI, dq decoupling, and back-EMF feed-forward.
omegaElectrical = speedRpm * rpmToRadians * polePairs;
dError = -idMeasured;
qError = iqReference - iqMeasured;
vdRaw = kpCurrent * dError + dIntegrator - omegaElectrical * lq * iqMeasured;
vqRaw = kpCurrent * qError + qIntegrator + ...
    omegaElectrical * (ld * idMeasured + fluxPM);

voltageLimit = single(0.95) * vdc * inverseSqrtThree;
magnitudeSquared = vdRaw * vdRaw + vqRaw * vqRaw;
if magnitudeSquared > voltageLimit * voltageLimit
    voltageScale = voltageLimit / single(sqrt(double(magnitudeSquared)));
else
    voltageScale = single(1.0);
end
vdCommand = vdRaw * voltageScale;
vqCommand = vqRaw * voltageScale;

dIntegratorNext = dIntegrator + kiCurrent * currentPeriod * dError + ...
    single(0.2) * (vdCommand - vdRaw);
qIntegratorNext = qIntegrator + kiCurrent * currentPeriod * qError + ...
    single(0.2) * (vqCommand - vqRaw);
dIntegratorNext = min(max(dIntegratorNext, -voltageLimit), voltageLimit);
qIntegratorNext = min(max(qIntegratorNext, -voltageLimit), voltageLimit);

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
dutyA = min(max(dutyA, single(0.02)), single(0.98));
dutyB = min(max(dutyB, single(0.02)), single(0.98));
dutyC = min(max(dutyC, single(0.02)), single(0.98));

outputValues = single([dutyA, dutyB, dutyC, iqReference, ...
    idMeasured, iqMeasured, vdCommand, vqCommand]);
nextStates = single([speedIntegratorNext, dIntegratorNext, ...
    qIntegratorNext, iqReference, speedDividerNext]);
end
