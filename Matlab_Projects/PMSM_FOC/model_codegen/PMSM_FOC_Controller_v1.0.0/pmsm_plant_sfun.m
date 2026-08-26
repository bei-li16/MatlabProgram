function pmsm_plant_sfun(block)
%PMSM_PLANT_SFUN Discrete surface-PMSM plant in the rotor dq frame.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 3;
block.NumOutputPorts = 8;
for index = 1:block.NumInputPorts
    block.InputPort(index).Dimensions = 1;
    block.InputPort(index).DatatypeID = 1; % single
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = false;
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
stateNames = {'idState','iqState','omegaMechanical','thetaElectrical'};
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
end

function outputs(block)
idState = single(block.Dwork(1).Data);
iqState = single(block.Dwork(2).Data);
omegaMechanical = single(block.Dwork(3).Data);
thetaElectrical = single(block.Dwork(4).Data);

sqrtThree = single(1.7320508075688772);
cosTheta = single(cos(double(thetaElectrical)));
sinTheta = single(sin(double(thetaElectrical)));
iAlpha = cosTheta * idState - sinTheta * iqState;
iBeta = sinTheta * idState + cosTheta * iqState;

block.OutputPort(1).Data = iAlpha;
block.OutputPort(2).Data = single(-0.5) * iAlpha + ...
    single(0.5) * sqrtThree * iBeta;
block.OutputPort(3).Data = single(-0.5) * iAlpha - ...
    single(0.5) * sqrtThree * iBeta;
block.OutputPort(4).Data = omegaMechanical * single(9.549296585513721);
block.OutputPort(5).Data = thetaElectrical;
block.OutputPort(6).Data = idState;
block.OutputPort(7).Data = iqState;
block.OutputPort(8).Data = single(1.5) * single(4.0) * ...
    single(0.05) * iqState;
end

function update(block)
vAlpha = single(block.InputPort(1).Data);
vBeta = single(block.InputPort(2).Data);
loadTorque = single(block.InputPort(3).Data);
idState = single(block.Dwork(1).Data);
iqState = single(block.Dwork(2).Data);
omegaMechanical = single(block.Dwork(3).Data);
thetaElectrical = single(block.Dwork(4).Data);

samplePeriod = single(1.0e-4);
rs = single(0.5);
ld = single(1.0e-3);
lq = single(1.0e-3);
fluxPM = single(0.05);
polePairs = single(4.0);
inertia = single(2.0e-3);
viscous = single(1.0e-4);
twoPi = single(6.283185307179586);

cosTheta = single(cos(double(thetaElectrical)));
sinTheta = single(sin(double(thetaElectrical)));
vd = cosTheta * vAlpha + sinTheta * vBeta;
vq = -sinTheta * vAlpha + cosTheta * vBeta;
omegaElectrical = polePairs * omegaMechanical;
dId = (vd - rs * idState + omegaElectrical * lq * iqState) / ld;
dIq = (vq - rs * iqState - ...
    omegaElectrical * (ld * idState + fluxPM)) / lq;
electromagneticTorque = single(1.5) * polePairs * fluxPM * iqState;
dOmega = (electromagneticTorque - loadTorque - ...
    viscous * omegaMechanical) / inertia;

idNext = idState + samplePeriod * dId;
iqNext = iqState + samplePeriod * dIq;
omegaNext = omegaMechanical + samplePeriod * dOmega;
thetaNext = thetaElectrical + samplePeriod * polePairs * omegaNext;
thetaNext = thetaNext - floor(thetaNext / twoPi) * twoPi;

block.Dwork(1).Data = min(max(idNext, single(-50)), single(50));
block.Dwork(2).Data = min(max(iqNext, single(-50)), single(50));
block.Dwork(3).Data = min(max(omegaNext, single(-1000)), single(1000));
block.Dwork(4).Data = thetaNext;
end
