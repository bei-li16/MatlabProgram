function verify_pmsm_foc_controller_sil
%VERIFY_PMSM_FOC_CONTROLLER_SIL Compare normal and generated-C execution.

workDirectory = fileparts(mfilename('fullpath'));
oldDirectory = pwd;
cleanupDirectory = onCleanup(@() cd(oldDirectory));
cd(workDirectory);

modelName = 'PMSM_FOC_Controller_Codegen_v2';
load_system(modelName);

samplePeriod = 1.0e-4;
stopTime = 0.03;
time = (0:samplePeriod:stopTime)';
speedReference = single(1000.0 * (time >= 0.002));
speedFeedback = single(900.0 * min(time / 0.025, 1.0));
electricalFrequency = 200.0;
electricalAngle = single(mod(2.0*pi*electricalFrequency*time, 2.0*pi));
phaseCurrentA = single(2.5*sin(double(electricalAngle)));
phaseCurrentB = single(2.5*sin(double(electricalAngle) - 2.0*pi/3.0));
dcBusVoltage = single(48.0 * ones(size(time)));

inputDataset = Simulink.SimulationData.Dataset;
inputDataset = inputDataset.addElement(timeseries(speedReference, time), 'SpeedReferenceRpm');
inputDataset = inputDataset.addElement(timeseries(speedFeedback, time), 'SpeedRpm');
inputDataset = inputDataset.addElement(timeseries(phaseCurrentA, time), 'PhaseCurrentA');
inputDataset = inputDataset.addElement(timeseries(phaseCurrentB, time), 'PhaseCurrentB');
inputDataset = inputDataset.addElement(timeseries(electricalAngle, time), 'ElectricalAngleRad');
inputDataset = inputDataset.addElement(timeseries(dcBusVoltage, time), 'DcBusVoltage');

commonInput = Simulink.SimulationInput(modelName);
commonInput = commonInput.setExternalInput(inputDataset);
commonInput = commonInput.setModelParameter( ...
    'StopTime', num2str(stopTime, '%.9g'), ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');

normalInput = commonInput.setModelParameter('SimulationMode', 'normal');
normalOutput = sim(normalInput);
silInput = commonInput.setModelParameter('SimulationMode', 'software-in-the-loop (sil)');
silOutput = sim(silInput);

normalDataset = normalOutput.yout;
silDataset = silOutput.yout;
maximumAbsoluteError = 0.0;
for index = 1:normalDataset.numElements
    normalData = double(normalDataset{index}.Values.Data);
    silData = double(silDataset{index}.Values.Data);
    signalError = max(abs(normalData(:) - silData(:)));
    maximumAbsoluteError = max(maximumAbsoluteError, signalError);
    fprintf('CODEX_SIL_%s_MAX_ABS_ERROR=%.9g\n', ...
        normalDataset{index}.Name, signalError);
end
fprintf('CODEX_SIL_MAX_ABS_ERROR=%.9g\n', maximumAbsoluteError);
% Normal mode and SIL can differ by a few single-precision ULPs because
% MATLAB and the target compiler may order floating-point operations
% differently. Five micro-units is a conservative absolute tolerance here.
fprintf('CODEX_SIL_PASS=%d\n', maximumAbsoluteError <= 5.0e-6);
end
