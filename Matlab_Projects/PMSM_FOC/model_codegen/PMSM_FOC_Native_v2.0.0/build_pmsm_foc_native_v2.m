function build_pmsm_foc_native_v2
%BUILD_PMSM_FOC_NATIVE_V2 Build native-Simulink PMSM FOC models.
% No S-Function, MATLAB Function, System object, or custom executable block
% is used in either generated model.

versionDirectory = fileparts(mfilename('fullpath'));
previousDirectory = pwd;
directoryCleanup = onCleanup(@() cd(previousDirectory));
cd(versionDirectory);

controllerName = 'PMSM_FOC_Native_Controller_v20';
harnessName = 'PMSM_FOC_Native_ClosedLoop_v20';
controllerFile = fullfile(versionDirectory, [controllerName '.slx']);
harnessFile = fullfile(versionDirectory, [harnessName '.slx']);

if bdIsLoaded(controllerName)
    close_system(controllerName, 0);
end
if bdIsLoaded(harnessName)
    close_system(harnessName, 0);
end

load_system('simulink');

controllerExists = exist(controllerFile, 'file') ~= 0;
harnessExists = exist(harnessFile, 'file') ~= 0;
if controllerExists && harnessExists
    load_system(controllerFile);
    load_system(harnessFile);
elseif controllerExists || harnessExists
    error(['The version package is incomplete: only one model file exists. ' ...
        'Restore the missing model or start with an empty version directory.']);
else
    createControllerModel(controllerName, controllerFile);
    createHarnessModel(harnessName, harnessFile);
end

applySpeedLoopTuning(controllerName);
applySpeedLoopTuning(harnessName);
set_param(harnessName, 'StopTime', '2.0');
save_system(controllerName, controllerFile);
save_system(harnessName, harnessFile);

set_param(harnessName, 'SimulationCommand', 'update');
simulationOutput = sim(harnessName, 'ReturnWorkspaceOutputs', 'on');

speedSeries = simulationOutput.get('native_speed_rpm');
torqueSeries = simulationOutput.get('native_torque_nm');
iqSeries = simulationOutput.get('native_iq_a');
iqReferenceSeries = simulationOutput.get('native_iq_ref_a');
dutyASeries = simulationOutput.get('native_duty_a');

finalSpeedRpm = double(speedSeries.Data(end));
maximumSpeedRpm = max(double(speedSeries.Data));
maximumAbsIqA = max(abs(double(iqSeries.Data)));
minimumDuty = min(double(dutyASeries.Data));
maximumDuty = max(double(dutyASeries.Data));
finalTorqueNm = double(torqueSeries.Data(end));
finiteSignals = all(isfinite(double(speedSeries.Data))) && ...
    all(isfinite(double(iqSeries.Data))) && ...
    all(isfinite(double(dutyASeries.Data)));

simulationPass = finiteSignals && ...
    finalSpeedRpm > 950.0 && finalSpeedRpm < 1050.0 && ...
    maximumSpeedRpm < 1200.0 && maximumAbsIqA <= 12.1 && ...
    minimumDuty >= 0.019 && maximumDuty <= 0.981;

figureHandle = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100 100 1100 760]);
subplot(3, 1, 1);
plot(speedSeries.Time, speedSeries.Data, 'LineWidth', 1.4);
hold on;
plot([0 2.0], [1000 1000], '--', 'LineWidth', 1.0);
grid on;
ylabel('Speed (rpm)');
legend('Measured', 'Reference', 'Location', 'southeast');
title('PMSM FOC Native Simulink v2.0.0');
subplot(3, 1, 2);
plot(iqSeries.Time, iqSeries.Data, 'LineWidth', 1.2);
hold on;
plot(iqReferenceSeries.Time, iqReferenceSeries.Data, '--', 'LineWidth', 1.2);
grid on;
ylabel('q current (A)');
legend('Iq', 'Iq reference', 'Location', 'best');
subplot(3, 1, 3);
plot(torqueSeries.Time, torqueSeries.Data, 'LineWidth', 1.2);
grid on;
ylabel('Torque (N m)');
xlabel('Time (s)');
exportgraphics(figureHandle, fullfile(versionDirectory, ...
    'PMSM_FOC_Native_v2_results.png'));
close(figureHandle);

forbiddenReport = inspectNativeBlocks(controllerName, harnessName);
if forbiddenReport.ForbiddenCount ~= 0
    error('Forbidden non-native blocks were found in the generated models.');
end

fprintf('CODEX_NATIVE_CONTROLLER=%s\n', controllerFile);
fprintf('CODEX_NATIVE_HARNESS=%s\n', harnessFile);
fprintf('CODEX_NATIVE_BLOCK_COUNT=%d\n', forbiddenReport.TotalBlockCount);
fprintf('CODEX_NATIVE_FORBIDDEN_COUNT=%d\n', forbiddenReport.ForbiddenCount);
fprintf('CODEX_NATIVE_FINAL_SPEED_RPM=%.6f\n', finalSpeedRpm);
fprintf('CODEX_NATIVE_MAX_SPEED_RPM=%.6f\n', maximumSpeedRpm);
fprintf('CODEX_NATIVE_MAX_ABS_IQ_A=%.6f\n', maximumAbsIqA);
fprintf('CODEX_NATIVE_DUTY_A_RANGE=[%.6f, %.6f]\n', minimumDuty, maximumDuty);
fprintf('CODEX_NATIVE_FINAL_TORQUE_NM=%.6f\n', finalTorqueNm);
fprintf('CODEX_NATIVE_SIM_PASS=%d\n', simulationPass);

if ~simulationPass
    error('Native closed-loop simulation did not meet acceptance limits.');
end

slbuild(controllerName);

generatedDirectory = fullfile(versionDirectory, [controllerName '_ert_rtw']);
generatedCFile = fullfile(generatedDirectory, [controllerName '.c']);
generatedHeaderFile = fullfile(generatedDirectory, [controllerName '.h']);
if ~exist(generatedCFile, 'file') || ~exist(generatedHeaderFile, 'file')
    error('Expected ERT C and header files were not generated.');
end

generatedCode = fileread(generatedCFile);
generatedHeader = fileread(generatedHeaderFile);
hasStep = contains(generatedHeader, [controllerName '_step']);
hasInitialize = contains(generatedHeader, [controllerName '_initialize']);
hasInputs = contains(generatedHeader, 'SpeedReferenceRpm') && ...
    contains(generatedHeader, 'PhaseCurrentA') && ...
    contains(generatedHeader, 'ElectricalAngleRad');
hasOutputs = contains(generatedHeader, 'DutyA') && ...
    contains(generatedHeader, 'DutyB') && contains(generatedHeader, 'DutyC');
hasControlMath = contains(generatedCode, 'sinf') && ...
    contains(generatedCode, 'cosf');
hasSFunctionText = contains(lower(generatedCode), 's-function') || ...
    contains(lower(generatedHeader), 's-function');
codePass = hasStep && hasInitialize && hasInputs && hasOutputs && ...
    hasControlMath && ~hasSFunctionText;

reportFile = fullfile(versionDirectory, 'verification_report.txt');
reportHandle = fopen(reportFile, 'w');
if reportHandle < 0
    error('Could not create verification report.');
end
reportCleanup = onCleanup(@() fclose(reportHandle));
fprintf(reportHandle, 'PMSM FOC Native Simulink v2.0.0 verification\n');
fprintf(reportHandle, 'Controller model: %s\n', controllerName);
fprintf(reportHandle, 'Closed-loop model: %s\n', harnessName);
fprintf(reportHandle, 'Total blocks inspected: %d\n', forbiddenReport.TotalBlockCount);
fprintf(reportHandle, 'Forbidden blocks: %d\n', forbiddenReport.ForbiddenCount);
fprintf(reportHandle, 'Final speed rpm: %.9g\n', finalSpeedRpm);
fprintf(reportHandle, 'Maximum speed rpm: %.9g\n', maximumSpeedRpm);
fprintf(reportHandle, 'Maximum abs Iq A: %.9g\n', maximumAbsIqA);
fprintf(reportHandle, 'Duty A minimum: %.9g\n', minimumDuty);
fprintf(reportHandle, 'Duty A maximum: %.9g\n', maximumDuty);
fprintf(reportHandle, 'Final torque Nm: %.9g\n', finalTorqueNm);
fprintf(reportHandle, 'Simulation pass: %d\n', simulationPass);
fprintf(reportHandle, 'ERT step entry point: %d\n', hasStep);
fprintf(reportHandle, 'ERT initialize entry point: %d\n', hasInitialize);
fprintf(reportHandle, 'Input interface present: %d\n', hasInputs);
fprintf(reportHandle, 'Output interface present: %d\n', hasOutputs);
fprintf(reportHandle, 'Generated trig control math present: %d\n', hasControlMath);
fprintf(reportHandle, 'Generated S-Function text present: %d\n', hasSFunctionText);
fprintf(reportHandle, 'Code verification pass: %d\n', codePass);
clear reportCleanup;

fprintf('CODEX_NATIVE_CODE_HAS_STEP=%d\n', hasStep);
fprintf('CODEX_NATIVE_CODE_HAS_INITIALIZE=%d\n', hasInitialize);
fprintf('CODEX_NATIVE_CODE_HAS_INPUTS=%d\n', hasInputs);
fprintf('CODEX_NATIVE_CODE_HAS_OUTPUTS=%d\n', hasOutputs);
fprintf('CODEX_NATIVE_CODE_HAS_TRIG=%d\n', hasControlMath);
fprintf('CODEX_NATIVE_CODE_HAS_SFUNCTION=%d\n', hasSFunctionText);
fprintf('CODEX_NATIVE_CODE_PASS=%d\n', codePass);

if ~codePass
    error('Generated C code did not pass structural verification.');
end

save_system(controllerName, controllerFile);
save_system(harnessName, harnessFile);
open_system(harnessName);
end

function createControllerModel(modelName, modelFile)
new_system(modelName);
setCommonModelConfiguration(modelName, 0.1);
set_param(modelName, ...
    'SystemTargetFile', 'ert.tlc', ...
    'TargetLang', 'C', ...
    'GenerateReport', 'off', ...
    'LaunchReport', 'off', ...
    'CodeInterfacePackaging', 'Nonreusable function', ...
    'DefaultParameterBehavior', 'Tunable', ...
    'SupportNonFinite', 'off');
assignNativeParameters(modelName);

inputNames = {'SpeedReferenceRpm', 'SpeedRpm', 'PhaseCurrentA', ...
    'PhaseCurrentB', 'ElectricalAngleRad', 'DcBusVoltage'};
outputNames = {'DutyA', 'DutyB', 'DutyC', 'IqReference', ...
    'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand'};

for index = 1:numel(inputNames)
    add_block('simulink/Sources/In1', [modelName '/' inputNames{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'SampleTime', '0.0001', ...
        'Position', [25 35+45*index 55 49+45*index]);
end

controllerPath = [modelName '/Native_FOC_Controller_100us'];
add_block('simulink/Ports & Subsystems/Subsystem', controllerPath, ...
    'Position', [170 60 440 350]);
buildControllerSubsystem(controllerPath);

for index = 1:numel(outputNames)
    add_block('simulink/Sinks/Out1', [modelName '/' outputNames{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [560 30+40*index 590 44+40*index]);
end

for index = 1:numel(inputNames)
    add_line(modelName, [inputNames{index} '/1'], ...
        ['Native_FOC_Controller_100us/' num2str(index)], 'autorouting', 'on');
end
for index = 1:numel(outputNames)
    add_line(modelName, ['Native_FOC_Controller_100us/' num2str(index)], ...
        [outputNames{index} '/1'], 'autorouting', 'on');
end

Simulink.BlockDiagram.arrangeSystem(modelName);
set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName, modelFile);
end

function createHarnessModel(modelName, modelFile)
new_system(modelName);
setCommonModelConfiguration(modelName, 2.0);
assignNativeParameters(modelName);

add_block('simulink/Sources/Step', [modelName '/Speed_Reference_Rpm'], ...
    'Time', '0.05', 'Before', '0', 'After', '1000', ...
    'SampleTime', '0.0001', 'OutDataTypeStr', 'single', ...
    'Position', [25 60 65 90]);
add_block('simulink/Sources/Step', [modelName '/Load_Torque_Nm'], ...
    'Time', '0.5', 'Before', '0', 'After', '0.2', ...
    'SampleTime', '0.0001', 'OutDataTypeStr', 'single', ...
    'Position', [25 360 65 390]);
add_block('simulink/Sources/Constant', [modelName '/DC_Bus_48V'], ...
    'Value', 'single(48.0)', 'OutDataTypeStr', 'single', ...
    'Position', [25 290 65 320]);

controllerPath = [modelName '/Native_FOC_Controller_100us'];
add_block('simulink/Ports & Subsystems/Subsystem', controllerPath, ...
    'Position', [210 80 455 315]);
buildControllerSubsystem(controllerPath);

inverterPath = [modelName '/Native_Average_Inverter'];
add_block('simulink/Ports & Subsystems/Subsystem', inverterPath, ...
    'Position', [535 105 700 245]);
buildInverterSubsystem(inverterPath);

plantPath = [modelName '/Native_Discrete_PMSM_Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', plantPath, ...
    'Position', [790 90 1015 300]);
buildPlantSubsystem(plantPath);

add_line(modelName, 'Speed_Reference_Rpm/1', ...
    'Native_FOC_Controller_100us/1', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/1', ...
    'Native_FOC_Controller_100us/2', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/3', ...
    'Native_FOC_Controller_100us/3', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/4', ...
    'Native_FOC_Controller_100us/4', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/2', ...
    'Native_FOC_Controller_100us/5', 'autorouting', 'on');
add_line(modelName, 'DC_Bus_48V/1', ...
    'Native_FOC_Controller_100us/6', 'autorouting', 'on');

for index = 1:3
    add_line(modelName, ['Native_FOC_Controller_100us/' num2str(index)], ...
        ['Native_Average_Inverter/' num2str(index)], 'autorouting', 'on');
end
add_line(modelName, 'DC_Bus_48V/1', ...
    'Native_Average_Inverter/4', 'autorouting', 'on');
add_line(modelName, 'Native_Average_Inverter/1', ...
    'Native_Discrete_PMSM_Plant/1', 'autorouting', 'on');
add_line(modelName, 'Native_Average_Inverter/2', ...
    'Native_Discrete_PMSM_Plant/2', 'autorouting', 'on');
add_line(modelName, 'Load_Torque_Nm/1', ...
    'Native_Discrete_PMSM_Plant/3', 'autorouting', 'on');

addToWorkspace(modelName, 'Speed_Log', 'native_speed_rpm', [1080 70 1170 100]);
addToWorkspace(modelName, 'Torque_Log', 'native_torque_nm', [1080 125 1170 155]);
addToWorkspace(modelName, 'Iq_Log', 'native_iq_a', [1080 180 1170 210]);
addToWorkspace(modelName, 'Iq_Ref_Log', 'native_iq_ref_a', [500 330 590 360]);
addToWorkspace(modelName, 'Duty_A_Log', 'native_duty_a', [500 380 590 410]);

add_line(modelName, 'Native_Discrete_PMSM_Plant/1', ...
    'Speed_Log/1', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/5', ...
    'Torque_Log/1', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/7', ...
    'Iq_Log/1', 'autorouting', 'on');
add_line(modelName, 'Native_FOC_Controller_100us/4', ...
    'Iq_Ref_Log/1', 'autorouting', 'on');
add_line(modelName, 'Native_FOC_Controller_100us/1', ...
    'Duty_A_Log/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Mux', [modelName '/Speed_Mux'], ...
    'Inputs', '2', 'Position', [1050 335 1055 385]);
add_block('simulink/Sinks/Scope', [modelName '/Speed_Scope'], ...
    'Position', [1110 340 1150 380]);
add_line(modelName, 'Speed_Reference_Rpm/1', 'Speed_Mux/1', 'autorouting', 'on');
add_line(modelName, 'Native_Discrete_PMSM_Plant/1', ...
    'Speed_Mux/2', 'autorouting', 'on');
add_line(modelName, 'Speed_Mux/1', 'Speed_Scope/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(modelName);
set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName, modelFile);
end

function setCommonModelConfiguration(modelName, stopTime)
set_param(modelName, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.0001', ...
    'StopTime', num2str(stopTime, '%.9g'), ...
    'SaveTime', 'on', ...
    'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SignalLogging', 'off');
end

function assignNativeParameters(modelName)
parameterDefinitions = {
    'FOC_Native_CurrentPeriod', single(1.0e-4), true;
    'FOC_Native_SpeedPeriod', single(1.0e-3), true;
    'FOC_Native_KpSpeed', single(0.02), true;
    'FOC_Native_KiSpeed', single(0.05), true;
    'FOC_Native_KpCurrent', single(1.0), true;
    'FOC_Native_KiCurrent', single(500.0), true;
    'FOC_Native_IqLimit', single(8.0), true;
    'FOC_Native_CurrentIntegratorLimit', single(30.0), true;
    'FOC_Native_VoltageLimit', single(26.0), true;
    'FOC_Native_DutyMin', single(0.02), true;
    'FOC_Native_DutyMax', single(0.98), true;
    'FOC_Native_Ld', single(1.0e-3), true;
    'FOC_Native_Lq', single(1.0e-3), true;
    'FOC_Native_FluxPM', single(0.05), true;
    'FOC_Native_PolePairs', single(4.0), true;
    'PMSM_Native_Rs', single(0.4), false;
    'PMSM_Native_J', single(2.0e-3), false;
    'PMSM_Native_B', single(1.0e-4), false;
    'NATIVE_RPM_TO_RAD_S', single(0.1047197551196598), false;
    'NATIVE_RAD_S_TO_RPM', single(9.549296585513721), false;
    'NATIVE_INV_SQRT3', single(0.5773502691896258), false;
    'NATIVE_SQRT3_BY2', single(0.8660254037844386), false};

workspace = get_param(modelName, 'ModelWorkspace');
for index = 1:size(parameterDefinitions, 1)
    if parameterDefinitions{index, 3}
        parameter = Simulink.Parameter(parameterDefinitions{index, 2});
        parameter.CoderInfo.StorageClass = 'ExportedGlobal';
        assignin(workspace, parameterDefinitions{index, 1}, parameter);
    else
        assignin(workspace, parameterDefinitions{index, 1}, ...
            parameterDefinitions{index, 2});
    end
end
end

function applySpeedLoopTuning(modelName)
workspace = get_param(modelName, 'ModelWorkspace');
parameterNames = {'FOC_Native_KpSpeed', 'FOC_Native_KiSpeed'};
parameterValues = {single(0.02), single(0.05)};
for index = 1:numel(parameterNames)
    parameter = getVariable(workspace, parameterNames{index});
    parameter.Value = parameterValues{index};
    assignin(workspace, parameterNames{index}, parameter);
end
end

function buildControllerSubsystem(parent)
Simulink.SubSystem.deleteContents(parent);
inputs = {'SpeedReferenceRpm', 'SpeedRpm', 'Ia', 'Ib', 'ThetaElectrical', 'Vdc'};
outputs = {'DutyA', 'DutyB', 'DutyC', 'IqReference', ...
    'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand'};
for index = 1:numel(inputs)
    add_block('simulink/Sources/In1', [parent '/' inputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [20 25+45*index 50 39+45*index]);
end
for index = 1:numel(outputs)
    add_block('simulink/Sinks/Out1', [parent '/' outputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [1250 20+38*index 1280 34+38*index]);
end

addTrig(parent, 'SinTheta', 'sin', [130 225 170 255]);
addTrig(parent, 'CosTheta', 'cos', [130 270 170 300]);
addGain(parent, 'Ib_x2', 'single(2.0)', [130 150 180 180]);
addSum(parent, 'Clarke_Sum', '++', [215 120 245 175]);
addGain(parent, 'InvSqrt3', 'NATIVE_INV_SQRT3', [280 130 350 165]);

addProduct(parent, 'Id_CosAlpha', '**', [395 115 430 145]);
addProduct(parent, 'Id_SinBeta', '**', [395 160 430 190]);
addSum(parent, 'Id_Sum', '++', [465 125 495 180]);
addProduct(parent, 'Iq_SinAlpha', '**', [395 210 430 240]);
addGain(parent, 'Iq_Negative', 'single(-1.0)', [455 210 510 240]);
addProduct(parent, 'Iq_CosBeta', '**', [395 255 430 285]);
addSum(parent, 'Iq_Sum', '++', [545 225 575 280]);

add_block('simulink/Discrete/Zero-Order Hold', [parent '/SpeedRef_1ms'], ...
    'SampleTime', '0.001', 'Position', [120 345 185 375]);
add_block('simulink/Discrete/Zero-Order Hold', [parent '/SpeedFb_1ms'], ...
    'SampleTime', '0.001', 'Position', [120 395 185 425]);
addSum(parent, 'Speed_Error', '+-', [220 350 250 410]);
addGain(parent, 'Speed_RpmToRad', 'NATIVE_RPM_TO_RAD_S', [285 365 365 395]);
addGain(parent, 'Speed_Kp', 'FOC_Native_KpSpeed', [405 340 475 370]);
addGain(parent, 'Speed_KiTs', ...
    'FOC_Native_KiSpeed*FOC_Native_SpeedPeriod', [405 405 500 435]);
add_block('simulink/Discrete/Unit Delay', [parent '/Speed_Integrator_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.001', ...
    'Position', [595 425 650 455]);
addSum(parent, 'Speed_Integrator_Add', '++', [535 390 565 445]);
addSaturation(parent, 'Speed_Integrator_Limit', ...
    '-FOC_Native_IqLimit', 'FOC_Native_IqLimit', [685 415 755 465]);
addSum(parent, 'Iq_Reference_Sum', '++', [695 335 725 390]);
addSaturation(parent, 'Iq_Reference_Limit', ...
    '-FOC_Native_IqLimit', 'FOC_Native_IqLimit', [765 340 835 390]);
add_block('simulink/Discrete/Zero-Order Hold', [parent '/Iq_Reference_100us'], ...
    'SampleTime', '0.0001', 'Position', [870 345 940 375]);

add_block('simulink/Sources/Constant', [parent '/Id_Reference_Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [620 500 670 530]);
addSum(parent, 'Id_Error', '+-', [710 490 740 545]);
addSum(parent, 'Iq_Error', '+-', [710 570 740 625]);
addGain(parent, 'Id_Kp', 'FOC_Native_KpCurrent', [780 485 850 515]);
addGain(parent, 'Iq_Kp', 'FOC_Native_KpCurrent', [780 565 850 595]);
addGain(parent, 'Id_KiTs', ...
    'FOC_Native_KiCurrent*FOC_Native_CurrentPeriod', [780 525 890 555]);
addGain(parent, 'Iq_KiTs', ...
    'FOC_Native_KiCurrent*FOC_Native_CurrentPeriod', [780 605 890 635]);
add_block('simulink/Discrete/Unit Delay', [parent '/Id_Integrator_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [995 520 1050 550]);
add_block('simulink/Discrete/Unit Delay', [parent '/Iq_Integrator_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [995 600 1050 630]);
addSum(parent, 'Id_Integrator_Add', '++', [920 500 950 555]);
addSum(parent, 'Iq_Integrator_Add', '++', [920 580 950 635]);
addSaturation(parent, 'Id_Integrator_Limit', ...
    '-FOC_Native_CurrentIntegratorLimit', ...
    'FOC_Native_CurrentIntegratorLimit', [1085 510 1175 560]);
addSaturation(parent, 'Iq_Integrator_Limit', ...
    '-FOC_Native_CurrentIntegratorLimit', ...
    'FOC_Native_CurrentIntegratorLimit', [1085 590 1175 640]);

addGain(parent, 'Electrical_Speed', ...
    'NATIVE_RPM_TO_RAD_S*FOC_Native_PolePairs', [620 680 740 710]);
addProduct(parent, 'Omega_x_Iq', '**', [785 660 820 690]);
addGain(parent, 'D_Decoupling', '-FOC_Native_Lq', [855 660 930 690]);
addGain(parent, 'Ld_x_Id', 'FOC_Native_Ld', [785 720 850 750]);
add_block('simulink/Sources/Constant', [parent '/Flux_PM'], ...
    'Value', 'FOC_Native_FluxPM', 'OutDataTypeStr', 'single', ...
    'Position', [785 770 845 800]);
addSum(parent, 'Flux_Linkage', '++', [890 720 920 780]);
addProduct(parent, 'Q_Feedforward', '**', [965 720 1000 750]);
addSum(parent, 'Vd_Raw', '+++', [1050 675 1080 745]);
addSum(parent, 'Vq_Raw', '+++', [1050 765 1080 835]);
addSaturation(parent, 'Vd_Limit', '-FOC_Native_VoltageLimit', ...
    'FOC_Native_VoltageLimit', [1120 680 1200 730]);
addSaturation(parent, 'Vq_Limit', '-FOC_Native_VoltageLimit', ...
    'FOC_Native_VoltageLimit', [1120 770 1200 820]);

addProduct(parent, 'Valpha_CosVd', '**', [130 875 165 905]);
addProduct(parent, 'Valpha_SinVq', '**', [130 920 165 950]);
addSum(parent, 'Valpha_Sum', '+-', [205 880 235 940]);
addProduct(parent, 'Vbeta_SinVd', '**', [130 980 165 1010]);
addProduct(parent, 'Vbeta_CosVq', '**', [130 1025 165 1055]);
addSum(parent, 'Vbeta_Sum', '++', [205 985 235 1045]);
addGain(parent, 'Vb_Alpha', 'single(-0.5)', [290 900 350 930]);
addGain(parent, 'Vb_Beta', 'NATIVE_SQRT3_BY2', [290 950 365 980]);
addSum(parent, 'Phase_Vb', '++', [405 910 435 970]);
addGain(parent, 'Vc_Alpha', 'single(-0.5)', [290 1010 350 1040]);
addGain(parent, 'Vc_Beta', '-NATIVE_SQRT3_BY2', [290 1060 365 1090]);
addSum(parent, 'Phase_Vc', '++', [405 1020 435 1080]);
add_block('simulink/Math Operations/MinMax', [parent '/Phase_Maximum'], ...
    'Function', 'max', 'Inputs', '3', 'Position', [500 885 545 965]);
add_block('simulink/Math Operations/MinMax', [parent '/Phase_Minimum'], ...
    'Function', 'min', 'Inputs', '3', 'Position', [500 1000 545 1080]);
addSum(parent, 'Max_Plus_Min', '++', [585 930 615 1010]);
addGain(parent, 'Common_Mode', 'single(-0.5)', [650 955 710 985]);
add_block('simulink/Sources/Constant', [parent '/Duty_Half'], ...
    'Value', 'single(0.5)', 'OutDataTypeStr', 'single', ...
    'Position', [755 1110 810 1140]);

phaseBlocks = {'Valpha_Sum', 'Phase_Vb', 'Phase_Vc'};
dutyNames = {'A', 'B', 'C'};
for index = 1:3
    yPosition = 875 + (index-1)*105;
    addSum(parent, ['Phase_' dutyNames{index} '_Plus_Common'], '++', ...
        [755 yPosition 785 yPosition+55]);
    addProduct(parent, ['Duty_' dutyNames{index} '_Divide_Vdc'], '*/', ...
        [840 yPosition+5 880 yPosition+45]);
    addSum(parent, ['Duty_' dutyNames{index} '_Plus_Half'], '++', ...
        [920 yPosition 950 yPosition+55]);
    addSaturation(parent, ['Duty_' dutyNames{index} '_Limit'], ...
        'FOC_Native_DutyMin', 'FOC_Native_DutyMax', ...
        [995 yPosition 1070 yPosition+55]);
    add_line(parent, [phaseBlocks{index} '/1'], ...
        ['Phase_' dutyNames{index} '_Plus_Common/1'], 'autorouting', 'on');
    add_line(parent, 'Common_Mode/1', ...
        ['Phase_' dutyNames{index} '_Plus_Common/2'], 'autorouting', 'on');
    add_line(parent, ['Phase_' dutyNames{index} '_Plus_Common/1'], ...
        ['Duty_' dutyNames{index} '_Divide_Vdc/1'], 'autorouting', 'on');
    add_line(parent, 'Vdc/1', ...
        ['Duty_' dutyNames{index} '_Divide_Vdc/2'], 'autorouting', 'on');
    add_line(parent, ['Duty_' dutyNames{index} '_Divide_Vdc/1'], ...
        ['Duty_' dutyNames{index} '_Plus_Half/1'], 'autorouting', 'on');
    add_line(parent, 'Duty_Half/1', ...
        ['Duty_' dutyNames{index} '_Plus_Half/2'], 'autorouting', 'on');
    add_line(parent, ['Duty_' dutyNames{index} '_Plus_Half/1'], ...
        ['Duty_' dutyNames{index} '_Limit/1'], 'autorouting', 'on');
    add_line(parent, ['Duty_' dutyNames{index} '_Limit/1'], ...
        ['Duty' dutyNames{index} '/1'], 'autorouting', 'on');
end

add_line(parent, 'ThetaElectrical/1', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'CosTheta/1', 'autorouting', 'on');
add_line(parent, 'Ib/1', 'Ib_x2/1', 'autorouting', 'on');
add_line(parent, 'Ia/1', 'Clarke_Sum/1', 'autorouting', 'on');
add_line(parent, 'Ib_x2/1', 'Clarke_Sum/2', 'autorouting', 'on');
add_line(parent, 'Clarke_Sum/1', 'InvSqrt3/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Id_CosAlpha/1', 'autorouting', 'on');
add_line(parent, 'Ia/1', 'Id_CosAlpha/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Id_SinBeta/1', 'autorouting', 'on');
add_line(parent, 'InvSqrt3/1', 'Id_SinBeta/2', 'autorouting', 'on');
add_line(parent, 'Id_CosAlpha/1', 'Id_Sum/1', 'autorouting', 'on');
add_line(parent, 'Id_SinBeta/1', 'Id_Sum/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Iq_SinAlpha/1', 'autorouting', 'on');
add_line(parent, 'Ia/1', 'Iq_SinAlpha/2', 'autorouting', 'on');
add_line(parent, 'Iq_SinAlpha/1', 'Iq_Negative/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Iq_CosBeta/1', 'autorouting', 'on');
add_line(parent, 'InvSqrt3/1', 'Iq_CosBeta/2', 'autorouting', 'on');
add_line(parent, 'Iq_Negative/1', 'Iq_Sum/1', 'autorouting', 'on');
add_line(parent, 'Iq_CosBeta/1', 'Iq_Sum/2', 'autorouting', 'on');
add_line(parent, 'Id_Sum/1', 'IdMeasured/1', 'autorouting', 'on');
add_line(parent, 'Iq_Sum/1', 'IqMeasured/1', 'autorouting', 'on');

add_line(parent, 'SpeedReferenceRpm/1', 'SpeedRef_1ms/1', 'autorouting', 'on');
add_line(parent, 'SpeedRpm/1', 'SpeedFb_1ms/1', 'autorouting', 'on');
add_line(parent, 'SpeedRef_1ms/1', 'Speed_Error/1', 'autorouting', 'on');
add_line(parent, 'SpeedFb_1ms/1', 'Speed_Error/2', 'autorouting', 'on');
add_line(parent, 'Speed_Error/1', 'Speed_RpmToRad/1', 'autorouting', 'on');
add_line(parent, 'Speed_RpmToRad/1', 'Speed_Kp/1', 'autorouting', 'on');
add_line(parent, 'Speed_RpmToRad/1', 'Speed_KiTs/1', 'autorouting', 'on');
add_line(parent, 'Speed_KiTs/1', 'Speed_Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Speed_Integrator_State/1', ...
    'Speed_Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Speed_Integrator_Add/1', ...
    'Speed_Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Speed_Integrator_Limit/1', ...
    'Speed_Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Speed_Kp/1', 'Iq_Reference_Sum/1', 'autorouting', 'on');
add_line(parent, 'Speed_Integrator_State/1', ...
    'Iq_Reference_Sum/2', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Sum/1', ...
    'Iq_Reference_Limit/1', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Limit/1', ...
    'Iq_Reference_100us/1', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_100us/1', 'IqReference/1', 'autorouting', 'on');

add_line(parent, 'Id_Reference_Zero/1', 'Id_Error/1', 'autorouting', 'on');
add_line(parent, 'Id_Sum/1', 'Id_Error/2', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_100us/1', 'Iq_Error/1', 'autorouting', 'on');
add_line(parent, 'Iq_Sum/1', 'Iq_Error/2', 'autorouting', 'on');
add_line(parent, 'Id_Error/1', 'Id_Kp/1', 'autorouting', 'on');
add_line(parent, 'Iq_Error/1', 'Iq_Kp/1', 'autorouting', 'on');
add_line(parent, 'Id_Error/1', 'Id_KiTs/1', 'autorouting', 'on');
add_line(parent, 'Iq_Error/1', 'Iq_KiTs/1', 'autorouting', 'on');
add_line(parent, 'Id_KiTs/1', 'Id_Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Id_Integrator_State/1', ...
    'Id_Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Id_Integrator_Add/1', ...
    'Id_Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Id_Integrator_Limit/1', ...
    'Id_Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Iq_KiTs/1', 'Iq_Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Iq_Integrator_State/1', ...
    'Iq_Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Iq_Integrator_Add/1', ...
    'Iq_Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Iq_Integrator_Limit/1', ...
    'Iq_Integrator_State/1', 'autorouting', 'on');

add_line(parent, 'SpeedRpm/1', 'Electrical_Speed/1', 'autorouting', 'on');
add_line(parent, 'Electrical_Speed/1', 'Omega_x_Iq/1', 'autorouting', 'on');
add_line(parent, 'Iq_Sum/1', 'Omega_x_Iq/2', 'autorouting', 'on');
add_line(parent, 'Omega_x_Iq/1', 'D_Decoupling/1', 'autorouting', 'on');
add_line(parent, 'Id_Sum/1', 'Ld_x_Id/1', 'autorouting', 'on');
add_line(parent, 'Ld_x_Id/1', 'Flux_Linkage/1', 'autorouting', 'on');
add_line(parent, 'Flux_PM/1', 'Flux_Linkage/2', 'autorouting', 'on');
add_line(parent, 'Electrical_Speed/1', 'Q_Feedforward/1', 'autorouting', 'on');
add_line(parent, 'Flux_Linkage/1', 'Q_Feedforward/2', 'autorouting', 'on');
add_line(parent, 'Id_Kp/1', 'Vd_Raw/1', 'autorouting', 'on');
add_line(parent, 'Id_Integrator_State/1', 'Vd_Raw/2', 'autorouting', 'on');
add_line(parent, 'D_Decoupling/1', 'Vd_Raw/3', 'autorouting', 'on');
add_line(parent, 'Iq_Kp/1', 'Vq_Raw/1', 'autorouting', 'on');
add_line(parent, 'Iq_Integrator_State/1', 'Vq_Raw/2', 'autorouting', 'on');
add_line(parent, 'Q_Feedforward/1', 'Vq_Raw/3', 'autorouting', 'on');
add_line(parent, 'Vd_Raw/1', 'Vd_Limit/1', 'autorouting', 'on');
add_line(parent, 'Vq_Raw/1', 'Vq_Limit/1', 'autorouting', 'on');
add_line(parent, 'Vd_Limit/1', 'VdCommand/1', 'autorouting', 'on');
add_line(parent, 'Vq_Limit/1', 'VqCommand/1', 'autorouting', 'on');

add_line(parent, 'CosTheta/1', 'Valpha_CosVd/1', 'autorouting', 'on');
add_line(parent, 'Vd_Limit/1', 'Valpha_CosVd/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Valpha_SinVq/1', 'autorouting', 'on');
add_line(parent, 'Vq_Limit/1', 'Valpha_SinVq/2', 'autorouting', 'on');
add_line(parent, 'Valpha_CosVd/1', 'Valpha_Sum/1', 'autorouting', 'on');
add_line(parent, 'Valpha_SinVq/1', 'Valpha_Sum/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Vbeta_SinVd/1', 'autorouting', 'on');
add_line(parent, 'Vd_Limit/1', 'Vbeta_SinVd/2', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Vbeta_CosVq/1', 'autorouting', 'on');
add_line(parent, 'Vq_Limit/1', 'Vbeta_CosVq/2', 'autorouting', 'on');
add_line(parent, 'Vbeta_SinVd/1', 'Vbeta_Sum/1', 'autorouting', 'on');
add_line(parent, 'Vbeta_CosVq/1', 'Vbeta_Sum/2', 'autorouting', 'on');
add_line(parent, 'Valpha_Sum/1', 'Vb_Alpha/1', 'autorouting', 'on');
add_line(parent, 'Vbeta_Sum/1', 'Vb_Beta/1', 'autorouting', 'on');
add_line(parent, 'Vb_Alpha/1', 'Phase_Vb/1', 'autorouting', 'on');
add_line(parent, 'Vb_Beta/1', 'Phase_Vb/2', 'autorouting', 'on');
add_line(parent, 'Valpha_Sum/1', 'Vc_Alpha/1', 'autorouting', 'on');
add_line(parent, 'Vbeta_Sum/1', 'Vc_Beta/1', 'autorouting', 'on');
add_line(parent, 'Vc_Alpha/1', 'Phase_Vc/1', 'autorouting', 'on');
add_line(parent, 'Vc_Beta/1', 'Phase_Vc/2', 'autorouting', 'on');
add_line(parent, 'Valpha_Sum/1', 'Phase_Maximum/1', 'autorouting', 'on');
add_line(parent, 'Phase_Vb/1', 'Phase_Maximum/2', 'autorouting', 'on');
add_line(parent, 'Phase_Vc/1', 'Phase_Maximum/3', 'autorouting', 'on');
add_line(parent, 'Valpha_Sum/1', 'Phase_Minimum/1', 'autorouting', 'on');
add_line(parent, 'Phase_Vb/1', 'Phase_Minimum/2', 'autorouting', 'on');
add_line(parent, 'Phase_Vc/1', 'Phase_Minimum/3', 'autorouting', 'on');
add_line(parent, 'Phase_Maximum/1', 'Max_Plus_Min/1', 'autorouting', 'on');
add_line(parent, 'Phase_Minimum/1', 'Max_Plus_Min/2', 'autorouting', 'on');
add_line(parent, 'Max_Plus_Min/1', 'Common_Mode/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(parent);
end

function buildInverterSubsystem(parent)
Simulink.SubSystem.deleteContents(parent);
inputs = {'DutyA', 'DutyB', 'DutyC', 'Vdc'};
outputs = {'VAlpha', 'VBeta'};
for index = 1:numel(inputs)
    add_block('simulink/Sources/In1', [parent '/' inputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [20 40+55*index 50 54+55*index]);
end
for index = 1:numel(outputs)
    add_block('simulink/Sinks/Out1', [parent '/' outputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [620 80+90*index 650 94+90*index]);
end
add_block('simulink/Sources/Constant', [parent '/Half'], ...
    'Value', 'single(0.5)', 'OutDataTypeStr', 'single', ...
    'Position', [90 270 140 300]);
phaseNames = {'A', 'B', 'C'};
for index = 1:3
    yPosition = 60 + 70*index;
    addSum(parent, ['Duty_' phaseNames{index} '_Centered'], '+-', ...
        [180 yPosition 210 yPosition+50]);
    addProduct(parent, ['Phase_' phaseNames{index} '_Voltage'], '**', ...
        [265 yPosition+5 300 yPosition+45]);
    add_line(parent, ['Duty' phaseNames{index} '/1'], ...
        ['Duty_' phaseNames{index} '_Centered/1'], 'autorouting', 'on');
    add_line(parent, 'Half/1', ...
        ['Duty_' phaseNames{index} '_Centered/2'], 'autorouting', 'on');
    add_line(parent, ['Duty_' phaseNames{index} '_Centered/1'], ...
        ['Phase_' phaseNames{index} '_Voltage/1'], 'autorouting', 'on');
    add_line(parent, 'Vdc/1', ...
        ['Phase_' phaseNames{index} '_Voltage/2'], 'autorouting', 'on');
end
addGain(parent, 'Vb_MinusHalf', 'single(-0.5)', [350 185 410 215]);
addGain(parent, 'Vc_MinusHalf', 'single(-0.5)', [350 255 410 285]);
addSum(parent, 'Alpha_Sum', '+++', [450 145 480 235]);
addGain(parent, 'Alpha_TwoThirds', 'single(0.6666666667)', [520 170 590 200]);
addSum(parent, 'Beta_Difference', '+-', [450 250 480 310]);
addGain(parent, 'Beta_InvSqrt3', 'NATIVE_INV_SQRT3', [520 265 590 295]);
add_line(parent, 'Phase_A_Voltage/1', 'Alpha_Sum/1', 'autorouting', 'on');
add_line(parent, 'Phase_B_Voltage/1', 'Vb_MinusHalf/1', 'autorouting', 'on');
add_line(parent, 'Phase_C_Voltage/1', 'Vc_MinusHalf/1', 'autorouting', 'on');
add_line(parent, 'Vb_MinusHalf/1', 'Alpha_Sum/2', 'autorouting', 'on');
add_line(parent, 'Vc_MinusHalf/1', 'Alpha_Sum/3', 'autorouting', 'on');
add_line(parent, 'Alpha_Sum/1', 'Alpha_TwoThirds/1', 'autorouting', 'on');
add_line(parent, 'Alpha_TwoThirds/1', 'VAlpha/1', 'autorouting', 'on');
add_line(parent, 'Phase_B_Voltage/1', 'Beta_Difference/1', 'autorouting', 'on');
add_line(parent, 'Phase_C_Voltage/1', 'Beta_Difference/2', 'autorouting', 'on');
add_line(parent, 'Beta_Difference/1', 'Beta_InvSqrt3/1', 'autorouting', 'on');
add_line(parent, 'Beta_InvSqrt3/1', 'VBeta/1', 'autorouting', 'on');
Simulink.BlockDiagram.arrangeSystem(parent);
end

function buildPlantSubsystem(parent)
Simulink.SubSystem.deleteContents(parent);
inputs = {'VAlpha', 'VBeta', 'LoadTorque'};
outputs = {'SpeedRpm', 'ThetaElectrical', 'Ia', 'Ib', ...
    'TorqueNm', 'Id', 'Iq'};
for index = 1:numel(inputs)
    add_block('simulink/Sources/In1', [parent '/' inputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [20 45+60*index 50 59+60*index]);
end
for index = 1:numel(outputs)
    add_block('simulink/Sinks/Out1', [parent '/' outputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [1100 30+55*index 1130 44+55*index]);
end

add_block('simulink/Discrete/Unit Delay', [parent '/Id_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [300 390 355 420]);
add_block('simulink/Discrete/Unit Delay', [parent '/Iq_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [300 470 355 500]);
add_block('simulink/Discrete/Unit Delay', [parent '/Omega_Mechanical_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [300 550 355 580]);
add_block('simulink/Discrete/Unit Delay', [parent '/Theta_Electrical_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [300 630 355 660]);
addTrig(parent, 'SinTheta', 'sin', [405 620 445 650]);
addTrig(parent, 'CosTheta', 'cos', [405 670 445 700]);

addProduct(parent, 'Vd_CosAlpha', '**', [130 90 165 120]);
addProduct(parent, 'Vd_SinBeta', '**', [130 135 165 165]);
addSum(parent, 'Vd', '++', [205 100 235 155]);
addProduct(parent, 'Vq_SinAlpha', '**', [130 195 165 225]);
addGain(parent, 'Vq_Negative', 'single(-1.0)', [200 195 255 225]);
addProduct(parent, 'Vq_CosBeta', '**', [130 240 165 270]);
addSum(parent, 'Vq', '++', [290 210 320 265]);

addGain(parent, 'Omega_Electrical', 'FOC_Native_PolePairs', [405 545 490 575]);
addGain(parent, 'Rs_Id', 'PMSM_Native_Rs', [405 360 470 390]);
addProduct(parent, 'Omega_x_Iq', '**', [520 390 555 420]);
addGain(parent, 'Lq_Coupling', 'FOC_Native_Lq', [590 390 660 420]);
addSum(parent, 'Id_Derivative_Numerator', '+-+', [700 325 730 405]);
addGain(parent, 'Inv_Ld', '1/FOC_Native_Ld', [770 345 840 375]);
addGain(parent, 'Id_Delta', 'FOC_Native_CurrentPeriod', [875 345 960 375]);
addSum(parent, 'Id_State_Add', '++', [995 355 1025 410]);
addSaturation(parent, 'Id_State_Limit', 'single(-20.0)', 'single(20.0)', ...
    [1060 365 1140 415]);

addGain(parent, 'Rs_Iq', 'PMSM_Native_Rs', [405 445 470 475]);
addGain(parent, 'Ld_Id', 'FOC_Native_Ld', [520 450 585 480]);
add_block('simulink/Sources/Constant', [parent '/Flux_PM'], ...
    'Value', 'FOC_Native_FluxPM', 'OutDataTypeStr', 'single', ...
    'Position', [520 500 580 530]);
addSum(parent, 'Flux_Linkage', '++', [625 455 655 515]);
addProduct(parent, 'Back_EMF', '**', [700 455 735 485]);
addSum(parent, 'Iq_Derivative_Numerator', '+--', [770 430 800 510]);
addGain(parent, 'Inv_Lq', '1/FOC_Native_Lq', [835 450 905 480]);
addGain(parent, 'Iq_Delta', 'FOC_Native_CurrentPeriod', [935 450 1020 480]);
addSum(parent, 'Iq_State_Add', '++', [1050 450 1080 505]);
addSaturation(parent, 'Iq_State_Limit', 'single(-20.0)', 'single(20.0)', ...
    [1115 455 1195 505]);

addGain(parent, 'Electromagnetic_Torque', ...
    'single(1.5)*FOC_Native_PolePairs*FOC_Native_FluxPM', ...
    [520 555 670 585]);
addGain(parent, 'Viscous_Friction', 'PMSM_Native_B', [520 605 600 635]);
addSum(parent, 'Mechanical_Torque_Sum', '+--', [710 555 740 635]);
addGain(parent, 'Inv_Inertia', '1/PMSM_Native_J', [780 575 850 605]);
addGain(parent, 'Omega_Delta', 'FOC_Native_CurrentPeriod', [880 575 965 605]);
addSum(parent, 'Omega_State_Add', '++', [1000 575 1030 630]);
addSaturation(parent, 'Omega_State_Limit', 'single(-500.0)', 'single(500.0)', ...
    [1065 585 1150 635]);
addGain(parent, 'Speed_RadToRpm', 'NATIVE_RAD_S_TO_RPM', [520 665 610 695]);
addGain(parent, 'Theta_Delta', 'FOC_Native_CurrentPeriod', [520 720 605 750]);
addSum(parent, 'Theta_State_Add', '++', [650 710 680 765]);

addProduct(parent, 'Ialpha_CosId', '**', [745 690 780 720]);
addProduct(parent, 'Ialpha_SinIq', '**', [745 735 780 765]);
addSum(parent, 'Ialpha', '+-', [820 700 850 760]);
addProduct(parent, 'Ibeta_SinId', '**', [745 790 780 820]);
addProduct(parent, 'Ibeta_CosIq', '**', [745 835 780 865]);
addSum(parent, 'Ibeta', '++', [820 800 850 860]);
addGain(parent, 'Ib_Alpha', 'single(-0.5)', [890 760 950 790]);
addGain(parent, 'Ib_Beta', 'NATIVE_SQRT3_BY2', [890 820 965 850]);
addSum(parent, 'Phase_Ib', '++', [1000 780 1030 840]);

add_line(parent, 'Theta_Electrical_State/1', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'Theta_Electrical_State/1', 'CosTheta/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Vd_CosAlpha/1', 'autorouting', 'on');
add_line(parent, 'VAlpha/1', 'Vd_CosAlpha/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Vd_SinBeta/1', 'autorouting', 'on');
add_line(parent, 'VBeta/1', 'Vd_SinBeta/2', 'autorouting', 'on');
add_line(parent, 'Vd_CosAlpha/1', 'Vd/1', 'autorouting', 'on');
add_line(parent, 'Vd_SinBeta/1', 'Vd/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Vq_SinAlpha/1', 'autorouting', 'on');
add_line(parent, 'VAlpha/1', 'Vq_SinAlpha/2', 'autorouting', 'on');
add_line(parent, 'Vq_SinAlpha/1', 'Vq_Negative/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Vq_CosBeta/1', 'autorouting', 'on');
add_line(parent, 'VBeta/1', 'Vq_CosBeta/2', 'autorouting', 'on');
add_line(parent, 'Vq_Negative/1', 'Vq/1', 'autorouting', 'on');
add_line(parent, 'Vq_CosBeta/1', 'Vq/2', 'autorouting', 'on');

add_line(parent, 'Omega_Mechanical_State/1', 'Omega_Electrical/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Rs_Id/1', 'autorouting', 'on');
add_line(parent, 'Omega_Electrical/1', 'Omega_x_Iq/1', 'autorouting', 'on');
add_line(parent, 'Iq_State/1', 'Omega_x_Iq/2', 'autorouting', 'on');
add_line(parent, 'Omega_x_Iq/1', 'Lq_Coupling/1', 'autorouting', 'on');
add_line(parent, 'Vd/1', 'Id_Derivative_Numerator/1', 'autorouting', 'on');
add_line(parent, 'Rs_Id/1', 'Id_Derivative_Numerator/2', 'autorouting', 'on');
add_line(parent, 'Lq_Coupling/1', 'Id_Derivative_Numerator/3', 'autorouting', 'on');
add_line(parent, 'Id_Derivative_Numerator/1', 'Inv_Ld/1', 'autorouting', 'on');
add_line(parent, 'Inv_Ld/1', 'Id_Delta/1', 'autorouting', 'on');
add_line(parent, 'Id_Delta/1', 'Id_State_Add/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Id_State_Add/2', 'autorouting', 'on');
add_line(parent, 'Id_State_Add/1', 'Id_State_Limit/1', 'autorouting', 'on');
add_line(parent, 'Id_State_Limit/1', 'Id_State/1', 'autorouting', 'on');

add_line(parent, 'Iq_State/1', 'Rs_Iq/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Ld_Id/1', 'autorouting', 'on');
add_line(parent, 'Ld_Id/1', 'Flux_Linkage/1', 'autorouting', 'on');
add_line(parent, 'Flux_PM/1', 'Flux_Linkage/2', 'autorouting', 'on');
add_line(parent, 'Omega_Electrical/1', 'Back_EMF/1', 'autorouting', 'on');
add_line(parent, 'Flux_Linkage/1', 'Back_EMF/2', 'autorouting', 'on');
add_line(parent, 'Vq/1', 'Iq_Derivative_Numerator/1', 'autorouting', 'on');
add_line(parent, 'Rs_Iq/1', 'Iq_Derivative_Numerator/2', 'autorouting', 'on');
add_line(parent, 'Back_EMF/1', 'Iq_Derivative_Numerator/3', 'autorouting', 'on');
add_line(parent, 'Iq_Derivative_Numerator/1', 'Inv_Lq/1', 'autorouting', 'on');
add_line(parent, 'Inv_Lq/1', 'Iq_Delta/1', 'autorouting', 'on');
add_line(parent, 'Iq_Delta/1', 'Iq_State_Add/1', 'autorouting', 'on');
add_line(parent, 'Iq_State/1', 'Iq_State_Add/2', 'autorouting', 'on');
add_line(parent, 'Iq_State_Add/1', 'Iq_State_Limit/1', 'autorouting', 'on');
add_line(parent, 'Iq_State_Limit/1', 'Iq_State/1', 'autorouting', 'on');

add_line(parent, 'Iq_State/1', 'Electromagnetic_Torque/1', 'autorouting', 'on');
add_line(parent, 'Omega_Mechanical_State/1', 'Viscous_Friction/1', 'autorouting', 'on');
add_line(parent, 'Electromagnetic_Torque/1', 'Mechanical_Torque_Sum/1', 'autorouting', 'on');
add_line(parent, 'LoadTorque/1', 'Mechanical_Torque_Sum/2', 'autorouting', 'on');
add_line(parent, 'Viscous_Friction/1', 'Mechanical_Torque_Sum/3', 'autorouting', 'on');
add_line(parent, 'Mechanical_Torque_Sum/1', 'Inv_Inertia/1', 'autorouting', 'on');
add_line(parent, 'Inv_Inertia/1', 'Omega_Delta/1', 'autorouting', 'on');
add_line(parent, 'Omega_Delta/1', 'Omega_State_Add/1', 'autorouting', 'on');
add_line(parent, 'Omega_Mechanical_State/1', 'Omega_State_Add/2', 'autorouting', 'on');
add_line(parent, 'Omega_State_Add/1', 'Omega_State_Limit/1', 'autorouting', 'on');
add_line(parent, 'Omega_State_Limit/1', 'Omega_Mechanical_State/1', 'autorouting', 'on');
add_line(parent, 'Omega_Mechanical_State/1', 'Speed_RadToRpm/1', 'autorouting', 'on');
add_line(parent, 'Omega_Electrical/1', 'Theta_Delta/1', 'autorouting', 'on');
add_line(parent, 'Theta_Delta/1', 'Theta_State_Add/1', 'autorouting', 'on');
add_line(parent, 'Theta_Electrical_State/1', 'Theta_State_Add/2', 'autorouting', 'on');
add_line(parent, 'Theta_State_Add/1', 'Theta_Electrical_State/1', 'autorouting', 'on');

add_line(parent, 'CosTheta/1', 'Ialpha_CosId/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Ialpha_CosId/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Ialpha_SinIq/1', 'autorouting', 'on');
add_line(parent, 'Iq_State/1', 'Ialpha_SinIq/2', 'autorouting', 'on');
add_line(parent, 'Ialpha_CosId/1', 'Ialpha/1', 'autorouting', 'on');
add_line(parent, 'Ialpha_SinIq/1', 'Ialpha/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Ibeta_SinId/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Ibeta_SinId/2', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Ibeta_CosIq/1', 'autorouting', 'on');
add_line(parent, 'Iq_State/1', 'Ibeta_CosIq/2', 'autorouting', 'on');
add_line(parent, 'Ibeta_SinId/1', 'Ibeta/1', 'autorouting', 'on');
add_line(parent, 'Ibeta_CosIq/1', 'Ibeta/2', 'autorouting', 'on');
add_line(parent, 'Ialpha/1', 'Ib_Alpha/1', 'autorouting', 'on');
add_line(parent, 'Ibeta/1', 'Ib_Beta/1', 'autorouting', 'on');
add_line(parent, 'Ib_Alpha/1', 'Phase_Ib/1', 'autorouting', 'on');
add_line(parent, 'Ib_Beta/1', 'Phase_Ib/2', 'autorouting', 'on');

add_line(parent, 'Speed_RadToRpm/1', 'SpeedRpm/1', 'autorouting', 'on');
add_line(parent, 'Theta_Electrical_State/1', 'ThetaElectrical/1', 'autorouting', 'on');
add_line(parent, 'Ialpha/1', 'Ia/1', 'autorouting', 'on');
add_line(parent, 'Phase_Ib/1', 'Ib/1', 'autorouting', 'on');
add_line(parent, 'Electromagnetic_Torque/1', 'TorqueNm/1', 'autorouting', 'on');
add_line(parent, 'Id_State/1', 'Id/1', 'autorouting', 'on');
add_line(parent, 'Iq_State/1', 'Iq/1', 'autorouting', 'on');
Simulink.BlockDiagram.arrangeSystem(parent);
end

function report = inspectNativeBlocks(controllerName, harnessName)
modelNames = {controllerName, harnessName};
totalBlocks = 0;
forbiddenPaths = {};
for modelIndex = 1:numel(modelNames)
    blocks = find_system(modelNames{modelIndex}, ...
        'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
    totalBlocks = totalBlocks + numel(blocks);
    for blockIndex = 1:numel(blocks)
        blockType = get_param(blocks{blockIndex}, 'BlockType');
        maskType = get_param(blocks{blockIndex}, 'MaskType');
        if strcmp(blockType, 'S-Function') || ...
                contains(lower(maskType), 's-function') || ...
                contains(lower(maskType), 'matlab function') || ...
                strcmp(blockType, 'MATLABSystem')
            forbiddenPaths{end+1} = blocks{blockIndex}; %#ok<AGROW>
        end
    end
end
rootObject = sfroot;
charts = rootObject.find('-isa', 'Stateflow.EMChart');
for chartIndex = 1:numel(charts)
    chartPath = charts(chartIndex).Path;
    if startsWith(chartPath, controllerName) || startsWith(chartPath, harnessName)
        forbiddenPaths{end+1} = chartPath; %#ok<AGROW>
    end
end
report.TotalBlockCount = totalBlocks;
report.ForbiddenCount = numel(forbiddenPaths);
report.ForbiddenPaths = forbiddenPaths;
end

function addGain(parent, name, gainValue, position)
add_block('simulink/Math Operations/Gain', [parent '/' name], ...
    'Gain', gainValue, 'Position', position);
end

function addSum(parent, name, signs, position)
add_block('simulink/Math Operations/Sum', [parent '/' name], ...
    'Inputs', signs, 'Position', position);
end

function addProduct(parent, name, inputs, position)
add_block('simulink/Math Operations/Product', [parent '/' name], ...
    'Inputs', inputs, 'Position', position);
end

function addTrig(parent, name, operator, position)
add_block('simulink/Math Operations/Trigonometric Function', ...
    [parent '/' name], 'Operator', operator, 'Position', position);
end

function addSaturation(parent, name, lowerLimit, upperLimit, position)
add_block('simulink/Discontinuities/Saturation', [parent '/' name], ...
    'LowerLimit', lowerLimit, 'UpperLimit', upperLimit, ...
    'Position', position);
end

function addToWorkspace(parent, name, variableName, position)
add_block('simulink/Sinks/To Workspace', [parent '/' name], ...
    'VariableName', variableName, 'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', '20000', 'Position', position);
end
