function build_pmsm_foc_controller_codegen
%BUILD_PMSM_FOC_CONTROLLER_CODEGEN Build a deployable FOC controller model.

% The simulation plant remains in PMSM_FOC_Basic_v2.slx. This model contains
% only the software controller, which is the portion deployed to an ECU.

workDirectory = fileparts(mfilename('fullpath'));
oldDirectory = pwd;
cleanupDirectory = onCleanup(@() cd(oldDirectory));
cd(workDirectory);

logFile = fullfile(workDirectory, 'PMSM_FOC_Controller_Codegen_v2_build.log');
diary(logFile);
cleanupDiary = onCleanup(@() diary('off'));

modelName = 'PMSM_FOC_Controller_Codegen_v2';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
modelFile = fullfile(workDirectory, [modelName '.slx']);

new_system(modelName);
load_system('simulink');
set_param(modelName, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.0001', ...
    'StopTime', '0.1', ...
    'SystemTargetFile', 'ert.tlc', ...
    'TargetLang', 'C', ...
    'GenerateReport', 'off', ...
    'LaunchReport', 'off', ...
    'CodeInterfacePackaging', 'Nonreusable function', ...
    'DefaultParameterBehavior', 'Tunable', ...
    'SupportNonFinite', 'off');

parameterDefinitions = {
    'FOC_CurrentPeriod',       single(1.0e-4);
    'FOC_SpeedPeriod',         single(1.0e-3);
    'FOC_PolePairs',           single(4.0);
    'FOC_Ld',                  single(1.0e-3);
    'FOC_Lq',                  single(1.0e-3);
    'FOC_FluxPM',              single(0.05);
    'FOC_KpCurrent',           single(1.0);
    'FOC_KiCurrent',           single(500.0);
    'FOC_KpSpeed',             single(0.20);
    'FOC_KiSpeed',             single(3.0);
    'FOC_IqLimit',             single(8.0);
    'FOC_VoltageUtilization',  single(0.95);
    'FOC_AntiWindupGain',      single(0.20);
    'FOC_DutyMin',             single(0.02);
    'FOC_DutyMax',             single(0.98)};
modelWorkspace = get_param(modelName, 'ModelWorkspace');
for index = 1:size(parameterDefinitions, 1)
    parameter = Simulink.Parameter(parameterDefinitions{index, 2});
    parameter.CoderInfo.StorageClass = 'ExportedGlobal';
    assignin(modelWorkspace, parameterDefinitions{index, 1}, parameter);
end

inputNames = {'SpeedReferenceRpm', 'SpeedRpm', 'PhaseCurrentA', ...
    'PhaseCurrentB', 'ElectricalAngleRad', 'DcBusVoltage'};
inputY = 55:55:330;
for index = 1:numel(inputNames)
    blockPath = [modelName '/' inputNames{index}];
    add_block('simulink/Sources/In1', blockPath, ...
        'Position', [30 inputY(index) 60 inputY(index)+14], ...
        'Port', num2str(index), ...
        'OutDataTypeStr', 'single', ...
        'SampleTime', '0.0001');
end

controllerPath = [modelName '/FOC_Controller_100us'];
add_block('simulink/User-Defined Functions/MATLAB Function', controllerPath, ...
    'Position', [180 65 440 340]);
rootObject = sfroot;
chart = rootObject.find('-isa', 'Stateflow.EMChart', 'Path', controllerPath);
if isempty(chart)
    error('Could not find MATLAB Function chart at %s.', controllerPath);
end
chart = chart(1);
chart.Script = fileread(fullfile(workDirectory, 'foc_controller_codegen_core.m'));
for index = 1:size(parameterDefinitions, 1)
    parameterData = Stateflow.Data(chart);
    parameterData.Name = parameterDefinitions{index, 1};
    parameterData.Scope = 'Parameter';
end

outputNames = {'DutyA', 'DutyB', 'DutyC', 'IqReference', ...
    'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand'};
outputY = 45:42:339;
for index = 1:numel(outputNames)
    blockPath = [modelName '/' outputNames{index}];
    add_block('simulink/Sinks/Out1', blockPath, ...
        'Position', [570 outputY(index) 600 outputY(index)+14], ...
        'Port', num2str(index), ...
        'OutDataTypeStr', 'single');
end

for index = 1:numel(inputNames)
    add_line(modelName, [inputNames{index} '/1'], ...
        ['FOC_Controller_100us/' num2str(index)], 'autorouting', 'on');
end
for index = 1:numel(outputNames)
    add_line(modelName, ['FOC_Controller_100us/' num2str(index)], ...
        [outputNames{index} '/1'], 'autorouting', 'on');
end

set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName, modelFile);
fprintf('CODEX_CODEGEN_MODEL=%s\n', modelFile);
fprintf('CODEX_CODEGEN_TARGET=%s\n', get_param(modelName, 'SystemTargetFile'));
fprintf('CODEX_CODEGEN_FIXED_STEP=%s\n', get_param(modelName, 'FixedStep'));

try
    buildResult = slbuild(modelName);
    %#ok<NASGU>
    fprintf('CODEX_CODEGEN_PASS=1\n');
catch exception
    fprintf('CODEX_CODEGEN_PASS=0\n');
    fprintf('%s\n', getReport(exception, 'extended', 'hyperlinks', 'off'));
    save_system(modelName, modelFile);
    rethrow(exception);
end

open_system(modelName);
end
