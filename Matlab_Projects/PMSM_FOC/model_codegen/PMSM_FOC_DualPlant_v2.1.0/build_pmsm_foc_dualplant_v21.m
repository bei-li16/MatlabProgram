function runReport = build_pmsm_foc_dualplant_v21(varargin)
%BUILD_PMSM_FOC_DUALPLANT_V21 Create a selectable native/MathWorks PMSM model.
% The v2.0.0 native model remains unchanged. This builder copies it into a
% new version package, adds the official Motor Control Blockset PMSM HDL
% plant as a second Variant Subsystem choice, simulates both choices, and
% regenerates the standalone ERT controller code.
% Optional name/value arguments:
%   CleanBuild - replace both v2.1.0 models with their v2.0.0 source models
%                before applying the deterministic v2.1.0 refactor.
%   BatchMode  - suppress opening model windows after the build.
%   ExportImages - export model architecture PNG files. Disable this for
%                  repeated CI builds to keep peak MATLAB memory bounded.

options = parseBuildOptions(varargin{:});
buildTimer = tic;
buildStartedAt = currentTimestamp();

versionDirectory = fileparts(mfilename('fullpath'));
sourceDirectory = fullfile(fileparts(versionDirectory), ...
    'PMSM_FOC_Native_v2.0.0');
previousDirectory = pwd;
directoryCleanup = onCleanup(@() cd(previousDirectory));
cd(versionDirectory);

sourceControllerName = 'PMSM_FOC_Native_Controller_v20';
sourceHarnessName = 'PMSM_FOC_Native_ClosedLoop_v20';
controllerName = 'PMSM_FOC_DualPlant_Controller_v21';
harnessName = 'PMSM_FOC_DualPlant_ClosedLoop_v21';
sourceControllerFile = fullfile(sourceDirectory, [sourceControllerName '.slx']);
sourceHarnessFile = fullfile(sourceDirectory, [sourceHarnessName '.slx']);
controllerFile = fullfile(versionDirectory, [controllerName '.slx']);
harnessFile = fullfile(versionDirectory, [harnessName '.slx']);

assert(exist(sourceControllerFile, 'file') ~= 0, 'Source controller is missing.');
assert(exist(sourceHarnessFile, 'file') ~= 0, 'Source harness is missing.');
environmentReport = inspectBuildEnvironment(versionDirectory);
verifyRequiredProducts(environmentReport);

closeIfLoaded(controllerName);
closeIfLoaded(harnessName);
if options.CleanBuild
    [controllerCopyOk, controllerCopyMessage] = copyfile( ...
        sourceControllerFile, controllerFile, 'f');
    assert(controllerCopyOk, 'Unable to reset controller model: %s', ...
        controllerCopyMessage);
    [harnessCopyOk, harnessCopyMessage] = copyfile( ...
        sourceHarnessFile, harnessFile, 'f');
    assert(harnessCopyOk, 'Unable to reset harness model: %s', ...
        harnessCopyMessage);
else
    if exist(controllerFile, 'file') == 0
        copyfile(sourceControllerFile, controllerFile);
    end
    if exist(harnessFile, 'file') == 0
        copyfile(sourceHarnessFile, harnessFile);
    end
end

load_system('simulink');
load_system('mcbhdlplantlib');
load_system('simulink_hmi_blocks');
load_system(controllerFile);
load_system(harnessFile);
if options.BatchMode
    modelCleanup = onCleanup(@() discardAndCloseBuildModels( ...
        controllerName, harnessName));
end

configureDualPlantHarness(harnessName, harnessFile, versionDirectory);
refactor_pmsm_foc_controller_v21(options.BatchMode);

nativeOutput = simulatePlant(harnessName, 1);
mcbOutput = simulatePlant(harnessName, 2);
nativeMetrics = collectMetrics(nativeOutput);
mcbMetrics = collectMetrics(mcbOutput);
stateflowReport = collectStateflowMetrics(mcbOutput);
calibrationOutput = simulateCalibrationOffset(harnessName);
calibrationReport = collectCalibrationMetrics(calibrationOutput);
faultOutput = simulateOvervoltageFault(harnessName);
faultReport = collectFaultMetrics(faultOutput);
fastFaultOutput = simulateFastOvercurrent(harnessName);
fastFaultReport = collectFastFaultMetrics(fastFaultOutput, 0.0903);
faultMatrixReport = verifyFaultLatchFromEveryState(harnessName);
faultResetOutput = simulateAcknowledgedFaultReset(harnessName);
faultResetReport = collectFaultResetMetrics(faultResetOutput);
resetHarnessStimuli(harnessName);

nativePass = metricsPass(nativeMetrics);
mcbPass = metricsPass(mcbMetrics);
fprintf('CODEX_DUAL_NATIVE_FINAL_SPEED_RPM=%.9g\n', nativeMetrics.FinalSpeedRpm);
fprintf('CODEX_DUAL_NATIVE_MAX_SPEED_RPM=%.9g\n', nativeMetrics.MaximumSpeedRpm);
fprintf('CODEX_DUAL_NATIVE_MAX_ABS_IQ_A=%.9g\n', nativeMetrics.MaximumAbsIqA);
fprintf('CODEX_DUAL_NATIVE_PASS=%d\n', nativePass);
fprintf('CODEX_DUAL_MCB_FINAL_SPEED_RPM=%.9g\n', mcbMetrics.FinalSpeedRpm);
fprintf('CODEX_DUAL_MCB_MAX_SPEED_RPM=%.9g\n', mcbMetrics.MaximumSpeedRpm);
fprintf('CODEX_DUAL_MCB_MAX_ABS_IQ_A=%.9g\n', mcbMetrics.MaximumAbsIqA);
fprintf('CODEX_DUAL_MCB_PASS=%d\n', mcbPass);
fprintf('CODEX_STATEFLOW_RUN_ENTRY_S=%.9g\n', stateflowReport.RunEntryTimeS);
fprintf('CODEX_STATEFLOW_NORMAL_PASS=%d\n', stateflowReport.Pass);
fprintf('CODEX_STATEFLOW_FAULT_ENTRY_S=%.9g\n', faultReport.FaultEntryTimeS);
fprintf('CODEX_STATEFLOW_FAULT_PASS=%d\n', faultReport.Pass);
fprintf('CODEX_FAST_FAULT_ENTRY_S=%.9g\n', fastFaultReport.FastFaultEntryTimeS);
fprintf('CODEX_FAST_PWM_DISABLE_LATENCY_S=%.9g\n', ...
    fastFaultReport.PwmDisableLatencyS);
fprintf('CODEX_FAST_OVERCURRENT_PASS=%d\n', fastFaultReport.Pass);
fprintf('CODEX_STATEFLOW_CALIBRATION_PASS=%d\n', calibrationReport.Pass);
fprintf('CODEX_STATEFLOW_ALIGNMENT_PASS=%d\n', stateflowReport.AlignmentPass);
fprintf('CODEX_STATEFLOW_ALL_STATE_FAULT_LATCH_PASS=%d\n', faultMatrixReport.Pass);
fprintf('CODEX_STATEFLOW_ACK_RESET_PASS=%d\n', faultResetReport.Pass);

createComparisonPlot(nativeOutput, mcbOutput, versionDirectory);
createStateflowTestPlot(mcbOutput, calibrationOutput, faultOutput, ...
    fastFaultOutput, faultResetOutput, versionDirectory);
if options.ExportImages
    exportArchitectureImages(controllerName, harnessName, versionDirectory);
end
if ~(nativePass && mcbPass && stateflowReport.Pass && calibrationReport.Pass && ...
        faultReport.Pass && fastFaultReport.Pass && faultMatrixReport.Pass && ...
        faultResetReport.Pass)
    error('A plant or Stateflow verification scenario failed.');
end

setPlantSelection(harnessName, 2);
save_system(harnessName, harnessFile);

slbuild(controllerName);
save_system(controllerName, controllerFile, ...
    'OverwriteIfChangedOnDisk', true);
save_system(harnessName, harnessFile, ...
    'OverwriteIfChangedOnDisk', true);
codeReport = inspectGeneratedControllerCode(controllerName, versionDirectory);
variantReport = inspectVariantStructure(harnessName);
architectureReport = inspectControllerArchitecture(controllerName, harnessName);
buildCompletedAt = currentTimestamp();
runReport = createBuildRunReport(controllerName, harnessName, options, ...
    environmentReport, buildStartedAt, buildCompletedAt, toc(buildTimer), ...
    nativeMetrics, mcbMetrics, nativePass, mcbPass, architectureReport, ...
    codeReport, stateflowReport, calibrationReport, faultReport, ...
    fastFaultReport, faultMatrixReport, faultResetReport);
writeVerificationReport(versionDirectory, controllerName, harnessName, ...
    nativeMetrics, mcbMetrics, nativePass, mcbPass, variantReport, ...
    architectureReport, codeReport, stateflowReport, calibrationReport, ...
    faultReport, fastFaultReport, faultMatrixReport, faultResetReport, ...
    runReport);

assert(runReport.OverallPass, ...
    'Structure, code generation, or one or more verification scenarios failed.');

fprintf('CODEX_DUAL_VARIANT_CHOICES=%d\n', variantReport.ChoiceCount);
fprintf('CODEX_DUAL_MCB_REFERENCE_OK=%d\n', variantReport.McbReferencePresent);
fprintf('CODEX_DUAL_ONE_CLICK_LINK=%d\n', variantReport.OneClickLinkPresent);
fprintf('CODEX_DUAL_FOC_COMPONENTS=%d\n', architectureReport.ComponentCount);
fprintf('CODEX_DUAL_SPEED_TASK_S=%.9g\n', architectureReport.SpeedTaskSampleTime);
fprintf('CODEX_DUAL_CURRENT_TASK_S=%.9g\n', architectureReport.CurrentTaskSampleTime);
fprintf('CODEX_DUAL_CONTROLLER_DANGLING_LINES=%d\n', architectureReport.ControllerDanglingLines);
fprintf('CODEX_DUAL_HARNESS_DANGLING_LINES=%d\n', architectureReport.HarnessDanglingLines);
fprintf('CODEX_DUAL_ARCHITECTURE_PASS=%d\n', architectureReport.Pass);
fprintf('CODEX_DUAL_CODE_PASS=%d\n', codeReport.Pass);
fprintf('CODEX_DUAL_CODE_SINF_CALLS=%d\n', codeReport.SinfCallCount);
fprintf('CODEX_DUAL_CODE_COSF_CALLS=%d\n', codeReport.CosfCallCount);
fprintf('CODEX_DUAL_STATEFLOW_STATES=%d\n', architectureReport.StateflowStateCount);
fprintf('CODEX_DUAL_STATEFLOW_TASK_S=%.9g\n', architectureReport.StateflowSampleTime);
fprintf('CODEX_DUAL_FAST_SAFETY_TASK_S=%.9g\n', ...
    architectureReport.FastSafetySampleTime);
fprintf('CODEX_DUAL_SLOW_SAFETY_TASK_S=%.9g\n', ...
    architectureReport.SlowSafetySampleTime);
fprintf('CODEX_DUAL_DEFAULT_SELECTION=2\n');
fprintf('CODEX_DUAL_CONTROLLER_CHECKSUM=%s\n', runReport.ControllerChecksum);
fprintf('CODEX_DUAL_HARNESS_CHECKSUM=%s\n', runReport.HarnessChecksum);
fprintf('CODEX_DUAL_BUILD_DURATION_S=%.9g\n', runReport.BuildDurationSeconds);
fprintf('CODEX_DUAL_OVERALL_PASS=%d\n', runReport.OverallPass);

if options.BatchMode
    close_system(harnessName, 0);
    close_system(controllerName, 0);
else
    open_system([harnessName '/Selectable_PMSM_Plant']);
end
end

function options = parseBuildOptions(varargin)
parser = inputParser;
parser.FunctionName = 'build_pmsm_foc_dualplant_v21';
addParameter(parser, 'CleanBuild', false, ...
    @(value) islogical(value) && isscalar(value));
addParameter(parser, 'BatchMode', false, ...
    @(value) islogical(value) && isscalar(value));
addParameter(parser, 'ExportImages', true, ...
    @(value) islogical(value) && isscalar(value));
parse(parser, varargin{:});
options = parser.Results;
end

function timestamp = currentTimestamp()
value = datetime('now', 'TimeZone', 'local', ...
    'Format', "yyyy-MM-dd'T'HH:mm:ssXXX");
timestamp = char(value);
end

function report = inspectBuildEnvironment(versionDirectory)
report.MATLABVersion = version;
report.MATLABRelease = version('-release');
report.Architecture = computer('arch');
report.OperatingSystem = strtrim(system_dependent('getos'));

requiredNames = {'MATLAB', 'Simulink', 'Stateflow', 'Simulink Coder', ...
    'Embedded Coder', 'Motor Control Blockset'};
licenseFeatures = {'MATLAB', 'Simulink', 'Stateflow', ...
    'Real-Time_Workshop', 'RTW_Embedded_Coder', 'Motor_Control_Blockset'};
installedProducts = ver;
productTemplate = struct('Name', '', 'Version', '', 'Installed', false, ...
    'LicenseFeature', '', 'LicenseAvailable', false);
report.RequiredProducts = repmat(productTemplate, numel(requiredNames), 1);
installedNames = {installedProducts.Name};
for productIndex = 1:numel(requiredNames)
    productName = requiredNames{productIndex};
    installedIndex = find(strcmpi(installedNames, productName), 1);
    report.RequiredProducts(productIndex).Name = productName;
    report.RequiredProducts(productIndex).LicenseFeature = ...
        licenseFeatures{productIndex};
    if ~isempty(installedIndex)
        report.RequiredProducts(productIndex).Installed = true;
        report.RequiredProducts(productIndex).Version = ...
            installedProducts(installedIndex).Version;
    end
    try
        report.RequiredProducts(productIndex).LicenseAvailable = ...
            logical(license('test', licenseFeatures{productIndex}));
    catch
        report.RequiredProducts(productIndex).LicenseAvailable = false;
    end
end

[gitStatus, gitCommit] = system(sprintf( ...
    'git -C "%s" rev-parse --verify HEAD', versionDirectory));
if gitStatus == 0
    report.GitCommit = strtrim(gitCommit);
else
    report.GitCommit = '<unavailable>';
end
[statusCommandResult, statusText] = system(sprintf( ...
    'git -C "%s" status --porcelain --untracked-files=normal', ...
    versionDirectory));
if statusCommandResult == 0
    report.GitStatus = strtrim(statusText);
    report.GitWorkingTreeClean = isempty(report.GitStatus);
    if report.GitWorkingTreeClean
        report.GitStatusEntryCount = 0;
    else
        report.GitStatusEntryCount = numel(regexp( ...
            report.GitStatus, '\r?\n', 'split'));
    end
else
    report.GitStatus = '<unavailable>';
    report.GitWorkingTreeClean = false;
    report.GitStatusEntryCount = -1;
end
end

function verifyRequiredProducts(environmentReport)
missingProducts = {environmentReport.RequiredProducts( ...
    ~[environmentReport.RequiredProducts.Installed]).Name};
assert(isempty(missingProducts), 'Missing required MATLAB products: %s', ...
    strjoin(missingProducts, ', '));
end

function report = createBuildRunReport(controllerName, harnessName, options, ...
        environmentReport, buildStartedAt, buildCompletedAt, buildDuration, ...
        nativeMetrics, mcbMetrics, nativePass, mcbPass, architectureReport, ...
        codeReport, stateflowReport, calibrationReport, faultReport, ...
        fastFaultReport, faultMatrixReport, faultResetReport)
report.BuildStartedAt = buildStartedAt;
report.BuildCompletedAt = buildCompletedAt;
report.BuildDurationSeconds = buildDuration;
report.CleanBuild = options.CleanBuild;
report.BatchMode = options.BatchMode;
report.ExportImages = options.ExportImages;
report.Environment = environmentReport;
report.ControllerChecksum = modelChecksum(controllerName);
report.HarnessChecksum = modelChecksum(harnessName);
report.ControllerConfiguration = captureModelConfiguration(controllerName);
report.HarnessConfiguration = captureModelConfiguration(harnessName);
report.ControllerInputCount = architectureReport.ControllerInputCount;
report.ControllerOutputCount = architectureReport.ControllerOutputCount;
report.ComponentCount = architectureReport.ComponentCount;
report.StateflowStateCount = architectureReport.StateflowStateCount;
report.ControllerDanglingLines = architectureReport.ControllerDanglingLines;
report.HarnessDanglingLines = architectureReport.HarnessDanglingLines;
report.InterfaceSignature = sprintf('%dBI-%dBO-%dC-%dS', ...
    report.ControllerInputCount, report.ControllerOutputCount, ...
    report.ComponentCount, report.StateflowStateCount);
report.NativeMetrics = compactMetrics(nativeMetrics);
report.McbMetrics = compactMetrics(mcbMetrics);
report.NativePass = nativePass;
report.McbPass = mcbPass;
report.ArchitecturePass = architectureReport.Pass;
report.CodePass = codeReport.Pass;
report.StateflowPass = stateflowReport.Pass;
report.CalibrationPass = calibrationReport.Pass;
report.FaultPass = faultReport.Pass;
report.FastFaultPass = fastFaultReport.Pass;
report.FaultMatrixPass = faultMatrixReport.Pass;
report.FaultResetPass = faultResetReport.Pass;
report.OverallPass = report.NativePass && report.McbPass && ...
    report.ArchitecturePass && report.CodePass && report.StateflowPass && ...
    report.CalibrationPass && report.FaultPass && report.FastFaultPass && ...
    report.FaultMatrixPass && report.FaultResetPass;
end

function result = compactMetrics(metrics)
result.FinalSpeedRpm = metrics.FinalSpeedRpm;
result.MaximumSpeedRpm = metrics.MaximumSpeedRpm;
result.MinimumSpeedRpm = metrics.MinimumSpeedRpm;
result.MaximumAbsIqA = metrics.MaximumAbsIqA;
result.MinimumDuty = metrics.MinimumDuty;
result.MaximumDuty = metrics.MaximumDuty;
result.FinalTorqueNm = metrics.FinalTorqueNm;
result.Finite = metrics.Finite;
end

function value = modelChecksum(modelName)
checksum = Simulink.BlockDiagram.getChecksum(modelName);
if isstruct(checksum) && isfield(checksum, 'Value')
    checksumValue = checksum.Value;
else
    checksumValue = checksum;
end
if isnumeric(checksumValue)
    value = char(join(compose('%08X', checksumValue), ''));
else
    value = char(string(checksumValue));
end
end

function configuration = captureModelConfiguration(modelName)
parameterNames = {'SolverType', 'Solver', 'FixedStep', ...
    'SystemTargetFile', 'ProdHWDeviceType', 'CodeInterfacePackaging', ...
    'MultiTasking', 'GenerateReport', 'Toolchain'};
for parameterIndex = 1:numel(parameterNames)
    parameterName = parameterNames{parameterIndex};
    try
        configuration.(parameterName) = valueToText( ...
            get_param(modelName, parameterName));
    catch
        configuration.(parameterName) = '<unavailable>';
    end
end
end

function textValue = valueToText(value)
if ischar(value)
    textValue = value;
elseif isstring(value)
    textValue = char(join(value, ','));
elseif isnumeric(value) || islogical(value)
    textValue = mat2str(value);
else
    textValue = char(string(value));
end
end

function configureDualPlantHarness(modelName, modelFile, versionDirectory)
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_PLANT_SELECTION', 2);

variantPath = [modelName '/Selectable_PMSM_Plant'];
oldPlantPath = [modelName '/Native_Discrete_PMSM_Plant'];
if getSimulinkBlockHandle(variantPath) == -1
    assert(getSimulinkBlockHandle(oldPlantPath) ~= -1, ...
        'The source native PMSM subsystem is missing.');
    nativeChoiceHandle = get_param(oldPlantPath, 'Handle');

    % Convert the already-connected native plant in place. This preserves
    % its 3-input/7-output interface and all external signal lines, avoiding
    % the one-input/one-output interface of a fresh library VSS block.
    Simulink.VariantUtils.convertToVariantSubsystem(nativeChoiceHandle);
    variantHandle = get_param(get_param(nativeChoiceHandle, 'Parent'), 'Handle');
    set_param(variantHandle, 'Name', 'Selectable_PMSM_Plant');
    variantPath = getfullname(variantHandle);
    set_param(nativeChoiceHandle, 'Name', 'Native_Discrete_PMSM', ...
        'VariantControl', ...
        'PMSM_PLANT_SELECTION == 1');

    mcbChoicePath = [variantPath '/MathWorks_MCB_PMSM_HDL'];
    add_block('simulink/Ports & Subsystems/Subsystem', mcbChoicePath, ...
        'Position', [120 360 420 580]);
    buildMcbPmsmWrapper(mcbChoicePath);
    set_param(mcbChoicePath, 'VariantControl', ...
        'PMSM_PLANT_SELECTION == 2');

end

% convertToVariantSubsystem creates label-mode variants. Enforce expression
% mode on every run so PMSM_PLANT_SELECTION genuinely selects the plant.
set_param([variantPath '/Native_Discrete_PMSM'], 'VariantControl', ...
    'PMSM_PLANT_SELECTION == 1');
set_param([variantPath '/MathWorks_MCB_PMSM_HDL'], 'VariantControl', ...
    'PMSM_PLANT_SELECTION == 2');
set_param(variantPath, 'VariantControlMode', 'expression');
% Refresh only after both expressions and the control mode are valid. A
% clean v2.0.0 conversion otherwise tries to update a deleted label choice.
set_param(modelName, 'SimulationCommand', 'update');

buttonPath = [modelName '/Switch_PMSM_Plant'];
if getSimulinkBlockHandle(buttonPath) == -1
    add_block('simulink_hmi_blocks/Callback Button', buttonPath, ...
        'Position', [1240 100 1570 165]);
end
configureCallbackButton(buttonPath, versionDirectory);
removeLegacyOneClickAnnotation(modelName);

set_param(modelName, 'StopTime', '2.0');
set_param(modelName, 'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '0.0001');
loadTorquePath = [modelName '/Load_Torque_Nm'];
if getSimulinkBlockHandle(loadTorquePath) ~= -1
    set_param(loadTorquePath, 'After', 'single(0.2)');
end
% Do not compile here: an older on-disk Stateflow integration may be loaded
% briefly and is rebuilt immediately below by the refactor step.
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_PLANT_SELECTION', 2);
save_system(modelName, modelFile);
end

function buildMcbPmsmWrapper(parent)
Simulink.SubSystem.deleteContents(parent);
inputs = {'VAlpha', 'VBeta', 'LoadTorque'};
outputs = {'SpeedRpm', 'ThetaElectrical', 'Ia', 'Ib', ...
    'TorqueNm', 'Id', 'Iq'};
for index = 1:numel(inputs)
    add_block('simulink/Sources/In1', [parent '/' inputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [20 50+70*index 50 64+70*index]);
end
for index = 1:numel(outputs)
    add_block('simulink/Sinks/Out1', [parent '/' outputs{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [1110 25+55*index 1140 39+55*index]);
end

add_block('mcbhdlplantlib/PMSM Configuration', [parent '/PMSM_Configuration'], ...
    'portConfig', 'Torque', 'Trq_N_mode', '0', 'Ts', '0.0001', ...
    'Pp', '4', 'Rs', '0.4', 'Ld', '0.001', 'Lq', '0.001', ...
    'lambda_pm', '0.05', 'J', '0.002', 'B', '0.0001', 'f', '0', ...
    'Position', [100 300 260 380]);
add_block('mcbhdlplantlib/PMSM HDL', [parent '/MathWorks_PMSM_HDL'], ...
    'Ts', '0.0001', 'Position', [520 250 700 410]);

addGain(parent, 'Vb_Alpha', 'single(-0.5)', [100 100 165 130]);
addGain(parent, 'Vb_Beta', 'single(0.8660254037844386)', [100 145 205 175]);
addSum(parent, 'Phase_Vb', '++', [245 110 275 165]);
addGain(parent, 'Vc_Alpha', 'single(-0.5)', [100 200 165 230]);
addGain(parent, 'Vc_Beta', 'single(-0.8660254037844386)', [100 245 210 275]);
addSum(parent, 'Phase_Vc', '++', [245 210 275 265]);
add_block('simulink/Signal Routing/Mux', [parent '/Phase_Voltage_ABC'], ...
    'Inputs', '3', 'Position', [350 115 355 255]);
add_block('simulink/Sources/Constant', [parent '/Reset_False'], ...
    'Value', 'false', 'OutDataTypeStr', 'boolean', ...
    'Position', [350 430 430 460]);

add_block('simulink/Sinks/Terminator', [parent '/Info_Terminator'], ...
    'Position', [750 235 770 255]);
add_block('simulink/Signal Routing/Demux', [parent '/Phase_Current_Demux'], ...
    'Outputs', '3', 'Position', [755 285 760 365]);
addGain(parent, 'Speed_RadToRpm', 'single(9.549296585513721)', ...
    [755 405 855 435]);

addTrig(parent, 'SinTheta', 'sin', [815 345 855 375]);
addTrig(parent, 'CosTheta', 'cos', [815 385 855 415]);
addGain(parent, 'Ib_x2', 'single(2.0)', [815 270 875 300]);
addSum(parent, 'Clarke_Sum', '++', [910 245 940 305]);
addGain(parent, 'InvSqrt3', 'single(0.5773502691896258)', ...
    [970 260 1060 290]);
addProduct(parent, 'Id_CosAlpha', '**', [880 480 915 510]);
addProduct(parent, 'Id_SinBeta', '**', [880 525 915 555]);
addSum(parent, 'Id_Sum', '++', [950 490 980 550]);
addProduct(parent, 'Iq_SinAlpha', '**', [880 580 915 610]);
addGain(parent, 'Iq_Negative', 'single(-1.0)', [940 580 1000 610]);
addProduct(parent, 'Iq_CosBeta', '**', [880 625 915 655]);
addSum(parent, 'Iq_Sum', '++', [1030 590 1060 650]);

conversionNames = {'Speed_ToSingle', 'Theta_ToSingle', 'Ia_ToSingle', ...
    'Ib_ToSingle', 'Torque_ToSingle', 'Id_ToSingle', 'Iq_ToSingle'};
conversionY = [80 135 190 245 300 520 620];
for index = 1:numel(conversionNames)
    add_block('simulink/Signal Attributes/Data Type Conversion', ...
        [parent '/' conversionNames{index}], 'OutDataTypeStr', 'single', ...
        'Position', [1000 conversionY(index) 1060 conversionY(index)+30]);
end

add_line(parent, 'PMSM_Configuration/1', 'MathWorks_PMSM_HDL/1', 'autorouting', 'on');
add_line(parent, 'VAlpha/1', 'Vb_Alpha/1', 'autorouting', 'on');
add_line(parent, 'VAlpha/1', 'Vc_Alpha/1', 'autorouting', 'on');
add_line(parent, 'VBeta/1', 'Vb_Beta/1', 'autorouting', 'on');
add_line(parent, 'VBeta/1', 'Vc_Beta/1', 'autorouting', 'on');
add_line(parent, 'Vb_Alpha/1', 'Phase_Vb/1', 'autorouting', 'on');
add_line(parent, 'Vb_Beta/1', 'Phase_Vb/2', 'autorouting', 'on');
add_line(parent, 'Vc_Alpha/1', 'Phase_Vc/1', 'autorouting', 'on');
add_line(parent, 'Vc_Beta/1', 'Phase_Vc/2', 'autorouting', 'on');
add_line(parent, 'VAlpha/1', 'Phase_Voltage_ABC/1', 'autorouting', 'on');
add_line(parent, 'Phase_Vb/1', 'Phase_Voltage_ABC/2', 'autorouting', 'on');
add_line(parent, 'Phase_Vc/1', 'Phase_Voltage_ABC/3', 'autorouting', 'on');
add_line(parent, 'Phase_Voltage_ABC/1', 'MathWorks_PMSM_HDL/2', 'autorouting', 'on');
add_line(parent, 'LoadTorque/1', 'MathWorks_PMSM_HDL/3', 'autorouting', 'on');
add_line(parent, 'Reset_False/1', 'MathWorks_PMSM_HDL/4', 'autorouting', 'on');

add_line(parent, 'MathWorks_PMSM_HDL/1', 'Info_Terminator/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/2', 'Phase_Current_Demux/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/3', 'Torque_ToSingle/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/4', 'Speed_RadToRpm/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/5', 'Theta_ToSingle/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/5', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'MathWorks_PMSM_HDL/5', 'CosTheta/1', 'autorouting', 'on');
add_line(parent, 'Speed_RadToRpm/1', 'Speed_ToSingle/1', 'autorouting', 'on');
add_line(parent, 'Phase_Current_Demux/1', 'Ia_ToSingle/1', 'autorouting', 'on');
add_line(parent, 'Phase_Current_Demux/2', 'Ib_ToSingle/1', 'autorouting', 'on');

add_line(parent, 'Phase_Current_Demux/1', 'Clarke_Sum/1', 'autorouting', 'on');
add_line(parent, 'Phase_Current_Demux/2', 'Ib_x2/1', 'autorouting', 'on');
add_line(parent, 'Ib_x2/1', 'Clarke_Sum/2', 'autorouting', 'on');
add_line(parent, 'Clarke_Sum/1', 'InvSqrt3/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Id_CosAlpha/1', 'autorouting', 'on');
add_line(parent, 'Phase_Current_Demux/1', 'Id_CosAlpha/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Id_SinBeta/1', 'autorouting', 'on');
add_line(parent, 'InvSqrt3/1', 'Id_SinBeta/2', 'autorouting', 'on');
add_line(parent, 'Id_CosAlpha/1', 'Id_Sum/1', 'autorouting', 'on');
add_line(parent, 'Id_SinBeta/1', 'Id_Sum/2', 'autorouting', 'on');
add_line(parent, 'Id_Sum/1', 'Id_ToSingle/1', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Iq_SinAlpha/1', 'autorouting', 'on');
add_line(parent, 'Phase_Current_Demux/1', 'Iq_SinAlpha/2', 'autorouting', 'on');
add_line(parent, 'Iq_SinAlpha/1', 'Iq_Negative/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Iq_CosBeta/1', 'autorouting', 'on');
add_line(parent, 'InvSqrt3/1', 'Iq_CosBeta/2', 'autorouting', 'on');
add_line(parent, 'Iq_Negative/1', 'Iq_Sum/1', 'autorouting', 'on');
add_line(parent, 'Iq_CosBeta/1', 'Iq_Sum/2', 'autorouting', 'on');
add_line(parent, 'Iq_Sum/1', 'Iq_ToSingle/1', 'autorouting', 'on');

for index = 1:numel(outputs)
    add_line(parent, [conversionNames{index} '/1'], ...
        [outputs{index} '/1'], 'autorouting', 'on');
end
Simulink.BlockDiagram.arrangeSystem(parent);
end

function configureCallbackButton(buttonPath, versionDirectory)
configuration = jsondecode(get_param(buttonPath, 'Configuration'));
callbackCode = ['run(fullfile(''' strrep(versionDirectory, '''', '''''') ...
    ''',''switch_pmsm_plant_v21.m''))'];
for componentIndex = 1:numel(configuration.components)
    if strcmp(configuration.components(componentIndex).name, 'ButtonStateComponent')
        configuration.components(componentIndex).settings.clickFcn = callbackCode;
        states = configuration.components(componentIndex).settings.states;
        for stateIndex = 1:numel(states)
            states(stateIndex).label.text.content = ...
                'Active: MathWorks MCB PMSM HDL (click to switch)';
        end
        configuration.components(componentIndex).settings.states = states;
    end
end
set_param(buttonPath, 'Configuration', jsonencode(configuration));
end

function removeLegacyOneClickAnnotation(modelName)
marker = 'ONE-CLICK PMSM PLANT SWITCH';
annotationHandles = find_system(modelName, 'FindAll', 'on', ...
    'MatchFilter', @Simulink.match.allVariants, 'Type', 'annotation');
for annotationIndex = 1:numel(annotationHandles)
    candidate = get_param(annotationHandles(annotationIndex), 'Object');
    if contains(candidate.Text, marker)
        delete(candidate);
    end
end
end

function simulationOutput = simulatePlant(modelName, selection)
resetHarnessStimuli(modelName);
setPlantSelection(modelName, selection);
set_param(modelName, 'SimulationCommand', 'update');
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
end

function simulationOutput = simulateCalibrationOffset(modelName)
% Inject known sensor offsets only in the harness and verify that the
% dedicated 100-sample calibration component estimates and removes them.
resetHarnessStimuli(modelName);
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_A', single(0.75));
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_B', single(-0.50));
stimulusCleanup = onCleanup(@() resetHarnessStimuli(modelName));
setPlantSelection(modelName, 1);
simulationOutput = sim(modelName, 'StopTime', '0.085', ...
    'ReturnWorkspaceOutputs', 'on');
end

function simulationOutput = simulateOvervoltageFault(modelName)
% Inject a 70 V DC-bus fault without persisting the test stimulus.
resetHarnessStimuli(modelName);
setPlantSelection(modelName, 1);
dcBusPath = [modelName '/DC_Bus_48V'];
nominalValue = get_param(dcBusPath, 'Value');
restoreCleanup = onCleanup(@() set_param(dcBusPath, 'Value', nominalValue));
set_param(dcBusPath, 'Value', 'single(70.0)');
simulationOutput = sim(modelName, 'StopTime', '0.02', ...
    'ReturnWorkspaceOutputs', 'on');
end

function simulationOutput = simulateFastOvercurrent(modelName)
% Apply a two-fast-tick, 20 A pulse while RUN is active. The final PWM
% permit must fall in the same 100 us scheduling interval, independently of
% the 1 ms Stateflow supervisor reaction.
resetHarnessStimuli(modelName);
modelWorkspace = get_param(modelName, 'ModelWorkspace');
overcurrentSignal = timeseries(single([0.0; 20.0; 0.0]), ...
    [0.0; 0.0903; 0.0905]);
assignin(modelWorkspace, 'PMSM_FAST_OVERCURRENT_SIGNAL', overcurrentSignal);
stimulusCleanup = onCleanup(@() resetHarnessStimuli(modelName));
setPlantSelection(modelName, 1);
simulationOutput = sim(modelName, 'StopTime', '0.095', ...
    'ReturnWorkspaceOutputs', 'on');
end

function report = verifyFaultLatchFromEveryState(modelName)
% Fault pulses clear before each simulation ends and no acknowledgement is
% supplied. Remaining in FAULT therefore verifies real latching.
targetStates = 1:5;
injectionTimes = [0.0 0.002 0.055 0.065 0.090];
stateNames = {'INIT','READY','CALIB','ALIGN','RUN'};
scenarioReports = repmat(struct( ...
    'TargetState', 0, 'TargetName', '', 'InjectionTimeS', 0, ...
    'VisitedStates', [], 'FinalState', 0, 'FinalFaultDetected', true, ...
    'FinalPwmEnable', true, 'FinalDuty', NaN, 'Pass', false), ...
    1, numel(targetStates));
for scenarioIndex = 1:numel(targetStates)
    injectionTime = injectionTimes(scenarioIndex);
    clearTime = injectionTime + 0.002;
    stopTime = max(clearTime + 0.006, 0.01);
    output = simulateFaultPulse(modelName, injectionTime, clearTime, ...
        [], stopTime);
    scenarioReports(scenarioIndex) = collectLatchedFaultMetrics(output, ...
        targetStates(scenarioIndex), stateNames{scenarioIndex}, ...
        injectionTime);
end
report.Scenarios = scenarioReports;
report.Pass = all([scenarioReports.Pass]);
end

function simulationOutput = simulateAcknowledgedFaultReset(modelName)
% The fault clears at 4 ms, but the state remains latched until a distinct
% reset acknowledgement pulse at 8 ms.
simulationOutput = simulateFaultPulse(modelName, 0.002, 0.004, ...
    [0.008 0.0095], 0.014);
end

function simulationOutput = simulateFaultPulse(modelName, injectionTime, ...
        clearTime, acknowledgementWindow, stopTime)
resetHarnessStimuli(modelName);
modelWorkspace = get_param(modelName, 'ModelWorkspace');
if injectionTime <= 0.0
    faultSignal = timeseries(logical([1; 0]), [0.0; clearTime]);
else
    faultSignal = timeseries(logical([0; 1; 0]), ...
        [0.0; injectionTime; clearTime]);
end
assignin(modelWorkspace, 'PMSM_DRIVER_FAULT_SIGNAL', faultSignal);
if isempty(acknowledgementWindow)
    ackSignal = timeseries(false, 0.0);
    stopSignal = timeseries(false, 0.0);
else
    ackSignal = timeseries(logical([0; 1; 0]), ...
        [0.0; acknowledgementWindow(1); acknowledgementWindow(2)]);
    stopSignal = timeseries(logical([0; 1]), ...
        [0.0; max(0.0, acknowledgementWindow(1) - 0.001)]);
end
assignin(modelWorkspace, 'PMSM_FAULT_RESET_REQUEST_SIGNAL', ackSignal);
assignin(modelWorkspace, 'PMSM_STOP_REQUEST_SIGNAL', stopSignal);
stimulusCleanup = onCleanup(@() resetHarnessStimuli(modelName));
setPlantSelection(modelName, 1);
simulationOutput = sim(modelName, 'StopTime', num2str(stopTime, '%.9g'), ...
    'ReturnWorkspaceOutputs', 'on');
end

function resetHarnessStimuli(modelName)
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_A', single(0.0));
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_B', single(0.0));
assignin(modelWorkspace, 'PMSM_FAST_OVERCURRENT_SIGNAL', ...
    timeseries(single(0.0), 0.0));
assignin(modelWorkspace, 'PMSM_START_REQUEST_SIGNAL', ...
    timeseries(logical([0; 1]), [0.0; 0.05]));
assignin(modelWorkspace, 'PMSM_STOP_REQUEST_SIGNAL', timeseries(false, 0.0));
assignin(modelWorkspace, 'PMSM_EMERGENCY_STOP_SIGNAL', timeseries(false, 0.0));
assignin(modelWorkspace, 'PMSM_DRIVER_FAULT_SIGNAL', timeseries(false, 0.0));
assignin(modelWorkspace, 'PMSM_FAULT_RESET_REQUEST_SIGNAL', ...
    timeseries(false, 0.0));
assignin(modelWorkspace, 'PMSM_HARDWARE_GATE_SIGNAL', timeseries(true, 0.0));
assignin(modelWorkspace, 'PMSM_MEASUREMENT_VALID_SIGNAL', timeseries(true, 0.0));
end

function report = collectStateflowMetrics(simulationOutput)
report.StateSeries = simulationOutput.get('motor_state_code');
report.ControlEnableSeries = simulationOutput.get('motor_control_enable');
report.PwmEnableSeries = simulationOutput.get('motor_pwm_enable');
report.CalibrationEnableSeries = simulationOutput.get('motor_calibration_enable');
report.CalibrationDoneSeries = simulationOutput.get('motor_calibration_done');
report.AlignmentEnableSeries = simulationOutput.get('motor_alignment_enable');
report.AppliedVdSeries = simulationOutput.get('motor_applied_vd');
report.AppliedVqSeries = simulationOutput.get('motor_applied_vq');
stateValues = double(report.StateSeries.Data);
report.VisitedStates = unique(stateValues(:))';
runIndex = find(stateValues == 5, 1, 'first');
if isempty(runIndex)
    report.RunEntryTimeS = NaN;
else
    report.RunEntryTimeS = double(report.StateSeries.Time(runIndex));
end
report.FinalState = stateValues(end);
report.FinalControlEnable = logical(report.ControlEnableSeries.Data(end));
report.FinalPwmEnable = logical(report.PwmEnableSeries.Data(end));
calibrationIndex = stateValues == 3;
alignmentIndex = stateValues == 4;
alignmentFirstIndex = find(alignmentIndex, 1, 'first');
alignmentCommandIndex = false(size(report.AlignmentEnableSeries.Time));
alignmentPwmIndex = false(size(report.PwmEnableSeries.Time));
alignmentVdIndex = false(size(report.AppliedVdSeries.Time));
alignmentVqIndex = false(size(report.AppliedVqSeries.Time));
if ~isempty(alignmentFirstIndex)
    alignmentFirstTime = double(report.StateSeries.Time(alignmentFirstIndex));
    runFirstIndex = find(stateValues == 5, 1, 'first');
    if isempty(runFirstIndex)
        alignmentEndTime = double(report.StateSeries.Time(end)) + 0.001;
    else
        alignmentEndTime = double(report.StateSeries.Time(runFirstIndex));
    end
    alignmentCommandIndex = report.AlignmentEnableSeries.Time >= ...
        alignmentFirstTime & report.AlignmentEnableSeries.Time < alignmentEndTime;
    alignmentPwmIndex = report.PwmEnableSeries.Time >= ...
        alignmentFirstTime + 0.0001 - eps & ...
        report.PwmEnableSeries.Time < alignmentEndTime - eps;
    alignmentVdIndex = report.AppliedVdSeries.Time >= ...
        alignmentFirstTime + 0.0001 - eps & ...
        report.AppliedVdSeries.Time < alignmentEndTime - eps;
    alignmentVqIndex = report.AppliedVqSeries.Time >= ...
        alignmentFirstTime + 0.0001 - eps & ...
        report.AppliedVqSeries.Time < alignmentEndTime - eps;
end
report.CalibrationPass = any(calibrationIndex) && ...
    all(logical(report.CalibrationEnableSeries.Data(calibrationIndex)));
report.AlignmentPass = any(alignmentPwmIndex) && ...
    all(logical(report.AlignmentEnableSeries.Data(alignmentCommandIndex))) && ...
    all(logical(report.PwmEnableSeries.Data(alignmentPwmIndex))) && ...
    max(abs(double(report.AppliedVdSeries.Data(alignmentVdIndex)) - 2.0)) < 1e-6 && ...
    max(abs(double(report.AppliedVqSeries.Data(alignmentVqIndex)))) < 1e-6;
report.Pass = isequal(report.VisitedStates, 1:5) && ...
    isfinite(report.RunEntryTimeS) && ...
    report.RunEntryTimeS >= 0.080 && report.RunEntryTimeS <= 0.085 && ...
    report.FinalState == 5 && report.FinalControlEnable && ...
    report.FinalPwmEnable && report.CalibrationPass && report.AlignmentPass;
end

function report = collectCalibrationMetrics(simulationOutput)
offsetA = simulationOutput.get('motor_current_offset_a');
offsetB = simulationOutput.get('motor_current_offset_b');
correctedA = simulationOutput.get('motor_corrected_current_a');
correctedB = simulationOutput.get('motor_corrected_current_b');
calibrationDone = simulationOutput.get('motor_calibration_done');
stateSeries = simulationOutput.get('motor_state_code');
doneIndex = find(logical(calibrationDone.Data), 1, 'first');
if isempty(doneIndex)
    report.CompletionTimeS = NaN;
    report.EstimatedOffsetA = NaN;
    report.EstimatedOffsetB = NaN;
    report.CorrectedCurrentAAtCompletion = NaN;
    report.CorrectedCurrentBAtCompletion = NaN;
else
    report.CompletionTimeS = double(calibrationDone.Time(doneIndex));
    report.EstimatedOffsetA = double(offsetA.Data(doneIndex));
    report.EstimatedOffsetB = double(offsetB.Data(doneIndex));
    report.CorrectedCurrentAAtCompletion = double(correctedA.Data(doneIndex));
    report.CorrectedCurrentBAtCompletion = double(correctedB.Data(doneIndex));
end
report.VisitedStates = unique(double(stateSeries.Data(:)))';
report.Pass = isfinite(report.CompletionTimeS) && ...
    abs(report.EstimatedOffsetA - 0.75) < 1e-4 && ...
    abs(report.EstimatedOffsetB + 0.50) < 1e-4 && ...
    abs(report.CorrectedCurrentAAtCompletion) < 1e-3 && ...
    abs(report.CorrectedCurrentBAtCompletion) < 1e-3 && ...
    any(report.VisitedStates == 4);
end

function report = collectLatchedFaultMetrics(simulationOutput, targetState, ...
        targetName, injectionTime)
stateSeries = simulationOutput.get('motor_state_code');
faultSeries = simulationOutput.get('motor_fault_detected');
pwmSeries = simulationOutput.get('motor_pwm_enable');
dutySeries = simulationOutput.get('native_duty_a');
stateValues = double(stateSeries.Data);
report.TargetState = targetState;
report.TargetName = targetName;
report.InjectionTimeS = injectionTime;
report.VisitedStates = unique(stateValues(:))';
report.FinalState = stateValues(end);
report.FinalFaultDetected = logical(faultSeries.Data(end));
report.FinalPwmEnable = logical(pwmSeries.Data(end));
report.FinalDuty = double(dutySeries.Data(end));
report.Pass = any(report.VisitedStates == targetState) && ...
    any(report.VisitedStates == 6) && report.FinalState == 6 && ...
    report.FinalFaultDetected && ~report.FinalPwmEnable && ...
    abs(report.FinalDuty - 0.5) < 1e-6;
end

function report = collectFaultResetMetrics(simulationOutput)
stateSeries = simulationOutput.get('motor_state_code');
faultSeries = simulationOutput.get('motor_fault_detected');
ackSeries = simulationOutput.get('motor_reset_ack');
stateValues = double(stateSeries.Data);
holdIndex = find(stateSeries.Time >= 0.006, 1, 'first');
postAckIndex = find(stateSeries.Time >= 0.008, 1, 'first');
resetIndex = [];
if ~isempty(postAckIndex)
    relativeIndex = find(stateValues(postAckIndex:end) ~= 6, 1, 'first');
    if ~isempty(relativeIndex)
        resetIndex = postAckIndex + relativeIndex - 1;
    end
end
report.VisitedStates = unique(stateValues(:))';
report.FaultHeldAfterSourceClear = ~isempty(holdIndex) && ...
    stateValues(holdIndex) == 6 && logical(faultSeries.Data(holdIndex));
report.AcknowledgementObserved = any(logical(ackSeries.Data));
if isempty(resetIndex)
    report.ResetExitTimeS = NaN;
else
    report.ResetExitTimeS = double(stateSeries.Time(resetIndex));
end
report.FinalState = stateValues(end);
report.Pass = any(report.VisitedStates == 6) && ...
    report.FaultHeldAfterSourceClear && report.AcknowledgementObserved && ...
    isfinite(report.ResetExitTimeS) && report.ResetExitTimeS >= 0.008 && ...
    report.FinalState == 2;
end

function report = collectFaultMetrics(simulationOutput)
report.StateSeries = simulationOutput.get('motor_state_code');
report.PwmEnableSeries = simulationOutput.get('motor_pwm_enable');
report.DutySeries = simulationOutput.get('native_duty_a');
stateValues = double(report.StateSeries.Data);
report.VisitedStates = unique(stateValues(:))';
faultIndex = find(stateValues == 6, 1, 'first');
if isempty(faultIndex)
    report.FaultEntryTimeS = NaN;
else
    report.FaultEntryTimeS = double(report.StateSeries.Time(faultIndex));
end
report.FinalState = stateValues(end);
report.FinalPwmEnable = logical(report.PwmEnableSeries.Data(end));
report.FinalDuty = double(report.DutySeries.Data(end));
report.Pass = any(report.VisitedStates == 6) && ...
    report.FinalState == 6 && ~report.FinalPwmEnable && ...
    abs(report.FinalDuty - 0.5) < 1e-6;
end

function report = collectFastFaultMetrics(simulationOutput, injectionTime)
stateSeries = simulationOutput.get('motor_state_code');
fastFaultSeries = simulationOutput.get('motor_fast_fault');
pwmSeries = simulationOutput.get('motor_pwm_enable');
dutySeries = simulationOutput.get('native_duty_a');
fastFaultValues = logical(fastFaultSeries.Data);
pwmValues = logical(pwmSeries.Data);
faultIndex = find(fastFaultValues & fastFaultSeries.Time >= injectionTime, ...
    1, 'first');
pwmDisableIndex = find(~pwmValues & pwmSeries.Time >= injectionTime, 1, 'first');
if isempty(faultIndex)
    report.FastFaultEntryTimeS = NaN;
else
    report.FastFaultEntryTimeS = double(fastFaultSeries.Time(faultIndex));
end
if isempty(pwmDisableIndex)
    report.PwmDisableTimeS = NaN;
    report.PwmDisableLatencyS = NaN;
else
    report.PwmDisableTimeS = double(pwmSeries.Time(pwmDisableIndex));
    report.PwmDisableLatencyS = report.PwmDisableTimeS - injectionTime;
end
stateValues = double(stateSeries.Data);
report.VisitedStates = unique(stateValues(:))';
report.FinalState = stateValues(end);
report.FinalFastFault = fastFaultValues(end);
report.FinalPwmEnable = pwmValues(end);
report.FinalDuty = double(dutySeries.Data(end));
report.Pass = isfinite(report.FastFaultEntryTimeS) && ...
    report.FastFaultEntryTimeS <= injectionTime + 0.0001 + eps && ...
    isfinite(report.PwmDisableLatencyS) && ...
    report.PwmDisableLatencyS >= -eps && ...
    report.PwmDisableLatencyS <= 0.0001 + eps && ...
    any(report.VisitedStates == 5) && any(report.VisitedStates == 6) && ...
    report.FinalState == 6 && report.FinalFastFault && ...
    ~report.FinalPwmEnable && abs(report.FinalDuty - 0.5) < 1e-6;
end

function setPlantSelection(modelName, selection)
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_PLANT_SELECTION', selection);
set_param(modelName, 'SimulationCommand', 'update');
end

function metrics = collectMetrics(simulationOutput)
speedSeries = simulationOutput.get('native_speed_rpm');
iqSeries = simulationOutput.get('native_iq_a');
dutySeries = simulationOutput.get('native_duty_a');
torqueSeries = simulationOutput.get('native_torque_nm');
metrics.SpeedSeries = speedSeries;
metrics.IqSeries = iqSeries;
metrics.DutySeries = dutySeries;
metrics.TorqueSeries = torqueSeries;
metrics.FinalSpeedRpm = double(speedSeries.Data(end));
metrics.MaximumSpeedRpm = max(double(speedSeries.Data));
metrics.MinimumSpeedRpm = min(double(speedSeries.Data));
metrics.MaximumAbsIqA = max(abs(double(iqSeries.Data)));
metrics.MinimumDuty = min(double(dutySeries.Data));
metrics.MaximumDuty = max(double(dutySeries.Data));
metrics.FinalTorqueNm = double(torqueSeries.Data(end));
metrics.Finite = all(isfinite(double(speedSeries.Data))) && ...
    all(isfinite(double(iqSeries.Data))) && ...
    all(isfinite(double(dutySeries.Data)));
end

function pass = metricsPass(metrics)
pass = metrics.Finite && metrics.FinalSpeedRpm > 900.0 && ...
    metrics.FinalSpeedRpm < 1100.0 && metrics.MaximumSpeedRpm < 1250.0 && ...
    metrics.MinimumSpeedRpm > -50.0 && metrics.MaximumAbsIqA < 12.0 && ...
    metrics.MinimumDuty >= 0.019 && metrics.MaximumDuty <= 0.981;
end

function createComparisonPlot(nativeOutput, mcbOutput, versionDirectory)
nativeSpeed = nativeOutput.get('native_speed_rpm');
mcbSpeed = mcbOutput.get('native_speed_rpm');
nativeIq = nativeOutput.get('native_iq_a');
mcbIq = mcbOutput.get('native_iq_a');
nativeTorque = nativeOutput.get('native_torque_nm');
mcbTorque = mcbOutput.get('native_torque_nm');
nativeDuty = nativeOutput.get('native_duty_a');
mcbDuty = mcbOutput.get('native_duty_a');
figureHandle = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100 100 1200 820]);
layout = tiledlayout(figureHandle, 2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile(layout);
plot(nativeSpeed.Time, nativeSpeed.Data, 'LineWidth', 1.3);
hold on;
plot(mcbSpeed.Time, mcbSpeed.Data, '--', 'LineWidth', 1.3);
plot([0 0.05 0.05 2], [0 0 1000 1000], ':', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Speed (rpm)');
legend('Native PMSM', 'MathWorks PMSM HDL', 'Reference', 'Location', 'best');

nexttile(layout);
plot(nativeIq.Time, nativeIq.Data, 'LineWidth', 1.3);
hold on;
plot(mcbIq.Time, mcbIq.Data, '--', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Iq (A)');
legend('Native PMSM', 'MathWorks PMSM HDL', 'Location', 'best');

nexttile(layout);
plot(nativeTorque.Time, nativeTorque.Data, 'LineWidth', 1.3);
hold on;
plot(mcbTorque.Time, mcbTorque.Data, '--', 'LineWidth', 1.3);
plot([0 0.5 0.5 2], [0 0 0.2 0.2], ':', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Native PMSM', 'MathWorks PMSM HDL', 'Load torque', 'Location', 'best');

nexttile(layout);
plot(nativeDuty.Time, nativeDuty.Data, 'LineWidth', 1.3);
hold on;
plot(mcbDuty.Time, mcbDuty.Data, '--', 'LineWidth', 1.3);
plot([0 2], [0.02 0.02], ':', 'LineWidth', 0.9);
plot([0 2], [0.98 0.98], ':', 'LineWidth', 0.9);
grid on;
xlabel('Time (s)');
ylabel('Duty A');
ylim([0 1]);
legend('Native PMSM', 'MathWorks PMSM HDL', 'Lower limit', ...
    'Upper limit', 'Location', 'best');

title(layout, 'PMSM FOC dual-plant comparison v2.1.0');
exportgraphics(figureHandle, fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_results.png'), 'Resolution', 160);
close(figureHandle);
end

function createStateflowTestPlot(normalOutput, calibrationOutput, faultOutput, ...
        fastFaultOutput, resetOutput, versionDirectory)
normalState = normalOutput.get('motor_state_code');
normalControl = normalOutput.get('motor_control_enable');
normalPwm = normalOutput.get('motor_pwm_enable');
normalCalibration = normalOutput.get('motor_calibration_enable');
normalAlignment = normalOutput.get('motor_alignment_enable');
calibrationOffsetA = calibrationOutput.get('motor_current_offset_a');
calibrationOffsetB = calibrationOutput.get('motor_current_offset_b');
calibrationCorrectedA = calibrationOutput.get('motor_corrected_current_a');
calibrationCorrectedB = calibrationOutput.get('motor_corrected_current_b');
faultState = faultOutput.get('motor_state_code');
faultPwm = faultOutput.get('motor_pwm_enable');
faultDuty = faultOutput.get('native_duty_a');
fastFaultState = fastFaultOutput.get('motor_state_code');
fastFaultSignal = fastFaultOutput.get('motor_fast_fault');
fastFaultPwm = fastFaultOutput.get('motor_pwm_enable');
resetState = resetOutput.get('motor_state_code');
resetFault = resetOutput.get('motor_fault_detected');
resetAck = resetOutput.get('motor_reset_ack');

figureHandle = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100 100 1280 1250]);
layout = tiledlayout(figureHandle, 5, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile(layout);
stairs(normalState.Time, double(normalState.Data), 'LineWidth', 1.5);
hold on;
stairs(normalControl.Time, 6.3 * double(normalControl.Data), '--', ...
    'LineWidth', 1.2);
stairs(normalPwm.Time, 6.0 * double(normalPwm.Data), ':', ...
    'LineWidth', 1.2);
stairs(normalCalibration.Time, 5.7 * double(normalCalibration.Data), '-.', ...
    'LineWidth', 1.1);
stairs(normalAlignment.Time, 5.4 * double(normalAlignment.Data), '--', ...
    'LineWidth', 1.1);
grid on;
xlim([0 0.12]);
ylim([0.5 6.7]);
yticks(1:6);
yticklabels({'INIT','READY','CALIB','ALIGN','RUN','FAULT'});
xlabel('Time (s)');
ylabel('Motor state');
legend('State code', 'Control enable', 'PWM enable', ...
    'Calibration enable', 'Alignment enable', ...
    'Location', 'eastoutside');
title('Normal startup: CALIB averaging, ALIGN voltage application, then RUN');

nexttile(layout);
plot(calibrationOffsetA.Time, calibrationOffsetA.Data, 'LineWidth', 1.3);
hold on;
plot(calibrationOffsetB.Time, calibrationOffsetB.Data, 'LineWidth', 1.3);
plot(calibrationCorrectedA.Time, calibrationCorrectedA.Data, '--', ...
    'LineWidth', 1.0);
plot(calibrationCorrectedB.Time, calibrationCorrectedB.Data, '--', ...
    'LineWidth', 1.0);
grid on;
xlim([0.048 0.065]);
xlabel('Time (s)');
ylabel('Current / offset (A)');
legend('Estimated offset A', 'Estimated offset B', ...
    'Corrected current A', 'Corrected current B', ...
    'Location', 'eastoutside');
title('100-sample current-offset calibration: injected +0.75 A / -0.50 A');

nexttile(layout);
stairs(faultState.Time, double(faultState.Data), 'LineWidth', 1.5);
hold on;
stairs(faultPwm.Time, 6.0 * double(faultPwm.Data), '--', ...
    'LineWidth', 1.2);
stairs(faultDuty.Time, 2.0 * double(faultDuty.Data), ':', ...
    'LineWidth', 1.2);
grid on;
xlim([0 0.02]);
ylim([0.5 6.7]);
yticks(1:6);
yticklabels({'INIT','READY','CALIB','ALIGN','RUN','FAULT'});
xlabel('Time (s)');
ylabel('Motor state');
legend('State code', 'PWM enable (scaled)', 'Duty A x 2', ...
    'Location', 'eastoutside');
title('70 V overvoltage injection: FAULT with PWM disabled and 50% safe duty');

nexttile(layout);
stairs(fastFaultState.Time, double(fastFaultState.Data), 'LineWidth', 1.5);
hold on;
stairs(fastFaultSignal.Time, 5.8 * double(fastFaultSignal.Data), '--', ...
    'LineWidth', 1.2);
stairs(fastFaultPwm.Time, 5.4 * double(fastFaultPwm.Data), ':', ...
    'LineWidth', 1.2);
grid on;
xlim([0.0895 0.0925]);
ylim([0.5 6.3]);
yticks(1:6);
yticklabels({'INIT','READY','CALIB','ALIGN','RUN','FAULT'});
xlabel('Time (s)');
ylabel('Motor state');
legend('State code', 'Fast overcurrent latch (scaled)', ...
    'Actual PWM permit (scaled)', 'Location', 'eastoutside');
title('20 A pulse: 100 us fast gate inhibits PWM before 1 ms supervisor update');

nexttile(layout);
stairs(resetState.Time, double(resetState.Data), 'LineWidth', 1.5);
hold on;
stairs(resetFault.Time, 5.8 * double(resetFault.Data), '--', ...
    'LineWidth', 1.2);
stairs(resetAck.Time, 5.4 * squeeze(double(resetAck.Data)), ':', ...
    'LineWidth', 1.2);
grid on;
xlim([0 0.014]);
ylim([0.5 6.3]);
yticks(1:6);
yticklabels({'INIT','READY','CALIB','ALIGN','RUN','FAULT'});
xlabel('Time (s)');
ylabel('Motor state');
legend('State code', 'Fault source (scaled)', 'Reset acknowledgement (scaled)', ...
    'Location', 'eastoutside');
title('FAULT remains latched after source clears; explicit acknowledgement releases it');

title(layout, 'PMSM FOC Stateflow verification v2.1.0');
exportgraphics(figureHandle, fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_stateflow_results.png'), 'Resolution', 180);
close(figureHandle);
end

function exportArchitectureImages(controllerName, modelName, versionDirectory)
overviewPath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_architecture.png');
plantPath = [modelName '/Selectable_PMSM_Plant'];
plantImagePath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_plant_variants.png');
controllerImagePath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_controller_architecture.png');
stateflowPath = [modelName '/Motor_Supervisor_1ms'];
stateflowImagePath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_stateflow_architecture.png');

open_system(modelName);
print(['-s' modelName], '-dpng', '-r160', overviewPath);
open_system(plantPath);
print(['-s' plantPath], '-dpng', '-r160', plantImagePath);
open_system(controllerName);
set_param(controllerName, 'ZoomFactor', 'FitSystem');
print(['-s' controllerName], '-dpng', '-r180', controllerImagePath);
open_system(stateflowPath);
print(['-s' stateflowPath], '-dpng', '-r180', stateflowImagePath);
end

function report = inspectVariantStructure(modelName)
variantPath = [modelName '/Selectable_PMSM_Plant'];
choices = find_system(variantPath, 'SearchDepth', 1, ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem');
choices = setdiff(choices, {variantPath});
report.ChoiceCount = numel(choices);
mcbPath = [variantPath '/MathWorks_MCB_PMSM_HDL/MathWorks_PMSM_HDL'];
report.McbReference = get_param(mcbPath, 'ReferenceBlock');
report.McbReferencePresent = strcmp(report.McbReference, ...
    'mcbhdlplantlib/PMSM HDL');
report.NativeChoicePresent = getSimulinkBlockHandle( ...
    [variantPath '/Native_Discrete_PMSM']) ~= -1;
report.CallbackButtonPresent = getSimulinkBlockHandle( ...
    [modelName '/Switch_PMSM_Plant']) ~= -1;
% The Dashboard button is the one-click control. The legacy rich-text link
% was removed because print() exported its raw HTML across the diagram.
report.OneClickLinkPresent = report.CallbackButtonPresent;
end

function report = inspectGeneratedControllerCode(controllerName, versionDirectory)
codeDirectory = fullfile(versionDirectory, [controllerName '_ert_rtw']);
sourceFile = fullfile(codeDirectory, [controllerName '.c']);
headerFile = fullfile(codeDirectory, [controllerName '.h']);
assert(exist(sourceFile, 'file') ~= 0, 'Generated controller C source is missing.');
assert(exist(headerFile, 'file') ~= 0, 'Generated controller header is missing.');
sourceText = fileread(sourceFile);
headerText = fileread(headerFile);
generatedFiles = [dir(fullfile(codeDirectory, '*.c')); ...
    dir(fullfile(codeDirectory, '*.h'))];
generatedText = '';
for generatedIndex = 1:numel(generatedFiles)
    generatedText = [generatedText newline fileread(fullfile( ...
        generatedFiles(generatedIndex).folder, generatedFiles(generatedIndex).name))]; %#ok<AGROW>
end
report.HasStep = contains(sourceText, [controllerName '_step(void)']);
report.HasInitialize = contains(sourceText, [controllerName '_initialize(void)']);
report.HasInputs = contains(headerText, 'External inputs');
report.HasOutputs = contains(headerText, 'External outputs');
report.HasSFunction = contains(lower(sourceText), 's-function') || ...
    contains(lower(headerText), 's-function');
report.HasStateflow = contains(sourceText, 'Motor_Supervisor_1ms') || ...
    contains(sourceText, 'is_active_');
% Virtual subsystem names are optimized to generated system identifiers
% (for example <S8>/<S15>), so verify their emitted operations instead of
% relying on top-level subsystem names that are not guaranteed in comments.
report.HasFastSafety = contains(sourceText, 'OverCurrentA') && ...
    contains(sourceText, 'OverCurrentB') && ...
    contains(generatedText, 'HardwareGate') && ...
    contains(generatedText, 'DriverFault') && ...
    contains(generatedText, 'FaultBits');
report.HasSlowSafety = contains(sourceText, 'OverSpeed') && ...
    contains(sourceText, 'UnderVoltage') && contains(sourceText, 'OverVoltage');
report.HasDualRateScheduler = contains(sourceText, 'TaskCounters.TID[1]');
report.HasControlCommandBus = contains(generatedText, 'ControlCommandBus');
report.HasMeasurementBus = contains(generatedText, 'MeasurementBus');
report.HasControlStatusBus = contains(generatedText, 'ControlStatusBus');
report.HasBusInterfaceTypes = report.HasControlCommandBus && ...
    report.HasMeasurementBus && report.HasControlStatusBus;
report.SinfCallCount = numel(regexp(sourceText, 'sinf\s*\(', 'match'));
report.CosfCallCount = numel(regexp(sourceText, 'cosf\s*\(', 'match'));
report.Pass = report.HasStep && report.HasInitialize && report.HasInputs && ...
    report.HasOutputs && report.HasStateflow && report.HasFastSafety && ...
    report.HasSlowSafety && report.HasDualRateScheduler && ...
    report.HasBusInterfaceTypes && ...
    report.SinfCallCount == 1 && report.CosfCallCount == 1 && ...
    ~report.HasSFunction;
end

function report = inspectControllerArchitecture(controllerName, harnessName)
componentPaths = {'Command_Arbiter_1ms', 'Motor_Supervisor_1ms', ...
    'Speed_PI_Controller_1ms', ...
    'Current_Offset_Calibration_100us', ...
    'Fast_Protection_100us', 'Fault_Latch_Manager_100us', ...
    'Final_PWM_Arbiter_100us', 'Slow_Safety_Monitor_1ms', ...
    'Clarke_Transform', 'Electrical_Angle_Trig_100us', 'Park_Transform', ...
    'D_Axis_Current_PI', 'Q_Axis_Current_PI', ...
    'DQ_Decoupling_Feedforward', 'DQ_Voltage_Command', ...
    'Alignment_DQ_Override_100us', ...
    'Inverse_Park_Transform', 'Inverse_Clarke_Transform', ...
    'SVPWM_Duty_Calculation'};
report.ComponentCount = numel(componentPaths);
report.ControllerComponentsPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([controllerName '/' path]) ~= -1, componentPaths));
report.HarnessComponentsPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([harnessName '/' path]) ~= -1, componentPaths));
taskBoundaryPaths = {'IqRef_Rate_Transition', 'AnyFault_To_1ms', ...
    'CalibrationDone_To_1ms', 'EmergencyStop_To_1ms', ...
    'DriverFault_To_1ms', 'HardwareGate_To_1ms', ...
    'SupervisorPwmRequest_To_100us', 'CalibrationEnable_To_100us', ...
    'CalibrationReset_To_100us', 'AlignmentEnable_To_100us', ...
    'ControllerReset_To_100us', 'ResetAllowed_To_100us', ...
    'Overspeed_To_100us', 'Undervoltage_To_100us', ...
    'Overvoltage_To_100us'};
report.ControllerTaskBoundariesPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([controllerName '/' path]) ~= -1, ...
    taskBoundaryPaths));
report.HarnessTaskBoundariesPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([harnessName '/' path]) ~= -1, ...
    taskBoundaryPaths));
report.ControllerFastSafetyPresent = getSimulinkBlockHandle( ...
    [controllerName '/Fast_Protection_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [controllerName '/Fault_Latch_Manager_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [controllerName '/Final_PWM_Arbiter_100us']) ~= -1;
report.HarnessFastSafetyPresent = getSimulinkBlockHandle( ...
    [harnessName '/Fast_Protection_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [harnessName '/Fault_Latch_Manager_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [harnessName '/Final_PWM_Arbiter_100us']) ~= -1;
report.ControllerSlowSafetyPresent = getSimulinkBlockHandle( ...
    [controllerName '/Slow_Safety_Monitor_1ms']) ~= -1;
report.HarnessSlowSafetyPresent = getSimulinkBlockHandle( ...
    [harnessName '/Slow_Safety_Monitor_1ms']) ~= -1;
report.ControllerSafetyMonitorPresent = report.ControllerFastSafetyPresent && ...
    report.ControllerSlowSafetyPresent;
report.HarnessSafetyMonitorPresent = report.HarnessFastSafetyPresent && ...
    report.HarnessSlowSafetyPresent;
stateflowRoot = sfroot;
stateflowMachine = stateflowRoot.find('-isa', 'Stateflow.Machine', ...
    'Name', controllerName);
stateflowChart = stateflowMachine.find('-isa', 'Stateflow.Chart', ...
    'Name', 'Motor_Supervisor_1ms');
report.StateflowPresent = ~isempty(stateflowChart);
if report.StateflowPresent
    stateflowChart = stateflowChart(1);
    operationalStateNames = {'INIT','READY','CALIB','ALIGN','RUN','FAULT'};
    report.StateflowStateCount = sum(cellfun(@(stateName) ~isempty( ...
        stateflowChart.find('-isa', 'Stateflow.State', 'Name', stateName)), ...
        operationalStateNames));
    report.StateflowSuperstatePresent = ~isempty(stateflowChart.find( ...
        '-isa', 'Stateflow.State', 'Name', 'SUPERVISED'));
    report.StateflowSampleTime = str2double(stateflowChart.SampleTime);
    report.ResetAckInputPresent = ~isempty(stateflowChart.find( ...
        '-isa', 'Stateflow.Data', 'Name', 'ResetAllowed'));
    report.CalibrationDoneInputPresent = ~isempty(stateflowChart.find( ...
        '-isa', 'Stateflow.Data', 'Name', 'CalibrationDone'));
    report.AlignmentEnableOutputPresent = ~isempty(stateflowChart.find( ...
        '-isa', 'Stateflow.Data', 'Name', 'AlignmentEnable'));
    report.ControllerResetOutputPresent = ~isempty(stateflowChart.find( ...
        '-isa', 'Stateflow.Data', 'Name', 'ControllerReset'));
    report.AcknowledgedFaultTransitionPresent = hasStateflowTransition( ...
        stateflowChart, 'FAULT', 'INIT', 'ResetAllowed');
    report.CalibrationDoneTransitionPresent = hasStateflowTransition( ...
        stateflowChart, 'CALIB', 'ALIGN', 'CalibrationDone');
    report.RunStopToInitPresent = hasStateflowTransition( ...
        stateflowChart, 'RUN', 'INIT', 'StopActive');
    report.AlignStopToInitPresent = hasStateflowTransition( ...
        stateflowChart, 'ALIGN', 'INIT', 'StopActive');
else
    report.StateflowStateCount = 0;
    report.StateflowSuperstatePresent = false;
    report.StateflowSampleTime = NaN;
    report.ResetAckInputPresent = false;
    report.CalibrationDoneInputPresent = false;
    report.AlignmentEnableOutputPresent = false;
    report.ControllerResetOutputPresent = false;
    report.AcknowledgedFaultTransitionPresent = false;
    report.CalibrationDoneTransitionPresent = false;
    report.RunStopToInitPresent = false;
    report.AlignStopToInitPresent = false;
end
controllerInports = find_system(controllerName, 'SearchDepth', 1, ...
    'BlockType', 'Inport');
controllerOutports = find_system(controllerName, 'SearchDepth', 1, ...
    'BlockType', 'Outport');
report.ControllerInputCount = numel(controllerInports);
report.ControllerOutputCount = numel(controllerOutports);
report.LegacyScalarFaultResetPortAbsent = getSimulinkBlockHandle( ...
    [controllerName '/FaultResetAck']) == -1;
report.ControlCommandBusPortPresent = typedRootPortPresent(controllerName, ...
    'ControlCommand', 'Bus: ControlCommandBus');
report.MeasurementBusPortPresent = typedRootPortPresent(controllerName, ...
    'Measurement', 'Bus: MeasurementBus');
report.ControlStatusBusPortPresent = typedRootPortPresent(controllerName, ...
    'ControlStatus', 'Bus: ControlStatusBus');
report.BusInterfacePresent = report.ControlCommandBusPortPresent && ...
    report.MeasurementBusPortPresent && report.ControlStatusBusPortPresent;
[report.DataDictionaryPresent, report.ParameterCatalogCount, ...
    report.InterfaceCatalogCount, report.DataDictionaryVersion, ...
    report.ControlCommandElementsPresent, ...
    report.MeasurementElementsPresent, report.ControlStatusElementsPresent, ...
    report.CalibrationElementsPresent, report.FaultResetRequestElementPresent, ...
    report.MigratedParameterShadowCount] = inspectDataDictionaryContracts( ...
    controllerName, harnessName);
report.DataContractPresent = report.DataDictionaryPresent && ...
    report.ParameterCatalogCount == 30 && report.InterfaceCatalogCount == 41 && ...
    strcmp(report.DataDictionaryVersion, '2.1.0') && ...
    report.ControlCommandElementsPresent && report.MeasurementElementsPresent && ...
    report.ControlStatusElementsPresent && report.CalibrationElementsPresent && ...
    report.FaultResetRequestElementPresent && ...
    report.MigratedParameterShadowCount == 0;
report.CalibrationComponentPresent = getSimulinkBlockHandle( ...
    [controllerName '/Current_Offset_Calibration_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [harnessName '/Current_Offset_Calibration_100us']) ~= -1;
report.AlignmentComponentPresent = getSimulinkBlockHandle( ...
    [controllerName '/Alignment_DQ_Override_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [harnessName '/Alignment_DQ_Override_100us']) ~= -1;
speedPiPorts = get_param([controllerName '/Speed_PI_Controller_1ms'], 'Ports');
dPiPorts = get_param([controllerName '/D_Axis_Current_PI'], 'Ports');
qPiPorts = get_param([controllerName '/Q_Axis_Current_PI'], 'Ports');
report.PiResetPortsPresent = speedPiPorts(1) == 3 && ...
    dPiPorts(1) == 3 && qPiPorts(1) == 3;
report.SpeedTaskSampleTime = str2double(get_param( ...
    [controllerName '/Speed_PI_Controller_1ms/Integrator_State'], 'SampleTime'));
report.CurrentTaskSampleTime = str2double(get_param( ...
    [controllerName '/D_Axis_Current_PI/Integrator_State'], ...
    'SampleTime'));
report.FastSafetySampleTime = str2double(get_param( ...
    [controllerName ...
    '/Fault_Latch_Manager_100us/EmergencyStop_Latch_State'], ...
    'SampleTime'));
report.SlowSafetySampleTime = str2double(get_param( ...
    [controllerName '/Slow_Safety_Monitor_1ms/Speed_Sample_1ms'], ...
    'SampleTime'));
report.SharedTrigPresent = getSimulinkBlockHandle( ...
    [controllerName '/Electrical_Angle_Trig_100us']) ~= -1 && ...
    getSimulinkBlockHandle( ...
    [harnessName '/Electrical_Angle_Trig_100us']) ~= -1;
report.ControllerDanglingLines = countDanglingLines(controllerName);
report.HarnessDanglingLines = countDanglingLines(harnessName);
report.Pass = report.ControllerComponentsPresent && ...
    report.HarnessComponentsPresent && ...
    report.ControllerTaskBoundariesPresent && ...
    report.HarnessTaskBoundariesPresent && ...
    report.ControllerFastSafetyPresent && report.HarnessFastSafetyPresent && ...
    report.ControllerSlowSafetyPresent && report.HarnessSlowSafetyPresent && ...
    report.StateflowPresent && ...
    report.StateflowStateCount == 6 && report.StateflowSuperstatePresent && ...
    report.ResetAckInputPresent && report.CalibrationDoneInputPresent && ...
    report.AlignmentEnableOutputPresent && report.ControllerResetOutputPresent && ...
    report.AcknowledgedFaultTransitionPresent && ...
    report.CalibrationDoneTransitionPresent && report.RunStopToInitPresent && ...
    report.AlignStopToInitPresent && report.ControllerInputCount == 2 && ...
    report.ControllerOutputCount == 1 && report.BusInterfacePresent && ...
    report.LegacyScalarFaultResetPortAbsent && report.DataContractPresent && ...
    report.CalibrationComponentPresent && report.AlignmentComponentPresent && ...
    report.PiResetPortsPresent && report.SharedTrigPresent && ...
    abs(report.StateflowSampleTime - 0.001) < eps && ...
    abs(report.SpeedTaskSampleTime - 0.001) < eps && ...
    abs(report.CurrentTaskSampleTime - 0.0001) < eps && ...
    abs(report.FastSafetySampleTime - 0.0001) < eps && ...
    abs(report.SlowSafetySampleTime - 0.001) < eps && ...
    report.ControllerDanglingLines == 0 && ...
    report.HarnessDanglingLines == 0;
end

function present = typedRootPortPresent(modelName, blockName, dataType)
path = [modelName '/' blockName];
present = getSimulinkBlockHandle(path) ~= -1 && ...
    strcmp(get_param(path, 'OutDataTypeStr'), dataType);
end

function [dictionaryPresent, parameterCount, interfaceCount, version, ...
        commandPresent, measurementPresent, statusPresent, calibrationPresent, ...
        resetPresent, shadowCount] = inspectDataDictionaryContracts( ...
        controllerName, harnessName)
dictionaryName = 'PMSM_FOC_Data.sldd';
dictionaryPresent = endsWith(get_param(controllerName, 'DataDictionary'), ...
    dictionaryName) && endsWith(get_param(harnessName, 'DataDictionary'), ...
    dictionaryName);
parameterCount = 0;
interfaceCount = 0;
version = '';
commandPresent = false;
measurementPresent = false;
statusPresent = false;
calibrationPresent = false;
resetPresent = false;
shadowCount = Inf;
if ~dictionaryPresent
    return;
end
try
    parameterCatalog = Simulink.data.evalinGlobal(controllerName, ...
        'PMSM_FOC_Parameter_Catalog');
    interfaceCatalog = Simulink.data.evalinGlobal(controllerName, ...
        'PMSM_FOC_Interface_Catalog');
    version = Simulink.data.evalinGlobal(controllerName, ...
        'PMSM_FOC_Parameter_Set_Version');
    commandBus = Simulink.data.evalinGlobal(controllerName, ...
        'ControlCommandBus');
    measurementBus = Simulink.data.evalinGlobal(controllerName, ...
        'MeasurementBus');
    statusBus = Simulink.data.evalinGlobal(controllerName, ...
        'ControlStatusBus');
    calibrationBus = Simulink.data.evalinGlobal(controllerName, ...
        'CalibrationBus');
    parameterCount = numel(parameterCatalog);
    interfaceCount = numel(interfaceCatalog);
    commandNames = {commandBus.Elements.Name};
    measurementNames = {measurementBus.Elements.Name};
    statusNames = {statusBus.Elements.Name};
    calibrationNames = {calibrationBus.Elements.Name};
    commandPresent = isequal(commandNames, {'StartRequest', 'StopRequest', ...
        'EmergencyStop', 'DriverFault', 'HardwareGate', 'Direction', ...
        'SpeedReferenceRpm', 'TorqueReferenceNm', 'FaultResetRequest'});
    measurementPresent = isequal(measurementNames, {'PhaseCurrentA', ...
        'PhaseCurrentB', 'PhaseCurrentC', 'DcBusVoltage', ...
        'ElectricalAngleRad', 'MechanicalSpeedRpm', 'Valid', ...
        'TimestampSeconds', 'FreshnessTicks'});
    statusPresent = isequal(statusNames, {'DutyA', 'DutyB', 'DutyC', ...
        'IqReference', 'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand', ...
        'PwmEnable', 'MotorStateCode', 'FaultBits', 'FaultCode', ...
        'CurrentLimitActive', 'VoltageLimitActive', 'MeasurementValid'});
    calibrationPresent = isequal(calibrationNames, { ...
        'PhaseCurrentOffsetA', 'PhaseCurrentOffsetB', 'AlignmentVd', ...
        'AlignmentVq', 'AlignmentDurationTicks', 'SampleCount', ...
        'ParameterSetVersion', 'ParameterSetCrc'});
    resetPresent = any(strcmp(commandNames, 'FaultResetRequest'));
    controllerWorkspace = get_param(controllerName, 'ModelWorkspace');
    harnessWorkspace = get_param(harnessName, 'ModelWorkspace');
    shadowCount = 0;
    for parameterIndex = 1:numel(parameterCatalog)
        parameterName = parameterCatalog(parameterIndex).Name;
        shadowCount = shadowCount + double( ...
            controllerWorkspace.hasVariable(parameterName));
        shadowCount = shadowCount + double( ...
            harnessWorkspace.hasVariable(parameterName));
    end
catch
    dictionaryPresent = false;
end
end

function present = hasStateflowTransition(chart, sourceName, ...
        destinationName, labelFragment)
present = false;
transitions = chart.find('-isa', 'Stateflow.Transition');
for transitionIndex = 1:numel(transitions)
    transition = transitions(transitionIndex);
    if isempty(transition.Source) || isempty(transition.Destination)
        continue;
    end
    if strcmp(transition.Source.Name, sourceName) && ...
            strcmp(transition.Destination.Name, destinationName) && ...
            contains(transition.LabelString, labelFragment)
        present = true;
        return;
    end
end
end

function count = countDanglingLines(parent)
count = 0;
lineHandles = find_system(parent, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for index = 1:numel(lineHandles)
    sourcePort = get_param(lineHandles(index), 'SrcPortHandle');
    destinationPorts = get_param(lineHandles(index), 'DstPortHandle');
    if isempty(sourcePort) || sourcePort == -1 || isempty(destinationPorts) || ...
            any(destinationPorts == -1)
        count = count + 1;
    end
end
end

function writeVerificationReport(versionDirectory, controllerName, harnessName, ...
        nativeMetrics, mcbMetrics, nativePass, mcbPass, variantReport, ...
        architectureReport, codeReport, stateflowReport, calibrationReport, ...
        faultReport, fastFaultReport, faultMatrixReport, faultResetReport, ...
        runReport)
reportPath = fullfile(versionDirectory, 'verification_report.txt');
reportHandle = fopen(reportPath, 'w');
assert(reportHandle ~= -1, 'Unable to create verification report.');
reportCleanup = onCleanup(@() fclose(reportHandle));
fprintf(reportHandle, 'PMSM FOC Dual Plant v2.1.0 verification\n');
writeBuildManifest(reportHandle, runReport);
fprintf(reportHandle, 'Controller model: %s\n', controllerName);
fprintf(reportHandle, 'Closed-loop model: %s\n', harnessName);
fprintf(reportHandle, 'Default selection: 2 (MathWorks MCB PMSM HDL)\n');
fprintf(reportHandle, 'Variant choices: %d\n', variantReport.ChoiceCount);
fprintf(reportHandle, 'Native choice present: %d\n', variantReport.NativeChoicePresent);
fprintf(reportHandle, 'MathWorks reference: %s\n', variantReport.McbReference);
fprintf(reportHandle, 'MathWorks reference verified: %d\n', variantReport.McbReferencePresent);
fprintf(reportHandle, 'Callback button present: %d\n', variantReport.CallbackButtonPresent);
fprintf(reportHandle, 'One-click switch control present: %d\n', variantReport.OneClickLinkPresent);
fprintf(reportHandle, 'FOC component count: %d\n', architectureReport.ComponentCount);
fprintf(reportHandle, 'Controller input count: %d\n', ...
    architectureReport.ControllerInputCount);
fprintf(reportHandle, 'Controller output count: %d\n', ...
    architectureReport.ControllerOutputCount);
fprintf(reportHandle, 'Controller components present: %d\n', ...
    architectureReport.ControllerComponentsPresent);
fprintf(reportHandle, 'Harness components present: %d\n', ...
    architectureReport.HarnessComponentsPresent);
fprintf(reportHandle, 'Controller explicit task boundaries present: %d\n', ...
    architectureReport.ControllerTaskBoundariesPresent);
fprintf(reportHandle, 'Harness explicit task boundaries present: %d\n', ...
    architectureReport.HarnessTaskBoundariesPresent);
fprintf(reportHandle, 'Controller safety monitor present: %d\n', ...
    architectureReport.ControllerSafetyMonitorPresent);
fprintf(reportHandle, 'Harness safety monitor present: %d\n', ...
    architectureReport.HarnessSafetyMonitorPresent);
fprintf(reportHandle, 'Controller 100 us fast safety gate present: %d\n', ...
    architectureReport.ControllerFastSafetyPresent);
fprintf(reportHandle, 'Harness 100 us fast safety gate present: %d\n', ...
    architectureReport.HarnessFastSafetyPresent);
fprintf(reportHandle, 'Controller 1 ms slow safety monitor present: %d\n', ...
    architectureReport.ControllerSlowSafetyPresent);
fprintf(reportHandle, 'Harness 1 ms slow safety monitor present: %d\n', ...
    architectureReport.HarnessSlowSafetyPresent);
fprintf(reportHandle, 'Stateflow chart present: %d\n', ...
    architectureReport.StateflowPresent);
fprintf(reportHandle, 'Stateflow state count: %d\n', ...
    architectureReport.StateflowStateCount);
fprintf(reportHandle, 'Stateflow SUPERVISED domain present: %d\n', ...
    architectureReport.StateflowSuperstatePresent);
fprintf(reportHandle, 'Legacy scalar FaultResetAck port absent: %d\n', ...
    architectureReport.LegacyScalarFaultResetPortAbsent);
fprintf(reportHandle, 'Typed Bus interface present: %d\n', ...
    architectureReport.BusInterfacePresent);
fprintf(reportHandle, 'Data dictionary present: %d\n', ...
    architectureReport.DataDictionaryPresent);
fprintf(reportHandle, 'Data dictionary parameter count: %d\n', ...
    architectureReport.ParameterCatalogCount);
fprintf(reportHandle, 'Data dictionary interface element count: %d\n', ...
    architectureReport.InterfaceCatalogCount);
fprintf(reportHandle, 'Data dictionary parameter-set version: %s\n', ...
    architectureReport.DataDictionaryVersion);
fprintf(reportHandle, 'ControlCommandBus elements verified: %d\n', ...
    architectureReport.ControlCommandElementsPresent);
fprintf(reportHandle, 'MeasurementBus elements verified: %d\n', ...
    architectureReport.MeasurementElementsPresent);
fprintf(reportHandle, 'ControlStatusBus elements verified: %d\n', ...
    architectureReport.ControlStatusElementsPresent);
fprintf(reportHandle, 'CalibrationBus elements verified: %d\n', ...
    architectureReport.CalibrationElementsPresent);
fprintf(reportHandle, 'FaultResetRequest Bus element present: %d\n', ...
    architectureReport.FaultResetRequestElementPresent);
fprintf(reportHandle, 'Migrated parameter model-workspace shadows: %d\n', ...
    architectureReport.MigratedParameterShadowCount);
fprintf(reportHandle, 'Acknowledged FAULT reset transition present: %d\n', ...
    architectureReport.AcknowledgedFaultTransitionPresent);
fprintf(reportHandle, 'CALIB done transition present: %d\n', ...
    architectureReport.CalibrationDoneTransitionPresent);
fprintf(reportHandle, 'RUN stop to INIT transition present: %d\n', ...
    architectureReport.RunStopToInitPresent);
fprintf(reportHandle, 'ALIGN stop to INIT transition present: %d\n', ...
    architectureReport.AlignStopToInitPresent);
fprintf(reportHandle, 'Current offset calibration component present: %d\n', ...
    architectureReport.CalibrationComponentPresent);
fprintf(reportHandle, 'Alignment DQ override component present: %d\n', ...
    architectureReport.AlignmentComponentPresent);
fprintf(reportHandle, 'PI reset ports present: %d\n', ...
    architectureReport.PiResetPortsPresent);
fprintf(reportHandle, 'Stateflow task sample time s: %.9g\n', ...
    architectureReport.StateflowSampleTime);
fprintf(reportHandle, 'Speed task sample time s: %.9g\n', ...
    architectureReport.SpeedTaskSampleTime);
fprintf(reportHandle, 'Current task sample time s: %.9g\n', ...
    architectureReport.CurrentTaskSampleTime);
fprintf(reportHandle, 'Fast safety task sample time s: %.9g\n', ...
    architectureReport.FastSafetySampleTime);
fprintf(reportHandle, 'Slow safety task sample time s: %.9g\n', ...
    architectureReport.SlowSafetySampleTime);
fprintf(reportHandle, 'Shared electrical-angle trig component present: %d\n', ...
    architectureReport.SharedTrigPresent);
fprintf(reportHandle, 'Controller dangling line count: %d\n', ...
    architectureReport.ControllerDanglingLines);
fprintf(reportHandle, 'Harness dangling line count: %d\n', ...
    architectureReport.HarnessDanglingLines);
fprintf(reportHandle, 'Component architecture verification pass: %d\n', ...
    architectureReport.Pass);
fprintf(reportHandle, 'Stateflow normal visited states: %s\n', ...
    mat2str(stateflowReport.VisitedStates));
fprintf(reportHandle, 'Stateflow RUN entry time s: %.9g\n', ...
    stateflowReport.RunEntryTimeS);
fprintf(reportHandle, 'Stateflow normal final state: %d\n', ...
    stateflowReport.FinalState);
fprintf(reportHandle, 'Stateflow normal verification pass: %d\n', ...
    stateflowReport.Pass);
fprintf(reportHandle, 'CALIB action verification pass: %d\n', ...
    stateflowReport.CalibrationPass);
fprintf(reportHandle, 'ALIGN action verification pass: %d\n', ...
    stateflowReport.AlignmentPass);
fprintf(reportHandle, 'Calibration injected offset A: 0.75\n');
fprintf(reportHandle, 'Calibration injected offset B: -0.5\n');
fprintf(reportHandle, 'Calibration estimated offset A: %.9g\n', ...
    calibrationReport.EstimatedOffsetA);
fprintf(reportHandle, 'Calibration estimated offset B: %.9g\n', ...
    calibrationReport.EstimatedOffsetB);
fprintf(reportHandle, 'Calibration corrected current A at completion: %.9g\n', ...
    calibrationReport.CorrectedCurrentAAtCompletion);
fprintf(reportHandle, 'Calibration corrected current B at completion: %.9g\n', ...
    calibrationReport.CorrectedCurrentBAtCompletion);
fprintf(reportHandle, 'Calibration completion time s: %.9g\n', ...
    calibrationReport.CompletionTimeS);
fprintf(reportHandle, 'Calibration numeric verification pass: %d\n', ...
    calibrationReport.Pass);
fprintf(reportHandle, 'Fault injection DC bus V: 70\n');
fprintf(reportHandle, 'Fault visited states: %s\n', ...
    mat2str(faultReport.VisitedStates));
fprintf(reportHandle, 'FAULT entry time s: %.9g\n', ...
    faultReport.FaultEntryTimeS);
fprintf(reportHandle, 'Fault final state: %d\n', faultReport.FinalState);
fprintf(reportHandle, 'Fault final PWM enable: %d\n', ...
    faultReport.FinalPwmEnable);
fprintf(reportHandle, 'Fault final duty: %.9g\n', faultReport.FinalDuty);
fprintf(reportHandle, 'Stateflow fault verification pass: %d\n', ...
    faultReport.Pass);
fprintf(reportHandle, 'Fast overcurrent injection time s: 0.0903\n');
fprintf(reportHandle, 'Fast fault entry time s: %.9g\n', ...
    fastFaultReport.FastFaultEntryTimeS);
fprintf(reportHandle, 'Fast PWM disable time s: %.9g\n', ...
    fastFaultReport.PwmDisableTimeS);
fprintf(reportHandle, 'Fast PWM disable latency s: %.9g\n', ...
    fastFaultReport.PwmDisableLatencyS);
fprintf(reportHandle, 'Fast fault final state: %d\n', ...
    fastFaultReport.FinalState);
fprintf(reportHandle, 'Fast overcurrent verification pass: %d\n', ...
    fastFaultReport.Pass);
for scenarioIndex = 1:numel(faultMatrixReport.Scenarios)
    scenario = faultMatrixReport.Scenarios(scenarioIndex);
    fprintf(reportHandle, 'Fault latch from %s visited states: %s\n', ...
        scenario.TargetName, mat2str(scenario.VisitedStates));
    fprintf(reportHandle, 'Fault latch from %s final source active: %d\n', ...
        scenario.TargetName, scenario.FinalFaultDetected);
    fprintf(reportHandle, 'Fault latch from %s pass: %d\n', ...
        scenario.TargetName, scenario.Pass);
end
fprintf(reportHandle, 'Fault latch from every operational state pass: %d\n', ...
    faultMatrixReport.Pass);
fprintf(reportHandle, 'Fault held after source clear before acknowledgement: %d\n', ...
    faultResetReport.FaultHeldAfterSourceClear);
fprintf(reportHandle, 'Fault reset acknowledgement observed: %d\n', ...
    faultResetReport.AcknowledgementObserved);
fprintf(reportHandle, 'Fault acknowledged reset exit time s: %.9g\n', ...
    faultResetReport.ResetExitTimeS);
fprintf(reportHandle, 'Fault acknowledged reset final state: %d\n', ...
    faultResetReport.FinalState);
fprintf(reportHandle, 'Fault acknowledged reset verification pass: %d\n', ...
    faultResetReport.Pass);
writeMetrics(reportHandle, 'Native', nativeMetrics, nativePass);
writeMetrics(reportHandle, 'MathWorks MCB PMSM HDL', mcbMetrics, mcbPass);
fprintf(reportHandle, 'ERT step present: %d\n', codeReport.HasStep);
fprintf(reportHandle, 'ERT initialize present: %d\n', codeReport.HasInitialize);
fprintf(reportHandle, 'ERT inputs present: %d\n', codeReport.HasInputs);
fprintf(reportHandle, 'ERT outputs present: %d\n', codeReport.HasOutputs);
fprintf(reportHandle, 'Generated Stateflow logic present: %d\n', ...
    codeReport.HasStateflow);
fprintf(reportHandle, 'Generated fast-safety logic present: %d\n', ...
    codeReport.HasFastSafety);
fprintf(reportHandle, 'Generated slow-safety logic present: %d\n', ...
    codeReport.HasSlowSafety);
fprintf(reportHandle, 'Generated dual-rate scheduler present: %d\n', ...
    codeReport.HasDualRateScheduler);
fprintf(reportHandle, 'Generated ControlCommandBus type present: %d\n', ...
    codeReport.HasControlCommandBus);
fprintf(reportHandle, 'Generated MeasurementBus type present: %d\n', ...
    codeReport.HasMeasurementBus);
fprintf(reportHandle, 'Generated ControlStatusBus type present: %d\n', ...
    codeReport.HasControlStatusBus);
fprintf(reportHandle, 'Generated sinf call count: %d\n', ...
    codeReport.SinfCallCount);
fprintf(reportHandle, 'Generated cosf call count: %d\n', ...
    codeReport.CosfCallCount);
fprintf(reportHandle, 'Generated S-Function text present: %d\n', codeReport.HasSFunction);
fprintf(reportHandle, 'Code verification pass: %d\n', codeReport.Pass);
fprintf(reportHandle, 'Overall verification pass: %d\n', runReport.OverallPass);
end

function writeBuildManifest(reportHandle, runReport)
fprintf(reportHandle, 'Build started at: %s\n', runReport.BuildStartedAt);
fprintf(reportHandle, 'Build completed at: %s\n', runReport.BuildCompletedAt);
fprintf(reportHandle, 'Build duration s: %.9g\n', runReport.BuildDurationSeconds);
fprintf(reportHandle, 'Clean build performed: %d\n', runReport.CleanBuild);
fprintf(reportHandle, 'Batch mode: %d\n', runReport.BatchMode);
fprintf(reportHandle, 'Architecture images exported: %d\n', ...
    runReport.ExportImages);
fprintf(reportHandle, 'MATLAB version: %s\n', ...
    runReport.Environment.MATLABVersion);
fprintf(reportHandle, 'MATLAB release: %s\n', ...
    runReport.Environment.MATLABRelease);
fprintf(reportHandle, 'Host architecture: %s\n', ...
    runReport.Environment.Architecture);
fprintf(reportHandle, 'Operating system: %s\n', ...
    runReport.Environment.OperatingSystem);
fprintf(reportHandle, 'Git commit at build: %s\n', ...
    runReport.Environment.GitCommit);
fprintf(reportHandle, 'Git working tree clean at build start: %d\n', ...
    runReport.Environment.GitWorkingTreeClean);
fprintf(reportHandle, 'Git status entry count at build start: %d\n', ...
    runReport.Environment.GitStatusEntryCount);
for productIndex = 1:numel(runReport.Environment.RequiredProducts)
    product = runReport.Environment.RequiredProducts(productIndex);
    fprintf(reportHandle, 'Required product %s: installed=%d, version=%s, license=%d\n', ...
        product.Name, product.Installed, product.Version, product.LicenseAvailable);
end
fprintf(reportHandle, 'Controller semantic checksum: %s\n', ...
    runReport.ControllerChecksum);
fprintf(reportHandle, 'Harness semantic checksum: %s\n', ...
    runReport.HarnessChecksum);
fprintf(reportHandle, 'Generated interface signature: %s\n', ...
    runReport.InterfaceSignature);
writeConfiguration(reportHandle, 'Controller', ...
    runReport.ControllerConfiguration);
writeConfiguration(reportHandle, 'Harness', runReport.HarnessConfiguration);
end

function writeConfiguration(reportHandle, label, configuration)
parameterNames = fieldnames(configuration);
for parameterIndex = 1:numel(parameterNames)
    parameterName = parameterNames{parameterIndex};
    fprintf(reportHandle, '%s configuration %s: %s\n', label, ...
        parameterName, configuration.(parameterName));
end
end

function writeMetrics(reportHandle, label, metrics, pass)
fprintf(reportHandle, '%s final speed rpm: %.9g\n', label, metrics.FinalSpeedRpm);
fprintf(reportHandle, '%s maximum speed rpm: %.9g\n', label, metrics.MaximumSpeedRpm);
fprintf(reportHandle, '%s maximum abs Iq A: %.9g\n', label, metrics.MaximumAbsIqA);
fprintf(reportHandle, '%s duty minimum: %.9g\n', label, metrics.MinimumDuty);
fprintf(reportHandle, '%s duty maximum: %.9g\n', label, metrics.MaximumDuty);
fprintf(reportHandle, '%s final torque Nm: %.9g\n', label, metrics.FinalTorqueNm);
fprintf(reportHandle, '%s simulation pass: %d\n', label, pass);
end

function addGain(parent, name, gain, position)
add_block('simulink/Math Operations/Gain', [parent '/' name], ...
    'Gain', gain, 'OutDataTypeStr', 'Inherit: Inherit via internal rule', ...
    'Position', position);
end

function addSum(parent, name, signs, position)
add_block('simulink/Math Operations/Sum', [parent '/' name], ...
    'Inputs', signs, 'OutDataTypeStr', 'Inherit: Inherit via internal rule', ...
    'Position', position);
end

function addProduct(parent, name, inputs, position)
add_block('simulink/Math Operations/Product', [parent '/' name], ...
    'Inputs', inputs, 'OutDataTypeStr', 'Inherit: Inherit via internal rule', ...
    'Position', position);
end

function addTrig(parent, name, operator, position)
add_block('simulink/Math Operations/Trigonometric Function', ...
    [parent '/' name], 'Operator', operator, ...
    'ApproximationMethod', 'None', 'Position', position);
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function discardAndCloseBuildModels(controllerName, harnessName)
% Prevent an interrupted batch build from leaving dirty, locked models.
modelNames = {harnessName, controllerName};
for modelIndex = 1:numel(modelNames)
    modelName = modelNames{modelIndex};
    if bdIsLoaded(modelName)
        try
            set_param(modelName, 'Dirty', 'off');
            close_system(modelName, 0);
        catch cleanupError
            warning('PMSMFOC:BatchCleanupFailed', ...
                'Unable to close batch model %s: %s', ...
                modelName, cleanupError.message);
        end
    end
end
end
