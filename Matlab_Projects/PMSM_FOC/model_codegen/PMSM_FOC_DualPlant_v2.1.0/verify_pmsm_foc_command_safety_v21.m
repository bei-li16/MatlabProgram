function report = verify_pmsm_foc_command_safety_v21
%VERIFY_PMSM_FOC_COMMAND_SAFETY_V21 Verify ARC-003/004/005 contracts.
% The test uses the native average plant for deterministic fault timing.
% It verifies explicit command behavior, the single three-level PWM arbiter,
% latched fault identity, reset preconditions and all named cross-rate gates.

versionDirectory = fileparts(mfilename('fullpath'));
previousDirectory = pwd;
directoryCleanup = onCleanup(@() cd(previousDirectory));
cd(versionDirectory);

controllerName = 'PMSM_FOC_DualPlant_Controller_v21';
harnessName = 'PMSM_FOC_DualPlant_ClosedLoop_v21';
load_system([controllerName '.slx']);
load_system([harnessName '.slx']);
modelCleanup = onCleanup(@() closeModels(controllerName, harnessName));
create_pmsm_foc_data_dictionary_v21({controllerName, harnessName});

structure = inspectStructure(controllerName, harnessName);

normalOutput = runScenario(harnessName, 'normal');
normal = collectNormal(normalOutput);
noStartOutput = runScenario(harnessName, 'no-start');
noStart = collectNoStart(noStartOutput, harnessName);
priorityOutput = runScenario(harnessName, 'stop-priority');
stopPriority = collectStopPriority(priorityOutput);
overcurrentOutput = runScenario(harnessName, 'overcurrent');
overcurrent = collectFault(overcurrentOutput, 0.0903, 4, 3);
emergencyOutput = runScenario(harnessName, 'emergency-stop');
emergency = collectFault(emergencyOutput, 0.0903, 1, 1);
driverOutput = runScenario(harnessName, 'driver-fault');
driverFault = collectFault(driverOutput, 0.0903, 2, 2);
invalidOutput = runScenario(harnessName, 'measurement-invalid');
measurementInvalid = collectFault(invalidOutput, 0.0903, 8, 4);
hardwareOutput = runScenario(harnessName, 'hardware-gate');
hardwareGate = collectHardwareGate(hardwareOutput, 0.0903, 0.0905);
stopOutput = runScenario(harnessName, 'stop-latency');
stopLatency = collectStopLatency(stopOutput, 0.0903);
resetOutput = runScenario(harnessName, 'acknowledged-reset');
faultReset = collectReset(resetOutput);

slbuild(controllerName);
code = inspectGeneratedCode(controllerName, versionDirectory);

report.Structure = structure;
report.Normal = normal;
report.NoStart = noStart;
report.StopPriority = stopPriority;
report.Overcurrent = overcurrent;
report.EmergencyStop = emergency;
report.DriverFault = driverFault;
report.MeasurementInvalid = measurementInvalid;
report.HardwareGate = hardwareGate;
report.StopLatency = stopLatency;
report.FaultReset = faultReset;
report.GeneratedCode = code;
report.Pass = structure.Pass && normal.Pass && noStart.Pass && ...
    stopPriority.Pass && overcurrent.Pass && emergency.Pass && ...
    driverFault.Pass && measurementInvalid.Pass && hardwareGate.Pass && ...
    stopLatency.Pass && faultReset.Pass && code.Pass;

createEvidencePlot(noStartOutput, overcurrentOutput, driverOutput, ...
    hardwareOutput, versionDirectory);
writeReport(report, versionDirectory);
save(fullfile(versionDirectory, 'arc003_arc005_verification.mat'), ...
    'report');

fprintf('CODEX_ARC003_EXPLICIT_COMMAND_PASS=%d\n', ...
    noStart.Pass && stopPriority.Pass && stopLatency.Pass);
fprintf('CODEX_ARC004_PWM_ARBITRATION_PASS=%d\n', ...
    overcurrent.Pass && emergency.Pass && driverFault.Pass && ...
    measurementInvalid.Pass && hardwareGate.Pass && faultReset.Pass);
fprintf('CODEX_ARC005_RATE_CONTRACT_PASS=%d\n', structure.RateTransitionsPass);
fprintf('CODEX_ARC003_ARC005_CODEGEN_PASS=%d\n', code.Pass);
fprintf('CODEX_ARC003_ARC005_OVERALL_PASS=%d\n', report.Pass);
assert(report.Pass, 'One or more ARC-003/004/005 verification scenarios failed.');
end

function structure = inspectStructure(controllerName, harnessName)
requiredComponents = {'Command_Arbiter_1ms','Motor_Supervisor_1ms', ...
    'Fast_Protection_100us','Slow_Safety_Monitor_1ms', ...
    'Fault_Latch_Manager_100us','Final_PWM_Arbiter_100us'};
structure.ComponentsPass = all(cellfun(@(name) ...
    getSimulinkBlockHandle([controllerName '/' name]) ~= -1, ...
    requiredComponents)) && all(cellfun(@(name) ...
    getSimulinkBlockHandle([harnessName '/' name]) ~= -1, ...
    requiredComponents));
rateTransitions = {'IqRef_Rate_Transition','AnyFault_To_1ms', ...
    'CalibrationDone_To_1ms','EmergencyStop_To_1ms', ...
    'DriverFault_To_1ms','HardwareGate_To_1ms', ...
    'SupervisorPwmRequest_To_100us','CalibrationEnable_To_100us', ...
    'CalibrationReset_To_100us','AlignmentEnable_To_100us', ...
    'ControllerReset_To_100us','ResetAllowed_To_100us', ...
    'Overspeed_To_100us','Undervoltage_To_100us', ...
    'Overvoltage_To_100us','StatusState_To_100us', ...
    'StatusIqReference_To_100us'};
structure.RateTransitionsPass = all(cellfun(@(name) ...
    isBlockType(controllerName, name, 'RateTransition'), rateTransitions)) && ...
    all(cellfun(@(name) isBlockType(harnessName, name, 'RateTransition'), ...
    rateTransitions(1:15)));
legacyBlocks = {'Start_Threshold_Rpm','Start_Command_Sample_1ms', ...
    'Start_Command','Fast_Safety_Gate_100us','Stateflow_PWM_Gate_A', ...
    'Stateflow_PWM_Gate_B','Stateflow_PWM_Gate_C','Safe_Duty_50pct'};
structure.LegacyBlocksAbsent = all(cellfun(@(name) ...
    getSimulinkBlockHandle([controllerName '/' name]) == -1, legacyBlocks));
structure.SingleFinalArbiter = numel(find_system(controllerName, ...
    'SearchDepth', 1, 'Name', 'Final_PWM_Arbiter_100us')) == 1 && ...
    numel(find_system(harnessName, 'SearchDepth', 1, ...
    'Name', 'Final_PWM_Arbiter_100us')) == 1;
structure.ControllerHasNoPlantDutyMap = getSimulinkBlockHandle( ...
    [controllerName '/Average_Plant_Enable_Map']) == -1;
structure.HarnessHasPlantDutyMap = getSimulinkBlockHandle( ...
    [harnessName '/Average_Plant_Enable_Map']) ~= -1;
structure.FinalFormulaPass = isBlockType(controllerName, ...
    'Final_PWM_Arbiter_100us/Three_Level_AND', 'Logic') && ...
    strcmp(get_param([controllerName ...
    '/Final_PWM_Arbiter_100us/Three_Level_AND'], 'Inputs'), '3');
structure.NoImplicitStartParameter = ~hasDictionaryVariable( ...
    controllerName, 'PMSM_StartThreshold_Rpm');
structure.CommandFieldsPass = commandFieldsPass(controllerName);
structure.StateflowContractPass = stateflowContractPass(controllerName);
structure.FastPeriod = str2double(get_param([controllerName ...
    '/Fault_Latch_Manager_100us/EmergencyStop_Latch_State'], 'SampleTime'));
structure.SlowPeriod = str2double(get_param([controllerName ...
    '/Slow_Safety_Monitor_1ms/Speed_Sample_1ms'], 'SampleTime'));
structure.PeriodPass = abs(structure.FastPeriod - 1e-4) < eps && ...
    abs(structure.SlowPeriod - 1e-3) < eps;
structure.Pass = structure.ComponentsPass && structure.RateTransitionsPass && ...
    structure.LegacyBlocksAbsent && structure.SingleFinalArbiter && ...
    structure.ControllerHasNoPlantDutyMap && structure.HarnessHasPlantDutyMap && ...
    structure.FinalFormulaPass && structure.NoImplicitStartParameter && ...
    structure.CommandFieldsPass && structure.StateflowContractPass && ...
    structure.PeriodPass;
end

function pass = isBlockType(modelName, relativePath, expectedType)
path = [modelName '/' relativePath];
pass = getSimulinkBlockHandle(path) ~= -1 && ...
    strcmp(get_param(path, 'BlockType'), expectedType);
end

function present = hasDictionaryVariable(modelName, variableName)
present = true;
try
    Simulink.data.evalinGlobal(modelName, variableName);
catch
    present = false;
end
end

function pass = commandFieldsPass(modelName)
commandBus = Simulink.data.evalinGlobal(modelName, 'ControlCommandBus');
names = {commandBus.Elements.Name};
pass = isequal(names, {'StartRequest','StopRequest','EmergencyStop', ...
    'DriverFault','HardwareGate','Direction','SpeedReferenceRpm', ...
    'TorqueReferenceNm','FaultResetRequest'});
end

function pass = stateflowContractPass(modelName)
root = sfroot;
machine = root.find('-isa', 'Stateflow.Machine', 'Name', modelName);
chart = machine.find('-isa', 'Stateflow.Chart', 'Name', 'Motor_Supervisor_1ms');
if isempty(chart)
    pass = false;
    return;
end
chart = chart(1);
requiredData = {'StartAccepted','StopActive','FaultDetected','ResetAllowed', ...
    'CalibrationDone','SupervisorPwmRequest','StateCode'};
pass = all(cellfun(@(name) ~isempty(chart.find('-isa', ...
    'Stateflow.Data', 'Name', name)), requiredData)) && ...
    isempty(chart.find('-isa', 'Stateflow.Data', 'Name', 'StartCmd')) && ...
    isempty(chart.find('-isa', 'Stateflow.Data', 'Name', 'PwmEnable'));
end

function output = runScenario(modelName, scenario)
resetStimuli(modelName);
workspace = get_param(modelName, 'ModelWorkspace');
stopTime = 0.1;
switch scenario
    case 'normal'
        stopTime = 0.11;
    case 'no-start'
        assignin(workspace, 'PMSM_START_REQUEST_SIGNAL', timeseries(false, 0));
        stopTime = 0.03;
    case 'stop-priority'
        assignin(workspace, 'PMSM_START_REQUEST_SIGNAL', timeseries(true, 0));
        assignin(workspace, 'PMSM_STOP_REQUEST_SIGNAL', timeseries(true, 0));
        stopTime = 0.01;
    case 'overcurrent'
        assignin(workspace, 'PMSM_FAST_OVERCURRENT_SIGNAL', ...
            timeseries(single([0;20;0]), [0;0.0903;0.0905]));
        stopTime = 0.096;
    case 'emergency-stop'
        assignin(workspace, 'PMSM_EMERGENCY_STOP_SIGNAL', ...
            timeseries(logical([0;1;0]), [0;0.0903;0.0905]));
        stopTime = 0.096;
    case 'driver-fault'
        assignin(workspace, 'PMSM_DRIVER_FAULT_SIGNAL', ...
            timeseries(logical([0;1;0]), [0;0.0903;0.0905]));
        stopTime = 0.096;
    case 'measurement-invalid'
        assignin(workspace, 'PMSM_MEASUREMENT_VALID_SIGNAL', ...
            timeseries(logical([1;0;1]), [0;0.0903;0.0905]));
        stopTime = 0.096;
    case 'hardware-gate'
        assignin(workspace, 'PMSM_HARDWARE_GATE_SIGNAL', ...
            timeseries(logical([1;0;1]), [0;0.0903;0.0905]));
        stopTime = 0.094;
    case 'stop-latency'
        assignin(workspace, 'PMSM_STOP_REQUEST_SIGNAL', ...
            timeseries(logical([0;1]), [0;0.0903]));
        stopTime = 0.094;
    case 'acknowledged-reset'
        assignin(workspace, 'PMSM_START_REQUEST_SIGNAL', ...
            timeseries(logical([0;1;0]), [0;0.05;0.091]));
        assignin(workspace, 'PMSM_DRIVER_FAULT_SIGNAL', ...
            timeseries(logical([0;1;0]), [0;0.0903;0.0905]));
        assignin(workspace, 'PMSM_STOP_REQUEST_SIGNAL', ...
            timeseries(logical([0;1]), [0;0.091]));
        assignin(workspace, 'PMSM_FAULT_RESET_REQUEST_SIGNAL', ...
            timeseries(logical([0;1;0]), [0;0.093;0.096]));
        stopTime = 0.102;
    otherwise
        error('Unknown scenario %s.', scenario);
end
assignin(workspace, 'PMSM_PLANT_SELECTION', 1);
set_param(modelName, 'SimulationCommand', 'update');
output = sim(modelName, 'StopTime', num2str(stopTime, '%.9g'), ...
    'ReturnWorkspaceOutputs', 'on');
end

function resetStimuli(modelName)
workspace = get_param(modelName, 'ModelWorkspace');
assignin(workspace, 'PMSM_TEST_CURRENT_OFFSET_A', single(0));
assignin(workspace, 'PMSM_TEST_CURRENT_OFFSET_B', single(0));
assignin(workspace, 'PMSM_FAST_OVERCURRENT_SIGNAL', timeseries(single(0), 0));
assignin(workspace, 'PMSM_START_REQUEST_SIGNAL', ...
    timeseries(logical([0;1]), [0;0.05]));
assignin(workspace, 'PMSM_STOP_REQUEST_SIGNAL', timeseries(false, 0));
assignin(workspace, 'PMSM_EMERGENCY_STOP_SIGNAL', timeseries(false, 0));
assignin(workspace, 'PMSM_DRIVER_FAULT_SIGNAL', timeseries(false, 0));
assignin(workspace, 'PMSM_FAULT_RESET_REQUEST_SIGNAL', timeseries(false, 0));
assignin(workspace, 'PMSM_HARDWARE_GATE_SIGNAL', timeseries(true, 0));
assignin(workspace, 'PMSM_MEASUREMENT_VALID_SIGNAL', timeseries(true, 0));
end

function result = collectNormal(output)
state = output.get('motor_state_code');
pwm = output.get('motor_pwm_enable');
values = double(state.Data(:));
runIndex = find(values == 5, 1, 'first');
result.VisitedStates = unique(values)';
result.RunEntryTimeS = valueOrNaN(state.Time, runIndex);
result.Pass = isequal(result.VisitedStates, 1:5) && ...
    isfinite(result.RunEntryTimeS) && logical(pwm.Data(end));
end

function result = collectNoStart(output, modelName)
state = output.get('motor_state_code');
pwm = output.get('motor_pwm_enable');
values = double(state.Data(:));
speedCommand = str2double(get_param([modelName '/Speed_Reference_Rpm'], 'After'));
result.VisitedStates = unique(values)';
result.NonzeroSpeedCommandRpm = speedCommand;
result.Pass = speedCommand ~= 0 && all(ismember(result.VisitedStates, [1 2])) && ...
    all(~logical(pwm.Data(:)));
end

function result = collectStopPriority(output)
state = output.get('motor_state_code');
pwm = output.get('motor_pwm_enable');
result.VisitedStates = unique(double(state.Data(:)))';
result.Pass = all(ismember(result.VisitedStates, [1 2])) && ...
    all(~logical(pwm.Data(:)));
end

function result = collectFault(output, injectionTime, expectedBits, expectedCode)
state = output.get('motor_state_code');
pwm = output.get('motor_pwm_enable');
bits = output.get('motor_fault_bits');
code = output.get('motor_fault_code');
stateValues = double(state.Data(:));
pwmValues = logical(pwm.Data(:));
bitsValues = uint32(bits.Data(:));
codeValues = uint16(code.Data(:));
disableIndex = find(pwm.Time >= injectionTime & ~pwmValues, 1, 'first');
bitIndex = find(bits.Time >= injectionTime & bitsValues == uint32(expectedBits), ...
    1, 'first');
result.PwmDisableTimeS = valueOrNaN(pwm.Time, disableIndex);
result.PwmDisableLatencyS = result.PwmDisableTimeS - injectionTime;
result.FaultIdentityTimeS = valueOrNaN(bits.Time, bitIndex);
result.FinalBits = double(bitsValues(end));
result.FinalCode = double(codeValues(end));
result.FinalState = stateValues(end);
result.Pass = any(stateValues == 5) && any(stateValues == 6) && ...
    isfinite(result.PwmDisableLatencyS) && result.PwmDisableLatencyS >= -eps && ...
    result.PwmDisableLatencyS <= 1e-4 + eps && ...
    result.FinalBits == expectedBits && result.FinalCode == expectedCode && ...
    result.FinalState == 6 && ~pwmValues(end);
end

function result = collectHardwareGate(output, disableTime, enableTime)
pwm = output.get('motor_pwm_enable');
gate = output.get('motor_hardware_gate');
bits = output.get('motor_fault_bits');
pwmValues = logical(pwm.Data(:));
disableIndex = find(pwm.Time >= disableTime & ~pwmValues, 1, 'first');
enableIndex = find(pwm.Time >= enableTime & pwmValues, 1, 'first');
result.DisableLatencyS = valueOrNaN(pwm.Time, disableIndex) - disableTime;
result.ReenableLatencyS = valueOrNaN(pwm.Time, enableIndex) - enableTime;
result.GateLowObserved = any(~logical(gate.Data(:)));
result.FinalBits = double(uint32(bits.Data(end)));
result.Pass = result.GateLowObserved && isfinite(result.DisableLatencyS) && ...
    result.DisableLatencyS >= -eps && result.DisableLatencyS <= 1e-4 + eps && ...
    isfinite(result.ReenableLatencyS) && result.ReenableLatencyS >= -eps && ...
    result.ReenableLatencyS <= 1e-4 + eps && result.FinalBits == 0 && pwmValues(end);
end

function result = collectStopLatency(output, injectionTime)
state = output.get('motor_state_code');
pwm = output.get('motor_pwm_enable');
stateValues = double(state.Data(:));
pwmValues = logical(pwm.Data(:));
stateIndex = find(state.Time >= injectionTime & stateValues == 1, 1, 'first');
pwmIndex = find(pwm.Time >= injectionTime & ~pwmValues, 1, 'first');
result.StateResponseLatencyS = valueOrNaN(state.Time, stateIndex) - injectionTime;
result.PwmDisableLatencyS = valueOrNaN(pwm.Time, pwmIndex) - injectionTime;
result.Pass = any(stateValues == 5) && isfinite(result.StateResponseLatencyS) && ...
    result.StateResponseLatencyS >= -eps && ...
    result.StateResponseLatencyS <= 1e-3 + eps && ...
    isfinite(result.PwmDisableLatencyS) && ...
    result.PwmDisableLatencyS <= 1.1e-3 + eps;
end

function result = collectReset(output)
state = output.get('motor_state_code');
bits = output.get('motor_fault_bits');
pwm = output.get('motor_pwm_enable');
resetAllowed = output.get('motor_reset_allowed');
stateValues = double(state.Data(:));
result.VisitedStates = unique(stateValues)';
result.ResetAllowedObserved = any(logical(resetAllowed.Data(:)));
result.FinalState = stateValues(end);
result.FinalBits = double(uint32(bits.Data(end)));
result.FinalPwmEnable = logical(pwm.Data(end));
result.Pass = any(stateValues == 6) && result.ResetAllowedObserved && ...
    result.FinalBits == 0 && ismember(result.FinalState, [1 2]) && ...
    ~result.FinalPwmEnable;
end

function value = valueOrNaN(values, index)
if isempty(index)
    value = NaN;
else
    value = double(values(index));
end
end

function code = inspectGeneratedCode(controllerName, versionDirectory)
codeDirectory = fullfile(versionDirectory, [controllerName '_ert_rtw']);
files = [dir(fullfile(codeDirectory, '*.c')); dir(fullfile(codeDirectory, '*.h'))];
textValue = '';
for fileIndex = 1:numel(files)
    textValue = [textValue newline fileread(fullfile( ...
        files(fileIndex).folder, files(fileIndex).name))]; %#ok<AGROW>
end
code.DriverFaultPresent = contains(textValue, 'DriverFault');
code.HardwareGatePresent = contains(textValue, 'HardwareGate');
code.FaultBitsPresent = contains(textValue, 'FaultBits');
code.FinalPwmEnablePresent = contains(textValue, 'PwmEnable');
code.LegacyStartThresholdAbsent = ~contains(textValue, 'StartThreshold');
code.NoSFunction = ~contains(lower(textValue), 's-function');
code.Pass = ~isempty(files) && code.DriverFaultPresent && ...
    code.HardwareGatePresent && code.FaultBitsPresent && ...
    code.FinalPwmEnablePresent && code.LegacyStartThresholdAbsent && ...
    code.NoSFunction;
end

function createEvidencePlot(noStartOutput, overcurrentOutput, driverOutput, ...
        hardwareOutput, versionDirectory)
figureHandle = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100 100 1300 850]);
plotCleanup = onCleanup(@() close(figureHandle));
subplot(2,2,1);
plotSeries(noStartOutput, 'motor_state_code', 1, 'State code');
hold on; plotSeries(noStartOutput, 'motor_pwm_enable', 5, 'PWM enable x5');
title('Nonzero speed reference without StartRequest'); grid on;
legend('Location','best');
subplot(2,2,2);
plotSeries(overcurrentOutput, 'motor_fault_code', 1, 'Fault code');
hold on; plotSeries(overcurrentOutput, 'motor_pwm_enable', 5, 'PWM enable x5');
title('Software overcurrent: code 3 and fast PWM inhibit'); grid on;
legend('Location','best');
subplot(2,2,3);
plotSeries(driverOutput, 'motor_fault_code', 1, 'Fault code');
hold on; plotSeries(driverOutput, 'motor_pwm_enable', 5, 'PWM enable x5');
title('Driver fault: code 2 and latched shutdown'); grid on;
legend('Location','best');
subplot(2,2,4);
plotSeries(hardwareOutput, 'motor_hardware_gate', 1, 'HardwareGate');
hold on; plotSeries(hardwareOutput, 'motor_pwm_enable', 1, 'FinalPwmEnable');
title('Independent hardware gate disable/re-enable'); grid on;
legend('Location','best');
sgtitle('ARC-003/004/005 command, fault identity and PWM arbitration evidence');
exportgraphics(figureHandle, fullfile(versionDirectory, ...
    'arc003_arc005_command_safety_test.png'), 'Resolution', 180);
end

function plotSeries(output, variableName, scale, displayName)
series = output.get(variableName);
stairs(series.Time, double(series.Data) * scale, 'LineWidth', 1.2, ...
    'DisplayName', displayName);
xlabel('Time (s)');
end

function writeReport(report, versionDirectory)
path = fullfile(versionDirectory, 'arc003_arc005_verification_report.txt');
handle = fopen(path, 'wt');
assert(handle ~= -1, 'Unable to create %s.', path);
cleanup = onCleanup(@() fclose(handle));
fprintf(handle, 'PMSM FOC v2.1.0 ARC-003/004/005 verification\n');
fprintf(handle, 'Generated: %s\n\n', char(datetime('now', ...
    'Format', 'yyyy-MM-dd HH:mm:ss')));
fprintf(handle, 'OVERALL_PASS=%d\n', report.Pass);
fprintf(handle, 'STRUCTURE_PASS=%d\n', report.Structure.Pass);
structureFields = {'ComponentsPass','RateTransitionsPass', ...
    'LegacyBlocksAbsent','SingleFinalArbiter', ...
    'ControllerHasNoPlantDutyMap','HarnessHasPlantDutyMap', ...
    'FinalFormulaPass','NoImplicitStartParameter','CommandFieldsPass', ...
    'StateflowContractPass','PeriodPass'};
for fieldIndex = 1:numel(structureFields)
    fieldName = structureFields{fieldIndex};
    fprintf(handle, '%s=%d\n', upper(fieldName), ...
        report.Structure.(fieldName));
end
fprintf(handle, 'RATE_TRANSITIONS_PASS=%d\n', ...
    report.Structure.RateTransitionsPass);
fprintf(handle, 'EXPLICIT_COMMAND_PASS=%d\n', ...
    report.NoStart.Pass && report.StopPriority.Pass && report.StopLatency.Pass);
fprintf(handle, 'NO_START_WITH_NONZERO_SPEED_PASS=%d\n', report.NoStart.Pass);
fprintf(handle, 'STOP_PRIORITY_PASS=%d\n', report.StopPriority.Pass);
fprintf(handle, 'STOP_STATE_LATENCY_S=%.9g\n', ...
    report.StopLatency.StateResponseLatencyS);
fprintf(handle, 'OVERCURRENT_PASS=%d LATENCY_S=%.9g BITS=%d CODE=%d\n', ...
    report.Overcurrent.Pass, report.Overcurrent.PwmDisableLatencyS, ...
    report.Overcurrent.FinalBits, report.Overcurrent.FinalCode);
fprintf(handle, 'EMERGENCY_STOP_PASS=%d LATENCY_S=%.9g BITS=%d CODE=%d\n', ...
    report.EmergencyStop.Pass, report.EmergencyStop.PwmDisableLatencyS, ...
    report.EmergencyStop.FinalBits, report.EmergencyStop.FinalCode);
fprintf(handle, 'DRIVER_FAULT_PASS=%d LATENCY_S=%.9g BITS=%d CODE=%d\n', ...
    report.DriverFault.Pass, report.DriverFault.PwmDisableLatencyS, ...
    report.DriverFault.FinalBits, report.DriverFault.FinalCode);
fprintf(handle, 'MEASUREMENT_INVALID_PASS=%d LATENCY_S=%.9g BITS=%d CODE=%d\n', ...
    report.MeasurementInvalid.Pass, ...
    report.MeasurementInvalid.PwmDisableLatencyS, ...
    report.MeasurementInvalid.FinalBits, report.MeasurementInvalid.FinalCode);
fprintf(handle, 'HARDWARE_GATE_PASS=%d DISABLE_S=%.9g REENABLE_S=%.9g\n', ...
    report.HardwareGate.Pass, report.HardwareGate.DisableLatencyS, ...
    report.HardwareGate.ReenableLatencyS);
fprintf(handle, 'ACKNOWLEDGED_RESET_PASS=%d\n', report.FaultReset.Pass);
fprintf(handle, 'GENERATED_CODE_PASS=%d\n', report.GeneratedCode.Pass);
end

function closeModels(controllerName, harnessName)
if bdIsLoaded(harnessName)
    close_system(harnessName, 0);
end
if bdIsLoaded(controllerName)
    close_system(controllerName, 0);
end
end
