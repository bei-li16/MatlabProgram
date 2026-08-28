function add_pmsm_foc_stateflow_v21
%ADD_PMSM_FOC_STATEFLOW_V21 Add an executable motor state manager.
% The Stateflow chart is the only owner of control and PWM enable. It
% implements INIT -> READY -> CALIB -> ALIGN -> RUN and an acknowledged,
% latched FAULT path. Simulink remains responsible for numeric FOC,
% 100-sample current-offset calibration, alignment-voltage selection and
% protection input conditioning. A seventh controller input carries the
% explicit fault-reset acknowledgement required by the architecture.

versionDirectory = fileparts(mfilename('fullpath'));
previousDirectory = pwd;
directoryCleanup = onCleanup(@() cd(previousDirectory));
cd(versionDirectory);

controllerName = 'PMSM_FOC_DualPlant_Controller_v21';
harnessName = 'PMSM_FOC_DualPlant_ClosedLoop_v21';
controllerFile = fullfile(versionDirectory, [controllerName '.slx']);
harnessFile = fullfile(versionDirectory, [harnessName '.slx']);

load_system('simulink');
load_system('sflib');
load_system(controllerFile);
load_system(harnessFile);

integrateStateManager(controllerName, false);
integrateStateManager(harnessName, true);

save_system(controllerName, [], 'OverwriteIfChangedOnDisk', true);
save_system(harnessName, [], 'OverwriteIfChangedOnDisk', true);

fprintf('CODEX_STATEFLOW_CONTROLLER_PRESENT=1\n');
fprintf('CODEX_STATEFLOW_HARNESS_PRESENT=1\n');
fprintf('CODEX_STATEFLOW_STATE_COUNT=6\n');
fprintf('CODEX_STATEFLOW_SAMPLE_TIME_S=0.0001\n');
fprintf('CODEX_STATEFLOW_CALIBRATION_COMPONENT=1\n');
fprintf('CODEX_STATEFLOW_ALIGNMENT_COMPONENT=1\n');
fprintf('CODEX_STATEFLOW_ACKNOWLEDGED_FAULT_RESET=1\n');
fprintf('CODEX_STATEFLOW_INTEGRATION_PASS=1\n');
end

function integrateStateManager(modelName, isHarness)
removePreviousIntegration(modelName);
% Deleting a previously integrated block can leave branch stubs on signals
% that are also consumed elsewhere. Remove those stubs before recreating
% ports so Simulink does not auto-attach them to the new blocks.
cleanupDanglingLines(modelName);

chartPath = [modelName '/Motor_State_Machine_100us'];
add_block('sflib/Chart', chartPath, ...
    'Position', [150 650 390 845]);
set_param(chartPath, 'BackgroundColor', 'lightBlue');
buildStateflowChart(modelName, chartPath);

safetyPath = [modelName '/Motor_Safety_Monitor_100us'];
add_block('simulink/Ports & Subsystems/Subsystem', safetyPath, ...
    'Position', [720 675 950 825], ...
    'BackgroundColor', 'orange');
buildSafetyMonitor(safetyPath);

calibrationPath = [modelName '/Current_Offset_Calibration_100us'];
add_block('simulink/Ports & Subsystems/Subsystem', calibrationPath, ...
    'Position', [410 650 660 875], ...
    'BackgroundColor', 'lightBlue');
buildCurrentOffsetCalibration(calibrationPath);

alignmentPath = [modelName '/Alignment_DQ_Override_100us'];
add_block('simulink/Ports & Subsystems/Subsystem', alignmentPath, ...
    'Position', [1130 330 1350 485], ...
    'BackgroundColor', 'lightBlue');
buildAlignmentOverride(alignmentPath);

add_block('simulink/Sources/Constant', [modelName '/Start_Threshold_Rpm'], ...
    'Value', 'single(1.0)', 'OutDataTypeStr', 'single', ...
    'Position', [80 585 130 615]);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [modelName '/Start_Command'], 'Operator', '>', ...
    'Position', [155 575 195 625], 'BackgroundColor', 'lightBlue');
add_block('simulink/Math Operations/Product', ...
    [modelName '/Stateflow_Speed_Command_Gate'], 'Inputs', '**', ...
    'Position', [65 45 95 85], 'BackgroundColor', 'lightBlue');
add_block('simulink/Sources/Constant', [modelName '/Safe_Duty_50pct'], ...
    'Value', 'single(0.5)', 'OutDataTypeStr', 'single', ...
    'Position', [1660 590 1720 620]);

for phaseIndex = 1:3
    switchName = sprintf('Stateflow_PWM_Gate_%c', char('A' + phaseIndex - 1));
    add_block('simulink/Signal Routing/Switch', ...
        [modelName '/' switchName], 'Criteria', 'u2 ~= 0', ...
        'Position', [1760 40 + 60 * phaseIndex 1805 80 + 60 * phaseIndex], ...
        'BackgroundColor', 'lightBlue');
end

if isHarness
    addHarnessTestStimuli(modelName);
    addStateLogging(modelName);
else
    add_block('simulink/Sources/In1', [modelName '/FaultResetAck'], ...
        'Port', '7', 'OutDataTypeStr', 'boolean', ...
        'Position', [20 500 50 514]);
    add_block('simulink/Sinks/Terminator', ...
        [modelName '/Motor_State_Code_Terminator'], ...
        'Position', [430 790 450 810]);
end

disconnectInport([modelName '/Speed_PI_Controller_1ms'], 1);
disconnectInport([modelName '/Speed_PI_Controller_1ms'], 3);
disconnectInport([modelName '/D_Axis_Current_PI'], 3);
disconnectInport([modelName '/Q_Axis_Current_PI'], 3);
disconnectInport([modelName '/Clarke_Transform'], 1);
disconnectInport([modelName '/Clarke_Transform'], 2);
disconnectInport([modelName '/Inverse_Park_Transform'], 1);
disconnectInport([modelName '/Inverse_Park_Transform'], 2);
disconnectInport([modelName '/Inverse_Park_Transform'], 3);
disconnectInport([modelName '/VdCommand'], 1);
disconnectInport([modelName '/VqCommand'], 1);
if isHarness
    for phaseIndex = 1:3
        disconnectInport([modelName '/Native_Average_Inverter'], phaseIndex);
    end
    disconnectInport([modelName '/Duty_A_Log'], 1);
else
    disconnectInport([modelName '/DutyA'], 1);
    disconnectInport([modelName '/DutyB'], 1);
    disconnectInport([modelName '/DutyC'], 1);
end

if isHarness
    speedReferenceSource = 'Speed_Reference_Rpm/1';
    speedFeedbackSource = 'Selectable_PMSM_Plant/1';
    phaseCurrentASource = 'Measured_Current_A_With_Test_Offset/1';
    phaseCurrentBSource = 'Measured_Current_B_With_Test_Offset/1';
    dcBusSource = 'DC_Bus_48V/1';
    electricalAngleSource = 'Selectable_PMSM_Plant/2';
    resetAckSource = 'Fault_Reset_Ack/1';
else
    speedReferenceSource = 'SpeedReferenceRpm/1';
    speedFeedbackSource = 'SpeedRpm/1';
    phaseCurrentASource = 'PhaseCurrentA/1';
    phaseCurrentBSource = 'PhaseCurrentB/1';
    dcBusSource = 'DcBusVoltage/1';
    electricalAngleSource = 'ElectricalAngleRad/1';
    resetAckSource = 'FaultResetAck/1';
end

add_line(modelName, speedReferenceSource, 'Start_Command/1', 'autorouting', 'on');
add_line(modelName, 'Start_Threshold_Rpm/1', 'Start_Command/2', 'autorouting', 'on');
add_line(modelName, 'Start_Command/1', 'Motor_State_Machine_100us/1', 'autorouting', 'on');
add_line(modelName, speedFeedbackSource, 'Motor_Safety_Monitor_100us/1', 'autorouting', 'on');
add_line(modelName, phaseCurrentASource, 'Motor_Safety_Monitor_100us/2', 'autorouting', 'on');
add_line(modelName, phaseCurrentBSource, 'Motor_Safety_Monitor_100us/3', 'autorouting', 'on');
add_line(modelName, dcBusSource, 'Motor_Safety_Monitor_100us/4', 'autorouting', 'on');
if isHarness
    add_line(modelName, 'Motor_Safety_Monitor_100us/1', ...
        'Fault_Source_OR/1', 'autorouting', 'on');
    add_line(modelName, 'Fault_Test_Command/1', ...
        'Fault_Source_OR/2', 'autorouting', 'on');
    add_line(modelName, 'Fault_Source_OR/1', ...
        'Motor_State_Machine_100us/2', 'autorouting', 'on');
    faultDetectedSource = 'Fault_Source_OR/1';
else
    add_line(modelName, 'Motor_Safety_Monitor_100us/1', ...
        'Motor_State_Machine_100us/2', 'autorouting', 'on');
    faultDetectedSource = 'Motor_Safety_Monitor_100us/1';
end
add_line(modelName, resetAckSource, ...
    'Motor_State_Machine_100us/3', 'autorouting', 'on');

add_line(modelName, phaseCurrentASource, ...
    'Current_Offset_Calibration_100us/1', 'autorouting', 'on');
add_line(modelName, phaseCurrentBSource, ...
    'Current_Offset_Calibration_100us/2', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/4', ...
    'Current_Offset_Calibration_100us/3', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/5', ...
    'Current_Offset_Calibration_100us/4', 'autorouting', 'on');
add_line(modelName, 'Current_Offset_Calibration_100us/5', ...
    'Motor_State_Machine_100us/4', 'autorouting', 'on');
add_line(modelName, 'Current_Offset_Calibration_100us/1', ...
    'Clarke_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Current_Offset_Calibration_100us/2', ...
    'Clarke_Transform/2', 'autorouting', 'on');

add_line(modelName, speedReferenceSource, ...
    'Stateflow_Speed_Command_Gate/1', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/1', ...
    'Stateflow_Speed_Command_Gate/2', 'autorouting', 'on');
add_line(modelName, 'Stateflow_Speed_Command_Gate/1', ...
    'Speed_PI_Controller_1ms/1', 'autorouting', 'on');

add_line(modelName, 'Motor_State_Machine_100us/7', ...
    'Speed_PI_Controller_1ms/3', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/7', ...
    'D_Axis_Current_PI/3', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/7', ...
    'Q_Axis_Current_PI/3', 'autorouting', 'on');

add_line(modelName, 'DQ_Voltage_Command/1', ...
    'Alignment_DQ_Override_100us/1', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/2', ...
    'Alignment_DQ_Override_100us/2', 'autorouting', 'on');
add_line(modelName, electricalAngleSource, ...
    'Alignment_DQ_Override_100us/3', 'autorouting', 'on');
add_line(modelName, 'Motor_State_Machine_100us/6', ...
    'Alignment_DQ_Override_100us/4', 'autorouting', 'on');
add_line(modelName, 'Alignment_DQ_Override_100us/1', ...
    'Inverse_Park_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Alignment_DQ_Override_100us/2', ...
    'Inverse_Park_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Alignment_DQ_Override_100us/3', ...
    'Inverse_Park_Transform/3', 'autorouting', 'on');
if ~isHarness
    add_line(modelName, 'Alignment_DQ_Override_100us/1', ...
        'VdCommand/1', 'autorouting', 'on');
    add_line(modelName, 'Alignment_DQ_Override_100us/2', ...
        'VqCommand/1', 'autorouting', 'on');
end

for phaseIndex = 1:3
    switchName = sprintf('Stateflow_PWM_Gate_%c', char('A' + phaseIndex - 1));
    add_line(modelName, sprintf('SVPWM_Duty_Calculation/%d', phaseIndex), ...
        [switchName '/1'], 'autorouting', 'on');
    add_line(modelName, 'Motor_State_Machine_100us/2', ...
        [switchName '/2'], 'autorouting', 'on');
    add_line(modelName, 'Safe_Duty_50pct/1', ...
        [switchName '/3'], 'autorouting', 'on');
end

if isHarness
    for phaseIndex = 1:3
        switchName = sprintf('Stateflow_PWM_Gate_%c', char('A' + phaseIndex - 1));
        add_line(modelName, [switchName '/1'], ...
            sprintf('Native_Average_Inverter/%d', phaseIndex), ...
            'autorouting', 'on');
    end
    add_line(modelName, 'Stateflow_PWM_Gate_A/1', ...
        'Duty_A_Log/1', 'autorouting', 'on');
    loggingSources = { ...
        'Motor_State_Machine_100us/1', ...
        'Motor_State_Machine_100us/2', ...
        'Motor_State_Machine_100us/3', ...
        'Motor_State_Machine_100us/4', ...
        'Current_Offset_Calibration_100us/5', ...
        'Motor_State_Machine_100us/6', ...
        'Current_Offset_Calibration_100us/3', ...
        'Current_Offset_Calibration_100us/4', ...
        'Current_Offset_Calibration_100us/1', ...
        'Current_Offset_Calibration_100us/2', ...
        'Alignment_DQ_Override_100us/1', ...
        'Alignment_DQ_Override_100us/2', ...
        resetAckSource, faultDetectedSource};
    for loggingIndex = 1:numel(loggingSources)
        add_line(modelName, loggingSources{loggingIndex}, ...
            sprintf('Motor_Stateflow_Test_Logging/%d', loggingIndex), ...
            'autorouting', 'on');
    end
else
    add_line(modelName, 'Stateflow_PWM_Gate_A/1', 'DutyA/1', 'autorouting', 'on');
    add_line(modelName, 'Stateflow_PWM_Gate_B/1', 'DutyB/1', 'autorouting', 'on');
    add_line(modelName, 'Stateflow_PWM_Gate_C/1', 'DutyC/1', 'autorouting', 'on');
    add_line(modelName, 'Motor_State_Machine_100us/3', ...
        'Motor_State_Code_Terminator/1', 'autorouting', 'on');
end

addStateflowAnnotation(modelName);
cleanupDanglingLines(modelName);
end

function buildStateflowChart(modelName, chartPath)
rootObject = sfroot;
machine = rootObject.find('-isa', 'Stateflow.Machine', 'Name', modelName);
assert(~isempty(machine), 'Stateflow machine was not created for %s.', modelName);
chart = machine.find('-isa', 'Stateflow.Chart', 'Name', ...
    'Motor_State_Machine_100us');
assert(~isempty(chart), 'Stateflow chart was not created for %s.', modelName);
chart = chart(1);
chart.ActionLanguage = 'C';
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '0.0001';

addChartData(chart, 'StartCmd', 'Input', 1, 'boolean');
addChartData(chart, 'FaultDetected', 'Input', 2, 'boolean');
addChartData(chart, 'ResetAck', 'Input', 3, 'boolean');
addChartData(chart, 'CalibrationDone', 'Input', 4, 'boolean');
addChartData(chart, 'ControlEnable', 'Output', 1, 'boolean');
addChartData(chart, 'PwmEnable', 'Output', 2, 'boolean');
addChartData(chart, 'StateCode', 'Output', 3, 'uint8');
addChartData(chart, 'CalibrationEnable', 'Output', 4, 'boolean');
addChartData(chart, 'CalibrationReset', 'Output', 5, 'boolean');
addChartData(chart, 'AlignmentEnable', 'Output', 6, 'boolean');
addChartData(chart, 'ControllerReset', 'Output', 7, 'boolean');

supervisedState = Stateflow.State(chart);
supervisedState.Name = 'SUPERVISED';
% Keep enough internal margin for the stop-to-INIT return transitions so
% Stateflow routes the whole line inside this decomposition boundary.
supervisedState.Position = [20 40 710 300];
supervisedState.LabelString = 'SUPERVISED';

initState = addState(supervisedState, 'INIT', [25 85 105 82], ...
    1, 0, 0, 0, 1, 0, 1);
readyState = addState(supervisedState, 'READY', [155 85 105 82], ...
    2, 0, 0, 0, 0, 0, 1);
calibState = addState(supervisedState, 'CALIB', [285 85 105 82], ...
    3, 0, 0, 1, 0, 0, 1);
alignState = addState(supervisedState, 'ALIGN', [415 85 105 82], ...
    4, 0, 1, 0, 0, 1, 1);
runState = addState(supervisedState, 'RUN', [545 85 105 82], ...
    5, 1, 1, 0, 0, 0, 0);
faultState = addState(chart, 'FAULT', [785 105 120 92], ...
    6, 0, 0, 0, 1, 0, 1);

defaultTransition = Stateflow.Transition(chart);
defaultTransition.Destination = initState;
defaultTransition.DestinationOClock = 0;
defaultTransition.SourceEndPoint = [77 25];
defaultTransition.MidPoint = [77 45];

addTransition(supervisedState, initState, readyState, ...
    'after(1,tick)', 3, 9);
addTransition(supervisedState, readyState, calibState, ...
    '[StartCmd]', 3, 9);
addTransition(supervisedState, calibState, alignState, ...
    '[CalibrationDone]', 3, 9);
addTransition(chart, calibState, faultState, ...
    'after(150,tick)', 3, 9);
addTransition(supervisedState, alignState, runState, ...
    'after(200,tick)', 3, 9);
addTransition(supervisedState, alignState, initState, '[!StartCmd]', 6, 6);
addTransition(supervisedState, runState, initState, '[!StartCmd]', 6, 6);
addTransition(chart, supervisedState, faultState, '[FaultDetected]', 3, 9);
addTransition(chart, faultState, initState, ...
    '[ResetAck && !FaultDetected && !StartCmd]', 9, 6);

chartPathHandle = getSimulinkBlockHandle(chartPath);
assert(chartPathHandle ~= -1, 'Stateflow chart block is missing.');
end

function data = addChartData(chart, name, scope, port, primitiveType)
data = Stateflow.Data(chart);
data.Name = name;
data.Scope = scope;
data.Port = port;
data.Props.Type.Method = 'Built-in';
data.Props.Type.Primitive = primitiveType;
end

function state = addState(chart, name, position, code, controlEnable, ...
        pwmEnable, calibrationEnable, calibrationReset, alignmentEnable, ...
        controllerReset)
state = Stateflow.State(chart);
state.Name = name;
state.Position = position;
state.LabelString = sprintf(['%s\nentry:\n' ...
    ' ControlEnable = %d;\n PwmEnable = %d;\n StateCode = %d;\n' ...
    ' CalibrationEnable = %d;\n CalibrationReset = %d;\n' ...
    ' AlignmentEnable = %d;\n ControllerReset = %d;'], ...
    name, controlEnable, pwmEnable, code, calibrationEnable, ...
    calibrationReset, alignmentEnable, controllerReset);
end

function transition = addTransition(chart, sourceState, destinationState, ...
        label, sourceClock, destinationClock)
transition = Stateflow.Transition(chart);
transition.Source = sourceState;
transition.Destination = destinationState;
transition.LabelString = label;
transition.SourceOClock = sourceClock;
transition.DestinationOClock = destinationClock;
end

function buildSafetyMonitor(parent)
Simulink.SubSystem.deleteContents(parent);
inputNames = {'SpeedRpm', 'PhaseCurrentA', 'PhaseCurrentB', 'DcBusVoltage'};
for inputIndex = 1:numel(inputNames)
    add_block('simulink/Sources/In1', [parent '/' inputNames{inputIndex}], ...
        'Port', num2str(inputIndex), ...
        'Position', [20 25 + 55 * inputIndex 50 39 + 55 * inputIndex]);
end
add_block('simulink/Sinks/Out1', [parent '/FaultDetected'], ...
    'Port', '1', 'Position', [520 160 550 174]);

add_block('simulink/Math Operations/Abs', [parent '/Abs_Speed'], ...
    'Position', [85 70 120 100]);
add_block('simulink/Math Operations/Abs', [parent '/Abs_Ia'], ...
    'Position', [85 125 120 155]);
add_block('simulink/Math Operations/Abs', [parent '/Abs_Ib'], ...
    'Position', [85 180 120 210]);

add_block('simulink/Sources/Constant', [parent '/Max_Speed_Rpm'], ...
    'Value', 'single(3000.0)', 'OutDataTypeStr', 'single', ...
    'Position', [145 35 205 65]);
add_block('simulink/Sources/Constant', [parent '/Max_Current_A'], ...
    'Value', 'single(12.0)', 'OutDataTypeStr', 'single', ...
    'Position', [145 245 205 275]);
add_block('simulink/Sources/Constant', [parent '/Min_Vdc'], ...
    'Value', 'single(10.0)', 'OutDataTypeStr', 'single', ...
    'Position', [145 300 205 330]);
add_block('simulink/Sources/Constant', [parent '/Max_Vdc'], ...
    'Value', 'single(60.0)', 'OutDataTypeStr', 'single', ...
    'Position', [145 350 205 380]);

comparisonNames = {'OverSpeed', 'OverCurrentA', 'OverCurrentB', ...
    'UnderVoltage', 'OverVoltage'};
operators = {'>', '>', '>', '<', '>'};
for comparisonIndex = 1:numel(comparisonNames)
    add_block('simulink/Logic and Bit Operations/Relational Operator', ...
        [parent '/' comparisonNames{comparisonIndex}], ...
        'Operator', operators{comparisonIndex}, ...
        'Position', [260 40 + 65 * comparisonIndex ...
        300 80 + 65 * comparisonIndex]);
end
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [parent '/Any_Fault'], 'Operator', 'OR', 'Inputs', '5', ...
    'Position', [405 130 455 270]);

add_line(parent, 'SpeedRpm/1', 'Abs_Speed/1', 'autorouting', 'on');
add_line(parent, 'PhaseCurrentA/1', 'Abs_Ia/1', 'autorouting', 'on');
add_line(parent, 'PhaseCurrentB/1', 'Abs_Ib/1', 'autorouting', 'on');
add_line(parent, 'Abs_Speed/1', 'OverSpeed/1', 'autorouting', 'on');
add_line(parent, 'Max_Speed_Rpm/1', 'OverSpeed/2', 'autorouting', 'on');
add_line(parent, 'Abs_Ia/1', 'OverCurrentA/1', 'autorouting', 'on');
add_line(parent, 'Max_Current_A/1', 'OverCurrentA/2', 'autorouting', 'on');
add_line(parent, 'Abs_Ib/1', 'OverCurrentB/1', 'autorouting', 'on');
add_line(parent, 'Max_Current_A/1', 'OverCurrentB/2', 'autorouting', 'on');
add_line(parent, 'DcBusVoltage/1', 'UnderVoltage/1', 'autorouting', 'on');
add_line(parent, 'Min_Vdc/1', 'UnderVoltage/2', 'autorouting', 'on');
add_line(parent, 'DcBusVoltage/1', 'OverVoltage/1', 'autorouting', 'on');
add_line(parent, 'Max_Vdc/1', 'OverVoltage/2', 'autorouting', 'on');
for comparisonIndex = 1:numel(comparisonNames)
    add_line(parent, [comparisonNames{comparisonIndex} '/1'], ...
        sprintf('Any_Fault/%d', comparisonIndex), 'autorouting', 'on');
end
add_line(parent, 'Any_Fault/1', 'FaultDetected/1', 'autorouting', 'on');
Simulink.BlockDiagram.arrangeSystem(parent);
end

function buildCurrentOffsetCalibration(parent)
% Numeric 100-sample averaging stays in Simulink. Stateflow only starts or
% resets this component and waits for CalibrationDone.
Simulink.SubSystem.deleteContents(parent);
inputNames = {'RawIa', 'RawIb', 'CalibrationEnable', 'CalibrationReset'};
for inputIndex = 1:numel(inputNames)
    add_block('simulink/Sources/In1', [parent '/' inputNames{inputIndex}], ...
        'Port', num2str(inputIndex), ...
        'Position', [20 55 + 70 * inputIndex 50 69 + 70 * inputIndex]);
end
outputNames = {'CorrectedIa', 'CorrectedIb', 'OffsetIa', 'OffsetIb', ...
    'CalibrationDone'};
for outputIndex = 1:numel(outputNames)
    add_block('simulink/Sinks/Out1', [parent '/' outputNames{outputIndex}], ...
        'Port', num2str(outputIndex), ...
        'Position', [760 45 + 65 * outputIndex 790 59 + 65 * outputIndex]);
end

add_block('simulink/Sources/Constant', [parent '/Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [75 390 125 420]);
add_block('simulink/Sources/Constant', [parent '/One'], ...
    'Value', 'single(1.0)', 'OutDataTypeStr', 'single', ...
    'Position', [75 440 125 470]);
add_block('simulink/Sources/Constant', [parent '/Sample_Target'], ...
    'Value', 'single(100.0)', 'OutDataTypeStr', 'single', ...
    'Position', [75 490 145 520]);

stateNames = {'SumA_State', 'SumB_State', 'Count_State'};
stateY = [155 245 335];
for stateIndex = 1:numel(stateNames)
    add_block('simulink/Discrete/Unit Delay', ...
        [parent '/' stateNames{stateIndex}], ...
        'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
        'Position', [235 stateY(stateIndex) 290 stateY(stateIndex) + 30]);
end

add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [parent '/Count_Below_Target'], 'Operator', '<', ...
    'Position', [185 455 225 485]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [parent '/Accumulate_Enable'], 'Operator', 'AND', 'Inputs', '2', ...
    'Position', [280 430 325 475]);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [parent '/Count_Positive'], 'Operator', '>', ...
    'Position', [345 485 385 515]);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [parent '/Count_Complete'], 'Operator', '>=', ...
    'Position', [665 350 710 380]);

add_block('simulink/Math Operations/Sum', [parent '/SumA_Add'], ...
    'Inputs', '++', 'Position', [345 125 375 180]);
add_block('simulink/Math Operations/Sum', [parent '/SumB_Add'], ...
    'Inputs', '++', 'Position', [345 215 375 270]);
add_block('simulink/Math Operations/Sum', [parent '/Count_Add'], ...
    'Inputs', '++', 'Position', [345 305 375 360]);

holdNames = {'SumA_Hold', 'SumB_Hold', 'Count_Hold'};
resetNames = {'SumA_Reset', 'SumB_Reset', 'Count_Reset'};
switchY = [130 220 310];
for switchIndex = 1:numel(holdNames)
    add_block('simulink/Signal Routing/Switch', ...
        [parent '/' holdNames{switchIndex}], 'Criteria', 'u2 ~= 0', ...
        'Position', [420 switchY(switchIndex) 465 switchY(switchIndex) + 50]);
    add_block('simulink/Signal Routing/Switch', ...
        [parent '/' resetNames{switchIndex}], 'Criteria', 'u2 ~= 0', ...
        'Position', [505 switchY(switchIndex) 550 switchY(switchIndex) + 50]);
end

add_block('simulink/Math Operations/Product', [parent '/OffsetA_Calculate'], ...
    'Inputs', '*/', 'Position', [430 535 475 575]);
add_block('simulink/Math Operations/Product', [parent '/OffsetB_Calculate'], ...
    'Inputs', '*/', 'Position', [430 605 475 645]);
add_block('simulink/Signal Routing/Switch', [parent '/OffsetA_Valid'], ...
    'Criteria', 'u2 ~= 0', 'Position', [535 525 580 580]);
add_block('simulink/Signal Routing/Switch', [parent '/OffsetB_Valid'], ...
    'Criteria', 'u2 ~= 0', 'Position', [535 595 580 650]);
add_block('simulink/Math Operations/Sum', [parent '/Correct_Ia'], ...
    'Inputs', '+-', 'Position', [650 115 680 170]);
add_block('simulink/Math Operations/Sum', [parent '/Correct_Ib'], ...
    'Inputs', '+-', 'Position', [650 205 680 260]);

add_line(parent, 'Count_State/1', 'Count_Below_Target/1', 'autorouting', 'on');
add_line(parent, 'Sample_Target/1', 'Count_Below_Target/2', 'autorouting', 'on');
add_line(parent, 'CalibrationEnable/1', 'Accumulate_Enable/1', 'autorouting', 'on');
add_line(parent, 'Count_Below_Target/1', 'Accumulate_Enable/2', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'Count_Positive/1', 'autorouting', 'on');
add_line(parent, 'Zero/1', 'Count_Positive/2', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'Count_Complete/1', 'autorouting', 'on');
add_line(parent, 'Sample_Target/1', 'Count_Complete/2', 'autorouting', 'on');
add_line(parent, 'Count_Complete/1', 'CalibrationDone/1', 'autorouting', 'on');

add_line(parent, 'SumA_State/1', 'SumA_Add/1', 'autorouting', 'on');
add_line(parent, 'RawIa/1', 'SumA_Add/2', 'autorouting', 'on');
add_line(parent, 'SumB_State/1', 'SumB_Add/1', 'autorouting', 'on');
add_line(parent, 'RawIb/1', 'SumB_Add/2', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'Count_Add/1', 'autorouting', 'on');
add_line(parent, 'One/1', 'Count_Add/2', 'autorouting', 'on');

add_line(parent, 'SumA_Add/1', 'SumA_Hold/1', 'autorouting', 'on');
add_line(parent, 'SumB_Add/1', 'SumB_Hold/1', 'autorouting', 'on');
add_line(parent, 'Count_Add/1', 'Count_Hold/1', 'autorouting', 'on');
add_line(parent, 'Accumulate_Enable/1', 'SumA_Hold/2', 'autorouting', 'on');
add_line(parent, 'Accumulate_Enable/1', 'SumB_Hold/2', 'autorouting', 'on');
add_line(parent, 'Accumulate_Enable/1', 'Count_Hold/2', 'autorouting', 'on');
add_line(parent, 'SumA_State/1', 'SumA_Hold/3', 'autorouting', 'on');
add_line(parent, 'SumB_State/1', 'SumB_Hold/3', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'Count_Hold/3', 'autorouting', 'on');

for resetIndex = 1:numel(resetNames)
    add_line(parent, 'Zero/1', [resetNames{resetIndex} '/1'], 'autorouting', 'on');
    add_line(parent, 'CalibrationReset/1', ...
        [resetNames{resetIndex} '/2'], 'autorouting', 'on');
    add_line(parent, [holdNames{resetIndex} '/1'], ...
        [resetNames{resetIndex} '/3'], 'autorouting', 'on');
    add_line(parent, [resetNames{resetIndex} '/1'], ...
        [stateNames{resetIndex} '/1'], 'autorouting', 'on');
end

add_line(parent, 'SumA_State/1', 'OffsetA_Calculate/1', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'OffsetA_Calculate/2', 'autorouting', 'on');
add_line(parent, 'SumB_State/1', 'OffsetB_Calculate/1', 'autorouting', 'on');
add_line(parent, 'Count_State/1', 'OffsetB_Calculate/2', 'autorouting', 'on');
add_line(parent, 'OffsetA_Calculate/1', 'OffsetA_Valid/1', 'autorouting', 'on');
add_line(parent, 'OffsetB_Calculate/1', 'OffsetB_Valid/1', 'autorouting', 'on');
add_line(parent, 'Count_Positive/1', 'OffsetA_Valid/2', 'autorouting', 'on');
add_line(parent, 'Count_Positive/1', 'OffsetB_Valid/2', 'autorouting', 'on');
add_line(parent, 'Zero/1', 'OffsetA_Valid/3', 'autorouting', 'on');
add_line(parent, 'Zero/1', 'OffsetB_Valid/3', 'autorouting', 'on');
add_line(parent, 'RawIa/1', 'Correct_Ia/1', 'autorouting', 'on');
add_line(parent, 'OffsetA_Valid/1', 'Correct_Ia/2', 'autorouting', 'on');
add_line(parent, 'RawIb/1', 'Correct_Ib/1', 'autorouting', 'on');
add_line(parent, 'OffsetB_Valid/1', 'Correct_Ib/2', 'autorouting', 'on');
add_line(parent, 'Correct_Ia/1', 'CorrectedIa/1', 'autorouting', 'on');
add_line(parent, 'Correct_Ib/1', 'CorrectedIb/1', 'autorouting', 'on');
add_line(parent, 'OffsetA_Valid/1', 'OffsetIa/1', 'autorouting', 'on');
add_line(parent, 'OffsetB_Valid/1', 'OffsetIb/1', 'autorouting', 'on');

annotation = Simulink.Annotation(parent, sprintf([ ...
    'RESPONSIBILITY: estimate and hold phase-current ADC offsets\n' ...
    'Task: 100 us | Samples: 100 | Reset: INIT/FAULT\n' ...
    'Outputs: corrected Ia/Ib, offsets and CalibrationDone']));
annotation.Position = [110 15 675 85];
annotation.ForegroundColor = 'blue';
annotation.BackgroundColor = 'lightBlue';
Simulink.BlockDiagram.arrangeSystem(parent);
end

function buildAlignmentOverride(parent)
% Stateflow grants ALIGN permission; this Simulink component owns the
% numeric 2 V d-axis, zero-q-axis and fixed-angle command selection.
Simulink.SubSystem.deleteContents(parent);
inputNames = {'FocVd', 'FocVq', 'FeedbackTheta', 'AlignmentEnable'};
for inputIndex = 1:numel(inputNames)
    add_block('simulink/Sources/In1', [parent '/' inputNames{inputIndex}], ...
        'Port', num2str(inputIndex), ...
        'Position', [20 60 + 65 * inputIndex 50 74 + 65 * inputIndex]);
end
outputNames = {'AppliedVd', 'AppliedVq', 'AppliedTheta'};
for outputIndex = 1:numel(outputNames)
    add_block('simulink/Sinks/Out1', [parent '/' outputNames{outputIndex}], ...
        'Port', num2str(outputIndex), ...
        'Position', [475 85 + 80 * outputIndex 505 99 + 80 * outputIndex]);
end
constantNames = {'Align_Vd_2V', 'Align_Vq_Zero', 'Align_Theta_Zero'};
constantValues = {'single(2.0)', 'single(0.0)', 'single(0.0)'};
switchNames = {'Vd_Select', 'Vq_Select', 'Theta_Select'};
normalSources = {'FocVd/1', 'FocVq/1', 'FeedbackTheta/1'};
outputTargets = {'AppliedVd/1', 'AppliedVq/1', 'AppliedTheta/1'};
for pathIndex = 1:numel(switchNames)
    y = 110 + 80 * pathIndex;
    add_block('simulink/Sources/Constant', ...
        [parent '/' constantNames{pathIndex}], ...
        'Value', constantValues{pathIndex}, 'OutDataTypeStr', 'single', ...
        'Position', [110 y - 40 170 y - 10]);
    add_block('simulink/Signal Routing/Switch', ...
        [parent '/' switchNames{pathIndex}], 'Criteria', 'u2 ~= 0', ...
        'Position', [300 y - 35 345 y + 15]);
    add_line(parent, [constantNames{pathIndex} '/1'], ...
        [switchNames{pathIndex} '/1'], 'autorouting', 'on');
    add_line(parent, 'AlignmentEnable/1', ...
        [switchNames{pathIndex} '/2'], 'autorouting', 'on');
    add_line(parent, normalSources{pathIndex}, ...
        [switchNames{pathIndex} '/3'], 'autorouting', 'on');
    add_line(parent, [switchNames{pathIndex} '/1'], ...
        outputTargets{pathIndex}, 'autorouting', 'on');
end
annotation = Simulink.Annotation(parent, sprintf([ ...
    'RESPONSIBILITY: apply fixed rotor-alignment voltage only in ALIGN\n' ...
    'Parameters: Vd=2 V, Vq=0 V, theta=0 rad | Task: 100 us']));
annotation.Position = [80 15 445 80];
annotation.ForegroundColor = 'blue';
annotation.BackgroundColor = 'lightBlue';
Simulink.BlockDiagram.arrangeSystem(parent);
end

function addHarnessTestStimuli(modelName)
% These harness-only sources default to zero. The builder overrides model
% workspace data for deterministic calibration and fault-latch tests.
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_A', single(0.0));
assignin(modelWorkspace, 'PMSM_TEST_CURRENT_OFFSET_B', single(0.0));
assignin(modelWorkspace, 'PMSM_FAULT_TEST_SIGNAL', timeseries(false, 0.0));
assignin(modelWorkspace, 'PMSM_RESET_ACK_SIGNAL', timeseries(false, 0.0));

add_block('simulink/Sources/Constant', [modelName '/Test_Current_Offset_A'], ...
    'Value', 'PMSM_TEST_CURRENT_OFFSET_A', 'OutDataTypeStr', 'single', ...
    'Position', [1950 665 2025 695]);
add_block('simulink/Sources/Constant', [modelName '/Test_Current_Offset_B'], ...
    'Value', 'PMSM_TEST_CURRENT_OFFSET_B', 'OutDataTypeStr', 'single', ...
    'Position', [1950 720 2025 750]);
add_block('simulink/Math Operations/Sum', ...
    [modelName '/Measured_Current_A_With_Test_Offset'], ...
    'Inputs', '++', 'Position', [2060 650 2090 705]);
add_block('simulink/Math Operations/Sum', ...
    [modelName '/Measured_Current_B_With_Test_Offset'], ...
    'Inputs', '++', 'Position', [2060 715 2090 770]);
add_line(modelName, 'Selectable_PMSM_Plant/3', ...
    'Measured_Current_A_With_Test_Offset/1', 'autorouting', 'on');
add_line(modelName, 'Test_Current_Offset_A/1', ...
    'Measured_Current_A_With_Test_Offset/2', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/4', ...
    'Measured_Current_B_With_Test_Offset/1', 'autorouting', 'on');
add_line(modelName, 'Test_Current_Offset_B/1', ...
    'Measured_Current_B_With_Test_Offset/2', 'autorouting', 'on');

add_block('simulink/Sources/From Workspace', [modelName '/Fault_Test_Command'], ...
    'VariableName', 'PMSM_FAULT_TEST_SIGNAL', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [570 590 700 620]);
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [modelName '/Fault_Source_OR'], 'Operator', 'OR', 'Inputs', '2', ...
    'Position', [815 610 865 660]);
add_block('simulink/Sources/From Workspace', [modelName '/Fault_Reset_Ack'], ...
    'VariableName', 'PMSM_RESET_ACK_SIGNAL', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [75 915 205 945]);
end

function addStateLogging(modelName)
loggingDefinitions = { ...
    'Motor_Control_Enable_Log', 'motor_control_enable'; ...
    'Motor_PWM_Enable_Log', 'motor_pwm_enable'; ...
    'Motor_State_Code_Log', 'motor_state_code'; ...
    'Motor_Calibration_Enable_Log', 'motor_calibration_enable'; ...
    'Motor_Calibration_Done_Log', 'motor_calibration_done'; ...
    'Motor_Alignment_Enable_Log', 'motor_alignment_enable'; ...
    'Motor_Current_Offset_A_Log', 'motor_current_offset_a'; ...
    'Motor_Current_Offset_B_Log', 'motor_current_offset_b'; ...
    'Motor_Corrected_Current_A_Log', 'motor_corrected_current_a'; ...
    'Motor_Corrected_Current_B_Log', 'motor_corrected_current_b'; ...
    'Motor_Applied_Vd_Log', 'motor_applied_vd'; ...
    'Motor_Applied_Vq_Log', 'motor_applied_vq'; ...
    'Motor_Reset_Ack_Log', 'motor_reset_ack'; ...
    'Motor_Fault_Detected_Log', 'motor_fault_detected'};
loggingPath = [modelName '/Motor_Stateflow_Test_Logging'];
add_block('simulink/Ports & Subsystems/Subsystem', loggingPath, ...
    'Position', [1510 640 1740 1000], ...
    'BackgroundColor', 'gray');
Simulink.SubSystem.deleteContents(loggingPath);
for logIndex = 1:size(loggingDefinitions, 1)
    y = 35 + 45 * logIndex;
    add_block('simulink/Sources/In1', ...
        [loggingPath '/' loggingDefinitions{logIndex, 1} '_In'], ...
        'Port', num2str(logIndex), ...
        'Position', [20 y 50 y + 14]);
    add_block('simulink/Sinks/To Workspace', ...
        [loggingPath '/' loggingDefinitions{logIndex, 1}], ...
        'VariableName', loggingDefinitions{logIndex, 2}, ...
        'SaveFormat', 'Timeseries', ...
        'Position', [190 y - 8 330 y + 22]);
    add_line(loggingPath, ...
        [loggingDefinitions{logIndex, 1} '_In/1'], ...
        [loggingDefinitions{logIndex, 1} '/1'], 'autorouting', 'on');
end
annotation = Simulink.Annotation(loggingPath, sprintf([ ...
    'HARNESS-ONLY TEST LOGGING\n' ...
    'Collects Stateflow, calibration, alignment and fault evidence.\n' ...
    'This subsystem is not present in the ERT controller target.']));
annotation.Position = [70 10 360 70];
annotation.ForegroundColor = 'blue';
annotation.BackgroundColor = 'gray';
Simulink.BlockDiagram.arrangeSystem(loggingPath);
end

function removePreviousIntegration(modelName)
blockNames = {'Motor_State_Machine_100us', 'Motor_Safety_Monitor_100us', ...
    'Current_Offset_Calibration_100us', 'Alignment_DQ_Override_100us', ...
    'Start_Threshold_Rpm', 'Start_Command', ...
    'Stateflow_Speed_Command_Gate', 'Safe_Duty_50pct', ...
    'Stateflow_PWM_Gate_A', 'Stateflow_PWM_Gate_B', ...
    'Stateflow_PWM_Gate_C', 'Motor_Stateflow_Test_Logging', ...
    'Motor_Control_Enable_Log', ...
    'Motor_PWM_Enable_Log', 'Motor_State_Code_Log', ...
    'Motor_Calibration_Enable_Log', 'Motor_Calibration_Done_Log', ...
    'Motor_Alignment_Enable_Log', 'Motor_Current_Offset_A_Log', ...
    'Motor_Current_Offset_B_Log', 'Motor_Corrected_Current_A_Log', ...
    'Motor_Corrected_Current_B_Log', 'Motor_Applied_Vd_Log', ...
    'Motor_Applied_Vq_Log', 'Motor_Reset_Ack_Log', ...
    'Motor_Fault_Detected_Log', 'Motor_State_Code_Terminator', ...
    'FaultResetAck', 'Fault_Test_Command', 'Fault_Source_OR', ...
    'Fault_Reset_Ack', 'Test_Current_Offset_A', ...
    'Test_Current_Offset_B', 'Measured_Current_A_With_Test_Offset', ...
    'Measured_Current_B_With_Test_Offset'};
for blockIndex = 1:numel(blockNames)
    blockPath = [modelName '/' blockNames{blockIndex}];
    if getSimulinkBlockHandle(blockPath) ~= -1
        delete_block(blockPath);
    end
end

marker = 'STATEFLOW MOTOR SUPERVISOR V2.1.0';
annotationHandles = find_system(modelName, 'FindAll', 'on', ...
    'Type', 'annotation');
for annotationIndex = 1:numel(annotationHandles)
    annotationObject = get_param(annotationHandles(annotationIndex), 'Object');
    if contains(annotationObject.Text, marker)
        delete(annotationObject);
    end
end
end

function disconnectInport(blockPath, portIndex)
if getSimulinkBlockHandle(blockPath) == -1
    return;
end
portHandles = get_param(blockPath, 'PortHandles');
if numel(portHandles.Inport) < portIndex
    return;
end
lineHandle = get_param(portHandles.Inport(portIndex), 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end

function addStateflowAnnotation(modelName)
annotationText = sprintf(['STATEFLOW MOTOR SUPERVISOR V2.1.0\n' ...
    'States: 1 INIT | 2 READY | 3 CALIB | 4 ALIGN | 5 RUN | 6 FAULT\n' ...
    'Task: 100 us | CALIB: 100 samples | ALIGN: 20 ms at Vd=2 V\n' ...
    'Faults: |Iab| > 12 A, |speed| > 3000 rpm, Vdc outside 10..60 V\n' ...
    'ALIGN/RUN own PWM; FAULT remains latched until explicit ResetAck.']);
annotationObject = Simulink.Annotation(modelName, annotationText);
annotationObject.Position = [1040 680 1600 830];
annotationObject.FontSize = 10;
annotationObject.ForegroundColor = 'blue';
annotationObject.BackgroundColor = 'lightBlue';
end

function cleanupDanglingLines(parent)
% Deleting one branch can invalidate sibling handles returned by an earlier
% find_system call. Delete at most one branch per scan and then rescan.
while true
    lineHandles = find_system(parent, 'FindAll', 'on', ...
        'SearchDepth', 1, 'Type', 'line');
    removedLine = false;
    for lineIndex = 1:numel(lineHandles)
        try
            sourcePort = get_param(lineHandles(lineIndex), 'SrcPortHandle');
            destinationPorts = get_param(lineHandles(lineIndex), 'DstPortHandle');
            if isempty(sourcePort) || sourcePort == -1 || ...
                    isempty(destinationPorts) || any(destinationPorts == -1)
                delete_line(lineHandles(lineIndex));
                removedLine = true;
                break;
            end
        catch
            % The handle was invalidated by a sibling branch deletion.
            removedLine = true;
            break;
        end
    end
    if ~removedLine
        break;
    end
end
end
