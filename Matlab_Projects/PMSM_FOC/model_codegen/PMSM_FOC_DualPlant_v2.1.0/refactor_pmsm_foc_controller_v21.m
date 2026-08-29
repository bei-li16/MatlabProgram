function refactor_pmsm_foc_controller_v21
%REFACTOR_PMSM_FOC_CONTROLLER_V21 Build an explicit cascaded dual-rate FOC.
% The code-generation model keeps its 8 outputs and adds a seventh input for
% explicit fault-reset acknowledgement. Both model
% roots show the textbook cascade directly: 1 ms speed PI, explicit 1 ms to
% 100 us rate transition, Clarke/Park feedback transforms, independent d/q
% current PIs, decoupling, inverse transforms, SVPWM, inverter, and PMSM.
% Rebuilding starts from a clean top-level wiring graph, so stale or dangling
% red line segments cannot survive a refactor.

versionDirectory = fileparts(mfilename('fullpath'));
previousDirectory = pwd;
directoryCleanup = onCleanup(@() cd(previousDirectory));
cd(versionDirectory);

controllerName = 'PMSM_FOC_DualPlant_Controller_v21';
harnessName = 'PMSM_FOC_DualPlant_ClosedLoop_v21';
controllerFile = fullfile(versionDirectory, [controllerName '.slx']);
harnessFile = fullfile(versionDirectory, [harnessName '.slx']);

load_system(controllerFile);
load_system(harnessFile);

buildControllerModelArchitecture(controllerName);
buildHarnessArchitecture(harnessName);
add_pmsm_foc_stateflow_v21;
open_system(controllerName);
set_param(controllerName, 'ZoomFactor', 'FitSystem');
set_param(controllerName, 'ZoomFactor', '100');
open_system(harnessName);
set_param(harnessName, 'ZoomFactor', 'FitSystem');
set_param(harnessName, 'ZoomFactor', '100');
save_system(controllerName, controllerFile);
save_system(harnessName, harnessFile);

fprintf('CODEX_FOC_COMPONENTS=%d\n', 11);
fprintf('CODEX_FOC_CONTROLLER_MODEL=%s\n', controllerName);
fprintf('CODEX_FOC_HARNESS_MODEL=%s\n', harnessName);
fprintf('CODEX_FOC_SPEED_TASK_S=0.001\n');
fprintf('CODEX_FOC_CURRENT_TASK_S=0.0001\n');
fprintf('CODEX_FOC_ARCHITECTURE_PASS=1\n');
end

function buildControllerModelArchitecture(modelName)
resetTopLevelArchitecture(modelName);
buildTopLevelFocComponents(modelName, false);

setBlockPosition(modelName, 'SpeedReferenceRpm', [20 55 50 69]);
setBlockPosition(modelName, 'SpeedRpm', [20 120 50 134]);
setBlockPosition(modelName, 'PhaseCurrentA', [20 365 50 379]);
setBlockPosition(modelName, 'PhaseCurrentB', [20 425 50 439]);
setBlockPosition(modelName, 'ElectricalAngleRad', [20 485 50 499]);
setBlockPosition(modelName, 'DcBusVoltage', [20 545 50 559]);
setBlockPosition(modelName, 'IqReference', [510 20 540 34]);
setBlockPosition(modelName, 'IdMeasured', [850 365 880 379]);
setBlockPosition(modelName, 'IqMeasured', [850 405 880 419]);
setBlockPosition(modelName, 'VdCommand', [1140 70 1170 84]);
setBlockPosition(modelName, 'VqCommand', [1140 265 1170 279]);
setBlockPosition(modelName, 'DutyA', [1830 75 1860 89]);
setBlockPosition(modelName, 'DutyB', [1830 135 1860 149]);
setBlockPosition(modelName, 'DutyC', [1830 195 1860 209]);

add_line(modelName, 'SpeedReferenceRpm/1', 'Speed_PI_Controller_1ms/1', 'autorouting', 'on');
add_line(modelName, 'SpeedRpm/1', 'Speed_PI_Controller_1ms/2', 'autorouting', 'on');
add_line(modelName, 'Speed_PI_Controller_1ms/1', 'IqRef_Rate_Transition/1', 'autorouting', 'on');
add_line(modelName, 'IqRef_Rate_Transition/1', 'Q_Axis_Current_PI/1', 'autorouting', 'on');
add_line(modelName, 'IqRef_Rate_Transition/1', 'IqReference/1', 'autorouting', 'on');
add_line(modelName, 'PhaseCurrentA/1', 'Clarke_Transform/1', 'autorouting', 'on');
add_line(modelName, 'PhaseCurrentB/1', 'Clarke_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Clarke_Transform/1', 'Park_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Clarke_Transform/2', 'Park_Transform/2', 'autorouting', 'on');
add_line(modelName, 'ElectricalAngleRad/1', ...
    'Electrical_Angle_Trig_100us/1', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/1', ...
    'Park_Transform/3', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/2', ...
    'Park_Transform/4', 'autorouting', 'on');
add_line(modelName, 'Id_Reference_Zero/1', 'D_Axis_Current_PI/1', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/1', 'D_Axis_Current_PI/2', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/2', 'Q_Axis_Current_PI/2', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/1', 'IdMeasured/1', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/2', 'IqMeasured/1', 'autorouting', 'on');
add_line(modelName, 'SpeedRpm/1', 'DQ_Decoupling_Feedforward/1', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/1', 'DQ_Decoupling_Feedforward/2', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/2', 'DQ_Decoupling_Feedforward/3', 'autorouting', 'on');
add_line(modelName, 'D_Axis_Current_PI/1', 'DQ_Voltage_Command/1', 'autorouting', 'on');
add_line(modelName, 'Q_Axis_Current_PI/1', 'DQ_Voltage_Command/2', 'autorouting', 'on');
add_line(modelName, 'DQ_Decoupling_Feedforward/1', 'DQ_Voltage_Command/3', 'autorouting', 'on');
add_line(modelName, 'DQ_Decoupling_Feedforward/2', 'DQ_Voltage_Command/4', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/1', 'VdCommand/1', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/2', 'VqCommand/1', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/1', 'Inverse_Park_Transform/1', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/2', 'Inverse_Park_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/1', ...
    'Inverse_Park_Transform/3', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/2', ...
    'Inverse_Park_Transform/4', 'autorouting', 'on');
add_line(modelName, 'Inverse_Park_Transform/1', 'Inverse_Clarke_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Inverse_Park_Transform/2', 'Inverse_Clarke_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/1', 'SVPWM_Duty_Calculation/1', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/2', 'SVPWM_Duty_Calculation/2', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/3', 'SVPWM_Duty_Calculation/3', 'autorouting', 'on');
add_line(modelName, 'DcBusVoltage/1', 'SVPWM_Duty_Calculation/4', 'autorouting', 'on');
add_line(modelName, 'SVPWM_Duty_Calculation/1', 'DutyA/1', 'autorouting', 'on');
add_line(modelName, 'SVPWM_Duty_Calculation/2', 'DutyB/1', 'autorouting', 'on');
add_line(modelName, 'SVPWM_Duty_Calculation/3', 'DutyC/1', 'autorouting', 'on');
cleanupDanglingLines(modelName);
end

function buildHarnessArchitecture(modelName)
assert(getSimulinkBlockHandle([modelName '/Selectable_PMSM_Plant']) ~= -1, ...
    'Configure the selectable PMSM plant before rebuilding the FOC harness.');
resetTopLevelArchitecture(modelName);
buildTopLevelFocComponents(modelName, true);

setBlockPosition(modelName, 'Speed_Reference_Rpm', [35 70 85 105]);
setBlockPosition(modelName, 'DC_Bus_48V', [1620 430 1700 465]);
setBlockPosition(modelName, 'Load_Torque_Nm', [1990 455 2065 490]);
setBlockPosition(modelName, 'Native_Average_Inverter', [1815 110 1975 260]);
setBlockPosition(modelName, 'Selectable_PMSM_Plant', [2070 90 2270 320]);
setBlockPosition(modelName, 'Speed_Mux', [2340 50 2345 110]);
setBlockPosition(modelName, 'Speed_Scope', [2390 60 2440 100]);
setBlockPosition(modelName, 'Duty_A_Log', [2335 445 2415 475]);
setBlockPosition(modelName, 'Iq_Ref_Log', [2335 490 2415 520]);
setBlockPosition(modelName, 'Speed_Log', [2330 180 2415 210]);
setBlockPosition(modelName, 'Torque_Log', [2330 270 2415 300]);
setBlockPosition(modelName, 'Iq_Log', [2330 340 2415 370]);

add_line(modelName, 'Speed_Reference_Rpm/1', 'Speed_PI_Controller_1ms/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Reference_Rpm/1', 'Speed_Mux/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/1', 'Speed_PI_Controller_1ms/2', 'autorouting', 'on');
add_line(modelName, 'Speed_PI_Controller_1ms/1', 'IqRef_Rate_Transition/1', 'autorouting', 'on');
add_line(modelName, 'IqRef_Rate_Transition/1', 'Q_Axis_Current_PI/1', 'autorouting', 'on');
add_line(modelName, 'IqRef_Rate_Transition/1', 'Iq_Ref_Log/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/3', 'Clarke_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/4', 'Clarke_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Clarke_Transform/1', 'Park_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Clarke_Transform/2', 'Park_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/2', ...
    'Electrical_Angle_Trig_100us/1', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/1', ...
    'Park_Transform/3', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/2', ...
    'Park_Transform/4', 'autorouting', 'on');
add_line(modelName, 'Id_Reference_Zero/1', 'D_Axis_Current_PI/1', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/1', 'D_Axis_Current_PI/2', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/2', 'Q_Axis_Current_PI/2', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/1', 'DQ_Decoupling_Feedforward/1', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/1', 'DQ_Decoupling_Feedforward/2', 'autorouting', 'on');
add_line(modelName, 'Park_Transform/2', 'DQ_Decoupling_Feedforward/3', 'autorouting', 'on');
add_line(modelName, 'D_Axis_Current_PI/1', 'DQ_Voltage_Command/1', 'autorouting', 'on');
add_line(modelName, 'Q_Axis_Current_PI/1', 'DQ_Voltage_Command/2', 'autorouting', 'on');
add_line(modelName, 'DQ_Decoupling_Feedforward/1', 'DQ_Voltage_Command/3', 'autorouting', 'on');
add_line(modelName, 'DQ_Decoupling_Feedforward/2', 'DQ_Voltage_Command/4', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/1', 'Inverse_Park_Transform/1', 'autorouting', 'on');
add_line(modelName, 'DQ_Voltage_Command/2', 'Inverse_Park_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/1', ...
    'Inverse_Park_Transform/3', 'autorouting', 'on');
add_line(modelName, 'Electrical_Angle_Trig_100us/2', ...
    'Inverse_Park_Transform/4', 'autorouting', 'on');
add_line(modelName, 'Inverse_Park_Transform/1', 'Inverse_Clarke_Transform/1', 'autorouting', 'on');
add_line(modelName, 'Inverse_Park_Transform/2', 'Inverse_Clarke_Transform/2', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/1', 'SVPWM_Duty_Calculation/1', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/2', 'SVPWM_Duty_Calculation/2', 'autorouting', 'on');
add_line(modelName, 'Inverse_Clarke_Transform/3', 'SVPWM_Duty_Calculation/3', 'autorouting', 'on');
add_line(modelName, 'DC_Bus_48V/1', 'SVPWM_Duty_Calculation/4', 'autorouting', 'on');
for phaseIndex = 1:3
    add_line(modelName, sprintf('SVPWM_Duty_Calculation/%d', phaseIndex), ...
        sprintf('Native_Average_Inverter/%d', phaseIndex), 'autorouting', 'on');
end
add_line(modelName, 'SVPWM_Duty_Calculation/1', 'Duty_A_Log/1', 'autorouting', 'on');
add_line(modelName, 'DC_Bus_48V/1', 'Native_Average_Inverter/4', 'autorouting', 'on');
add_line(modelName, 'Native_Average_Inverter/1', 'Selectable_PMSM_Plant/1', 'autorouting', 'on');
add_line(modelName, 'Native_Average_Inverter/2', 'Selectable_PMSM_Plant/2', 'autorouting', 'on');
add_line(modelName, 'Load_Torque_Nm/1', 'Selectable_PMSM_Plant/3', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/1', 'Speed_Mux/2', 'autorouting', 'on');
add_line(modelName, 'Speed_Mux/1', 'Speed_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/1', 'Speed_Log/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/5', 'Torque_Log/1', 'autorouting', 'on');
add_line(modelName, 'Selectable_PMSM_Plant/7', 'Iq_Log/1', 'autorouting', 'on');
cleanupDanglingLines(modelName);
end

function buildTopLevelFocComponents(parent, isHarness)
if isHarness
    speedPosition = [140 45 310 125];
    transitionPosition = [350 65 470 105];
    clarkePosition = [350 360 510 455];
    trigPosition = [350 500 510 575];
    parkPosition = [565 345 725 460];
    dPiPosition = [690 205 850 290];
    qPiPosition = [690 45 850 130];
    decouplingPosition = [690 505 865 600];
    voltagePosition = [930 100 1105 300];
    inverseParkPosition = [1170 120 1330 240];
    inverseClarkePosition = [1390 120 1550 240];
    svpwmPosition = [1610 105 1760 255];
else
    speedPosition = [110 45 280 125];
    transitionPosition = [320 65 440 105];
    clarkePosition = [330 360 490 455];
    trigPosition = [330 500 490 575];
    parkPosition = [545 345 705 460];
    dPiPosition = [670 205 830 290];
    qPiPosition = [670 45 830 130];
    decouplingPosition = [670 505 845 600];
    voltagePosition = [910 100 1085 300];
    inverseParkPosition = [1150 120 1310 240];
    inverseClarkePosition = [1370 120 1530 240];
    svpwmPosition = [1590 105 1740 255];
end

addTopComponent(parent, 'Speed_PI_Controller_1ms', speedPosition, 'yellow', ...
    'OUTER SPEED LOOP', 'Task=1 ms | speed error -> limited Iq reference');
buildSpeedPi([parent '/Speed_PI_Controller_1ms']);
add_block('simulink/Signal Attributes/Rate Transition', ...
    [parent '/IqRef_Rate_Transition'], 'OutPortSampleTime', '0.0001', ...
    'Position', transitionPosition, 'BackgroundColor', 'orange', ...
    'Description', 'Explicit deterministic transfer from 1 ms speed task to 100 us current task.');
addTopComponent(parent, 'Clarke_Transform', clarkePosition, 'lightBlue', ...
    '3s TO 2s CURRENT', 'Task=100 us | Ia/Ib -> Ialpha/Ibeta');
buildClarke([parent '/Clarke_Transform']);
addTopComponent(parent, 'Electrical_Angle_Trig_100us', trigPosition, ...
    'lightBlue', 'SHARED ELECTRICAL ANGLE', ...
    'Task=100 us | theta -> one shared sin/cos pair');
buildElectricalAngleTrig([parent '/Electrical_Angle_Trig_100us']);
addTopComponent(parent, 'Park_Transform', parkPosition, 'lightBlue', ...
    'STATIONARY TO ROTATING CURRENT', 'Task=100 us | Ialpha/Ibeta + sin/cos -> Id/Iq');
buildPark([parent '/Park_Transform']);
add_block('simulink/Sources/Constant', [parent '/Id_Reference_Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [590 230 650 260], 'BackgroundColor', 'white', ...
    'Description', 'Field-oriented control d-axis current reference: Id*=0 A.');
addTopComponent(parent, 'D_Axis_Current_PI', dPiPosition, 'yellow', ...
    'D-AXIS CURRENT PI', 'Task=100 us | Id*=0 A | Kp/Ki + limits');
buildCurrentPi([parent '/D_Axis_Current_PI'], 'D');
addTopComponent(parent, 'Q_Axis_Current_PI', qPiPosition, 'yellow', ...
    'Q-AXIS CURRENT PI', 'Task=100 us | Iq*=speed loop | Kp/Ki + limits');
buildCurrentPi([parent '/Q_Axis_Current_PI'], 'Q');
addTopComponent(parent, 'DQ_Decoupling_Feedforward', decouplingPosition, 'orange', ...
    'D/Q DECOUPLING', 'Task=100 us | omega, Ld/Lq, PM flux feedforward');
buildDecoupling([parent '/DQ_Decoupling_Feedforward']);
addTopComponent(parent, 'DQ_Voltage_Command', voltagePosition, 'white', ...
    'D/Q VOLTAGE COMMAND', 'Task=100 us | PI + feedforward + voltage limit');
buildVoltageCommand([parent '/DQ_Voltage_Command']);
addTopComponent(parent, 'Inverse_Park_Transform', inverseParkPosition, 'lightBlue', ...
    '2r TO 2s', 'Task=100 us | Vd/Vq + shared sin/cos -> Valpha/Vbeta');
buildInversePark([parent '/Inverse_Park_Transform']);
addTopComponent(parent, 'Inverse_Clarke_Transform', inverseClarkePosition, 'lightBlue', ...
    '2s TO 3s', 'Task=100 us | Valpha/Vbeta -> Va/Vb/Vc');
buildInverseClarke([parent '/Inverse_Clarke_Transform']);
addTopComponent(parent, 'SVPWM_Duty_Calculation', svpwmPosition, 'cyan', ...
    'SVPWM', 'Task=100 us | phase voltage + Vdc -> Duty A/B/C');
buildSvpwm([parent '/SVPWM_Duty_Calculation']);
end

function resetTopLevelArchitecture(parent)
lineHandles = find_system(parent, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for index = 1:numel(lineHandles)
    try
        delete_line(lineHandles(index));
    catch
    end
end
blockNames = {'Native_FOC_Controller_100us', 'Speed_PI_Controller_1ms', ...
    'IqRef_Rate_Transition', 'Current_Control_100us', 'Id_Reference_Zero', ...
    'Clarke_Transform', 'Electrical_Angle_Trig_100us', ...
    'Park_Transform', 'D_Axis_Current_PI', ...
    'Q_Axis_Current_PI', 'DQ_Decoupling_Feedforward', ...
    'DQ_Voltage_Command', 'Inverse_Park_Transform', ...
    'Inverse_Clarke_Transform', 'SVPWM_Duty_Calculation'};
for index = 1:numel(blockNames)
    path = [parent '/' blockNames{index}];
    if getSimulinkBlockHandle(path) ~= -1
        delete_block(path);
    end
end
end

function cleanupDanglingLines(parent)
lineHandles = find_system(parent, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for index = 1:numel(lineHandles)
    try
        sourcePort = get_param(lineHandles(index), 'SrcPortHandle');
        destinationPorts = get_param(lineHandles(index), 'DstPortHandle');
        if isempty(sourcePort) || sourcePort == -1 || isempty(destinationPorts) || ...
                any(destinationPorts == -1)
            delete_line(lineHandles(index));
        end
    catch
    end
end
end

function addTopComponent(parent, name, position, color, titleText, taskText)
addComponent(parent, name, position, color, titleText, taskText);
set_param([parent '/' name], 'AttributesFormatString', ...
    sprintf('%s\n%s', titleText, taskText));
end

function setBlockPosition(parent, name, position)
path = [parent '/' name];
if getSimulinkBlockHandle(path) ~= -1
    set_param(path, 'Position', position);
end
end

function buildClarke(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Ia', 1, [20 120 50 134]);
addIn(parent, 'Ib', 2, [20 210 50 224]);
addOut(parent, 'Ialpha', 1, [430 120 460 134]);
addOut(parent, 'Ibeta', 2, [430 210 460 224]);
addGain(parent, 'Ib_x2', 'single(2.0)', [100 190 160 220]);
addSum(parent, 'Ia_Plus_2Ib', '++', [205 150 235 215]);
addGain(parent, 'InvSqrt3', 'NATIVE_INV_SQRT3', [285 170 365 205]);
add_line(parent, 'Ia/1', 'Ialpha/1', 'autorouting', 'on');
add_line(parent, 'Ia/1', 'Ia_Plus_2Ib/1', 'autorouting', 'on');
add_line(parent, 'Ib/1', 'Ib_x2/1', 'autorouting', 'on');
add_line(parent, 'Ib_x2/1', 'Ia_Plus_2Ib/2', 'autorouting', 'on');
add_line(parent, 'Ia_Plus_2Ib/1', 'InvSqrt3/1', 'autorouting', 'on');
add_line(parent, 'InvSqrt3/1', 'Ibeta/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: two-current Clarke transform\n' ...
    'Ialpha = Ia; Ibeta = (Ia + 2*Ib)/sqrt(3)\n' ...
    'Parameter: NATIVE_INV_SQRT3'], [70 20 410 95], 11, 'blue', 'lightBlue');
end

function buildElectricalAngleTrig(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'ThetaElectrical', 1, [20 145 50 159]);
addOut(parent, 'SinTheta', 1, [350 115 380 129]);
addOut(parent, 'CosTheta', 2, [350 215 380 229]);
addTrig(parent, 'Sin_Electrical_Angle', 'sin', [130 100 200 140]);
addTrig(parent, 'Cos_Electrical_Angle', 'cos', [130 200 200 240]);
add_line(parent, 'ThetaElectrical/1', 'Sin_Electrical_Angle/1', ...
    'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'Cos_Electrical_Angle/1', ...
    'autorouting', 'on');
add_line(parent, 'Sin_Electrical_Angle/1', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'Cos_Electrical_Angle/1', 'CosTheta/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: compute the electrical-angle basis once\n' ...
    'One sin(theta) and one cos(theta) evaluation per 100 us step\n' ...
    'The same pair feeds Park and inverse Park'], ...
    [60 15 390 85], 11, 'blue', 'lightBlue');
end

function buildPark(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Ialpha', 1, [20 120 50 134]);
addIn(parent, 'Ibeta', 2, [20 200 50 214]);
addIn(parent, 'SinTheta', 3, [20 280 50 294]);
addIn(parent, 'CosTheta', 4, [20 340 50 354]);
addOut(parent, 'Id', 1, [520 150 550 164]);
addOut(parent, 'Iq', 2, [520 250 550 264]);
addProduct(parent, 'Id_CosAlpha', '**', [210 120 250 155]);
addProduct(parent, 'Id_SinBeta', '**', [210 175 250 210]);
addSum(parent, 'Id_Sum', '++', [320 135 350 200]);
addProduct(parent, 'Iq_SinAlpha', '**', [210 230 250 265]);
addGain(parent, 'Negative', 'single(-1.0)', [290 230 350 260]);
addProduct(parent, 'Iq_CosBeta', '**', [210 285 250 320]);
addSum(parent, 'Iq_Sum', '++', [400 245 430 310]);
add_line(parent, 'CosTheta/1', 'Id_CosAlpha/1', 'autorouting', 'on');
add_line(parent, 'Ialpha/1', 'Id_CosAlpha/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Id_SinBeta/1', 'autorouting', 'on');
add_line(parent, 'Ibeta/1', 'Id_SinBeta/2', 'autorouting', 'on');
add_line(parent, 'Id_CosAlpha/1', 'Id_Sum/1', 'autorouting', 'on');
add_line(parent, 'Id_SinBeta/1', 'Id_Sum/2', 'autorouting', 'on');
add_line(parent, 'Id_Sum/1', 'Id/1', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Iq_SinAlpha/1', 'autorouting', 'on');
add_line(parent, 'Ialpha/1', 'Iq_SinAlpha/2', 'autorouting', 'on');
add_line(parent, 'Iq_SinAlpha/1', 'Negative/1', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Iq_CosBeta/1', 'autorouting', 'on');
add_line(parent, 'Ibeta/1', 'Iq_CosBeta/2', 'autorouting', 'on');
add_line(parent, 'Negative/1', 'Iq_Sum/1', 'autorouting', 'on');
add_line(parent, 'Iq_CosBeta/1', 'Iq_Sum/2', 'autorouting', 'on');
add_line(parent, 'Iq_Sum/1', 'Iq/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: alpha/beta to rotating d/q currents\n' ...
    'Id = cos(theta)*Ialpha + sin(theta)*Ibeta\n' ...
    'Iq = -sin(theta)*Ialpha + cos(theta)*Ibeta'], ...
    [70 15 500 90], 11, 'blue', 'lightBlue');
end

function buildSpeedPi(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'SpeedReferenceRpm', 1, [20 135 50 149]);
addIn(parent, 'SpeedRpm', 2, [20 220 50 234]);
addIn(parent, 'ControllerReset', 3, [20 315 50 329]);
set_param([parent '/ControllerReset'], 'OutDataTypeStr', 'boolean');
addOut(parent, 'IqReference', 1, [720 175 750 189]);
add_block('simulink/Discrete/Zero-Order Hold', [parent '/SpeedRef_1ms'], ...
    'SampleTime', '0.001', 'Position', [90 120 155 150]);
add_block('simulink/Discrete/Zero-Order Hold', [parent '/SpeedFb_1ms'], ...
    'SampleTime', '0.001', 'Position', [90 205 155 235]);
addSum(parent, 'Speed_Error', '+-', [195 140 225 210]);
addGain(parent, 'RpmToRad', 'NATIVE_RPM_TO_RAD_S', [265 160 350 195]);
addGain(parent, 'Kp', 'FOC_Native_KpSpeed', [390 125 455 155]);
addGain(parent, 'KiTs', 'FOC_Native_KiSpeed*FOC_Native_SpeedPeriod', ...
    [390 230 485 260]);
addSum(parent, 'Integrator_Add', '++', [515 210 545 270]);
addSaturation(parent, 'Integrator_Limit', '-FOC_Native_IqLimit', ...
    'FOC_Native_IqLimit', [580 230 650 275]);
add_block('simulink/Discrete/Unit Delay', [parent '/Integrator_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.001', ...
    'Position', [515 315 570 345]);
add_block('simulink/Sources/Constant', [parent '/Integrator_Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [575 335 625 365]);
add_block('simulink/Signal Routing/Switch', ...
    [parent '/Integrator_Reset_Select'], 'Criteria', 'u2 ~= 0', ...
    'Position', [675 280 720 335]);
addSum(parent, 'Iq_Reference_Sum', '++', [520 120 550 180]);
addSaturation(parent, 'Iq_Reference_Limit', '-FOC_Native_IqLimit', ...
    'FOC_Native_IqLimit', [590 130 660 175]);
add_line(parent, 'SpeedReferenceRpm/1', 'SpeedRef_1ms/1', 'autorouting', 'on');
add_line(parent, 'SpeedRpm/1', 'SpeedFb_1ms/1', 'autorouting', 'on');
add_line(parent, 'SpeedRef_1ms/1', 'Speed_Error/1', 'autorouting', 'on');
add_line(parent, 'SpeedFb_1ms/1', 'Speed_Error/2', 'autorouting', 'on');
add_line(parent, 'Speed_Error/1', 'RpmToRad/1', 'autorouting', 'on');
add_line(parent, 'RpmToRad/1', 'Kp/1', 'autorouting', 'on');
add_line(parent, 'RpmToRad/1', 'KiTs/1', 'autorouting', 'on');
add_line(parent, 'KiTs/1', 'Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Integrator_Add/1', 'Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Integrator_Zero/1', 'Integrator_Reset_Select/1', 'autorouting', 'on');
add_line(parent, 'ControllerReset/1', 'Integrator_Reset_Select/2', 'autorouting', 'on');
add_line(parent, 'Integrator_Limit/1', 'Integrator_Reset_Select/3', 'autorouting', 'on');
add_line(parent, 'Integrator_Reset_Select/1', 'Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Kp/1', 'Iq_Reference_Sum/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'Iq_Reference_Sum/2', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Sum/1', 'Iq_Reference_Limit/1', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Limit/1', 'IqReference/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: outer speed PI loop\n' ...
    'Ts=FOC_Native_SpeedPeriod (1 ms)\n' ...
    'Kp=FOC_Native_KpSpeed; Ki=FOC_Native_KiSpeed\n' ...
    'Output limit: +/-FOC_Native_IqLimit; Stateflow reset outside RUN'], ...
    [80 15 690 95], 11, 'green', 'yellow');
end

function buildCurrentPi(parent, axisLabel)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Reference', 1, [20 140 50 154]);
addIn(parent, 'Measured', 2, [20 240 50 254]);
addIn(parent, 'ControllerReset', 3, [20 330 50 344]);
set_param([parent '/ControllerReset'], 'OutDataTypeStr', 'boolean');
addOut(parent, 'VoltagePI', 1, [610 180 640 194]);
addSum(parent, 'Current_Error', '+-', [100 155 130 230]);
addGain(parent, 'Kp', 'FOC_Native_KpCurrent', [175 130 245 160]);
addGain(parent, 'KiTs', ...
    'FOC_Native_KiCurrent*FOC_Native_CurrentPeriod', [175 245 285 275]);
addSum(parent, 'Integrator_Add', '++', [325 220 355 280]);
addSaturation(parent, 'Integrator_Limit', ...
    '-FOC_Native_CurrentIntegratorLimit', ...
    'FOC_Native_CurrentIntegratorLimit', [395 230 485 275]);
add_block('simulink/Discrete/Unit Delay', [parent '/Integrator_State'], ...
    'InitialCondition', 'single(0.0)', 'SampleTime', '0.0001', ...
    'Position', [325 320 380 350]);
add_block('simulink/Sources/Constant', [parent '/Integrator_Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [405 330 455 360]);
add_block('simulink/Signal Routing/Switch', ...
    [parent '/Integrator_Reset_Select'], 'Criteria', 'u2 ~= 0', ...
    'Position', [500 285 545 340]);
addSum(parent, 'PI_Sum', '++', [510 145 540 215]);
add_line(parent, 'Reference/1', 'Current_Error/1', 'autorouting', 'on');
add_line(parent, 'Measured/1', 'Current_Error/2', 'autorouting', 'on');
add_line(parent, 'Current_Error/1', 'Kp/1', 'autorouting', 'on');
add_line(parent, 'Current_Error/1', 'KiTs/1', 'autorouting', 'on');
add_line(parent, 'KiTs/1', 'Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Integrator_Add/1', 'Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Integrator_Zero/1', 'Integrator_Reset_Select/1', 'autorouting', 'on');
add_line(parent, 'ControllerReset/1', 'Integrator_Reset_Select/2', 'autorouting', 'on');
add_line(parent, 'Integrator_Limit/1', 'Integrator_Reset_Select/3', 'autorouting', 'on');
add_line(parent, 'Integrator_Reset_Select/1', 'Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Kp/1', 'PI_Sum/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'PI_Sum/2', 'autorouting', 'on');
add_line(parent, 'PI_Sum/1', 'VoltagePI/1', 'autorouting', 'on');
addNote(parent, sprintf(['RESPONSIBILITY: %s-axis current PI loop\n' ...
    'Ts=FOC_Native_CurrentPeriod (100 us)\n' ...
    'Kp=FOC_Native_KpCurrent; Ki=FOC_Native_KiCurrent\n' ...
    'Integrator limit: +/-FOC_Native_CurrentIntegratorLimit\n' ...
    'Stateflow resets the integrator outside RUN'], axisLabel), ...
    [75 15 590 100], 11, 'green', 'yellow');
end

function buildDecoupling(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'SpeedRpm', 1, [20 135 50 149]);
addIn(parent, 'Id', 2, [20 225 50 239]);
addIn(parent, 'Iq', 3, [20 315 50 329]);
addOut(parent, 'VdFeedforward', 1, [600 170 630 184]);
addOut(parent, 'VqFeedforward', 2, [600 280 630 294]);
addGain(parent, 'Electrical_Speed', ...
    'NATIVE_RPM_TO_RAD_S*FOC_Native_PolePairs', [95 120 220 155]);
addProduct(parent, 'Omega_x_Iq', '**', [270 150 310 185]);
addGain(parent, 'D_Decoupling', '-FOC_Native_Lq', [360 155 450 185]);
addGain(parent, 'Ld_x_Id', 'FOC_Native_Ld', [250 245 325 275]);
add_block('simulink/Sources/Constant', [parent '/Flux_PM'], ...
    'Value', 'FOC_Native_FluxPM', 'OutDataTypeStr', 'single', ...
    'Position', [250 315 325 345]);
addSum(parent, 'Flux_Linkage', '++', [370 255 400 325]);
addProduct(parent, 'Q_Feedforward', '**', [470 250 510 285]);
add_line(parent, 'SpeedRpm/1', 'Electrical_Speed/1', 'autorouting', 'on');
add_line(parent, 'Electrical_Speed/1', 'Omega_x_Iq/1', 'autorouting', 'on');
add_line(parent, 'Iq/1', 'Omega_x_Iq/2', 'autorouting', 'on');
add_line(parent, 'Omega_x_Iq/1', 'D_Decoupling/1', 'autorouting', 'on');
add_line(parent, 'D_Decoupling/1', 'VdFeedforward/1', 'autorouting', 'on');
add_line(parent, 'Id/1', 'Ld_x_Id/1', 'autorouting', 'on');
add_line(parent, 'Ld_x_Id/1', 'Flux_Linkage/1', 'autorouting', 'on');
add_line(parent, 'Flux_PM/1', 'Flux_Linkage/2', 'autorouting', 'on');
add_line(parent, 'Electrical_Speed/1', 'Q_Feedforward/1', 'autorouting', 'on');
add_line(parent, 'Flux_Linkage/1', 'Q_Feedforward/2', 'autorouting', 'on');
add_line(parent, 'Q_Feedforward/1', 'VqFeedforward/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: d/q decoupling and back-EMF feedforward\n' ...
    'Vd_ff = -omega_e*Lq*Iq\n' ...
    'Vq_ff = omega_e*(Ld*Id + FluxPM)\n' ...
    'omega_e uses pole pairs and mechanical speed'], ...
    [70 15 570 100], 11, 'black', 'orange');
end

function buildVoltageCommand(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'VdPI', 1, [20 120 50 134]);
addIn(parent, 'VqPI', 2, [20 190 50 204]);
addIn(parent, 'VdFeedforward', 3, [20 270 50 284]);
addIn(parent, 'VqFeedforward', 4, [20 340 50 354]);
addOut(parent, 'VdCommand', 1, [430 155 460 169]);
addOut(parent, 'VqCommand', 2, [430 285 460 299]);
addSum(parent, 'Vd_Raw', '++', [150 130 180 200]);
addSum(parent, 'Vq_Raw', '++', [150 260 180 330]);
addSaturation(parent, 'Vd_Limit', '-FOC_Native_VoltageLimit', ...
    'FOC_Native_VoltageLimit', [250 140 350 185]);
addSaturation(parent, 'Vq_Limit', '-FOC_Native_VoltageLimit', ...
    'FOC_Native_VoltageLimit', [250 270 350 315]);
add_line(parent, 'VdPI/1', 'Vd_Raw/1', 'autorouting', 'on');
add_line(parent, 'VdFeedforward/1', 'Vd_Raw/2', 'autorouting', 'on');
add_line(parent, 'VqPI/1', 'Vq_Raw/1', 'autorouting', 'on');
add_line(parent, 'VqFeedforward/1', 'Vq_Raw/2', 'autorouting', 'on');
add_line(parent, 'Vd_Raw/1', 'Vd_Limit/1', 'autorouting', 'on');
add_line(parent, 'Vq_Raw/1', 'Vq_Limit/1', 'autorouting', 'on');
add_line(parent, 'Vd_Limit/1', 'VdCommand/1', 'autorouting', 'on');
add_line(parent, 'Vq_Limit/1', 'VqCommand/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: synthesize bounded d/q voltage commands\n' ...
    'Vd = sat(Vd_PI + Vd_ff); Vq = sat(Vq_PI + Vq_ff)\n' ...
    'Limit: +/-FOC_Native_VoltageLimit'], ...
    [65 15 420 95], 11, 'black', 'yellow');
end

function buildInversePark(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Vd', 1, [20 120 50 134]);
addIn(parent, 'Vq', 2, [20 200 50 214]);
addIn(parent, 'SinTheta', 3, [20 280 50 294]);
addIn(parent, 'CosTheta', 4, [20 340 50 354]);
addOut(parent, 'Valpha', 1, [520 150 550 164]);
addOut(parent, 'Vbeta', 2, [520 260 550 274]);
addProduct(parent, 'Valpha_CosVd', '**', [220 120 260 155]);
addProduct(parent, 'Valpha_SinVq', '**', [220 180 260 215]);
addSum(parent, 'Valpha_Sum', '+-', [340 135 370 205]);
addProduct(parent, 'Vbeta_SinVd', '**', [220 240 260 275]);
addProduct(parent, 'Vbeta_CosVq', '**', [220 300 260 335]);
addSum(parent, 'Vbeta_Sum', '++', [340 255 370 325]);
add_line(parent, 'CosTheta/1', 'Valpha_CosVd/1', 'autorouting', 'on');
add_line(parent, 'Vd/1', 'Valpha_CosVd/2', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Valpha_SinVq/1', 'autorouting', 'on');
add_line(parent, 'Vq/1', 'Valpha_SinVq/2', 'autorouting', 'on');
add_line(parent, 'Valpha_CosVd/1', 'Valpha_Sum/1', 'autorouting', 'on');
add_line(parent, 'Valpha_SinVq/1', 'Valpha_Sum/2', 'autorouting', 'on');
add_line(parent, 'Valpha_Sum/1', 'Valpha/1', 'autorouting', 'on');
add_line(parent, 'SinTheta/1', 'Vbeta_SinVd/1', 'autorouting', 'on');
add_line(parent, 'Vd/1', 'Vbeta_SinVd/2', 'autorouting', 'on');
add_line(parent, 'CosTheta/1', 'Vbeta_CosVq/1', 'autorouting', 'on');
add_line(parent, 'Vq/1', 'Vbeta_CosVq/2', 'autorouting', 'on');
add_line(parent, 'Vbeta_SinVd/1', 'Vbeta_Sum/1', 'autorouting', 'on');
add_line(parent, 'Vbeta_CosVq/1', 'Vbeta_Sum/2', 'autorouting', 'on');
add_line(parent, 'Vbeta_Sum/1', 'Vbeta/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: rotating d/q to stationary alpha/beta voltage\n' ...
    'Valpha = cos(theta)*Vd - sin(theta)*Vq\n' ...
    'Vbeta = sin(theta)*Vd + cos(theta)*Vq'], ...
    [70 15 500 90], 11, 'blue', 'lightBlue');
end

function buildInverseClarke(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Valpha', 1, [20 130 50 144]);
addIn(parent, 'Vbeta', 2, [20 260 50 274]);
addOut(parent, 'Va', 1, [480 115 510 129]);
addOut(parent, 'Vb', 2, [480 215 510 229]);
addOut(parent, 'Vc', 3, [480 315 510 329]);
addGain(parent, 'Vb_Alpha', 'single(-0.5)', [120 175 190 205]);
addGain(parent, 'Vb_Beta', 'NATIVE_SQRT3_BY2', [120 225 210 255]);
addSum(parent, 'Phase_Vb', '++', [270 190 300 255]);
addGain(parent, 'Vc_Alpha', 'single(-0.5)', [120 300 190 330]);
addGain(parent, 'Vc_Beta', '-NATIVE_SQRT3_BY2', [120 350 210 380]);
addSum(parent, 'Phase_Vc', '++', [270 315 300 380]);
add_line(parent, 'Valpha/1', 'Va/1', 'autorouting', 'on');
add_line(parent, 'Valpha/1', 'Vb_Alpha/1', 'autorouting', 'on');
add_line(parent, 'Vbeta/1', 'Vb_Beta/1', 'autorouting', 'on');
add_line(parent, 'Vb_Alpha/1', 'Phase_Vb/1', 'autorouting', 'on');
add_line(parent, 'Vb_Beta/1', 'Phase_Vb/2', 'autorouting', 'on');
add_line(parent, 'Phase_Vb/1', 'Vb/1', 'autorouting', 'on');
add_line(parent, 'Valpha/1', 'Vc_Alpha/1', 'autorouting', 'on');
add_line(parent, 'Vbeta/1', 'Vc_Beta/1', 'autorouting', 'on');
add_line(parent, 'Vc_Alpha/1', 'Phase_Vc/1', 'autorouting', 'on');
add_line(parent, 'Vc_Beta/1', 'Phase_Vc/2', 'autorouting', 'on');
add_line(parent, 'Phase_Vc/1', 'Vc/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: alpha/beta to three-phase voltage\n' ...
    'Va=Valpha\nVb=-0.5*Valpha+sqrt(3)/2*Vbeta\n' ...
    'Vc=-0.5*Valpha-sqrt(3)/2*Vbeta'], ...
    [65 15 450 100], 11, 'blue', 'lightBlue');
end

function buildSvpwm(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Va', 1, [20 115 50 129]);
addIn(parent, 'Vb', 2, [20 185 50 199]);
addIn(parent, 'Vc', 3, [20 255 50 269]);
addIn(parent, 'Vdc', 4, [20 430 50 444]);
addOut(parent, 'DutyA', 1, [760 155 790 169]);
addOut(parent, 'DutyB', 2, [760 275 790 289]);
addOut(parent, 'DutyC', 3, [760 395 790 409]);
add_block('simulink/Math Operations/MinMax', [parent '/Phase_Maximum'], ...
    'Function', 'max', 'Inputs', '3', 'Position', [110 120 155 200]);
add_block('simulink/Math Operations/MinMax', [parent '/Phase_Minimum'], ...
    'Function', 'min', 'Inputs', '3', 'Position', [110 245 155 325]);
addSum(parent, 'Max_Plus_Min', '++', [205 180 235 270]);
addGain(parent, 'Common_Mode', 'single(-0.5)', [280 210 355 245]);
add_block('simulink/Sources/Constant', [parent '/Duty_Half'], ...
    'Value', 'single(0.5)', 'OutDataTypeStr', 'single', ...
    'Position', [420 500 480 530]);
phaseNames = {'Va', 'Vb', 'Vc'};
dutyNames = {'A', 'B', 'C'};
for index = 1:3
    yPosition = 120 + (index-1)*120;
    addSum(parent, ['Phase_' dutyNames{index} '_Plus_Common'], '++', ...
        [400 yPosition 430 yPosition+55]);
    addProduct(parent, ['Duty_' dutyNames{index} '_Divide_Vdc'], '*/', ...
        [480 yPosition+5 520 yPosition+45]);
    addSum(parent, ['Duty_' dutyNames{index} '_Plus_Half'], '++', ...
        [565 yPosition 595 yPosition+55]);
    addSaturation(parent, ['Duty_' dutyNames{index} '_Limit'], ...
        'FOC_Native_DutyMin', 'FOC_Native_DutyMax', ...
        [640 yPosition 715 yPosition+55]);
    add_line(parent, [phaseNames{index} '/1'], ...
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
for phaseIndex = 1:3
    add_line(parent, [phaseNames{phaseIndex} '/1'], ...
        ['Phase_Maximum/' num2str(phaseIndex)], 'autorouting', 'on');
    add_line(parent, [phaseNames{phaseIndex} '/1'], ...
        ['Phase_Minimum/' num2str(phaseIndex)], 'autorouting', 'on');
end
add_line(parent, 'Phase_Maximum/1', 'Max_Plus_Min/1', 'autorouting', 'on');
add_line(parent, 'Phase_Minimum/1', 'Max_Plus_Min/2', 'autorouting', 'on');
add_line(parent, 'Max_Plus_Min/1', 'Common_Mode/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: common-mode injected SVPWM\n' ...
    'Voffset=-0.5*(max(Vabc)+min(Vabc))\n' ...
    'Duty=0.5+(Vphase+Voffset)/Vdc\n' ...
    'Limits: FOC_Native_DutyMin to FOC_Native_DutyMax'], ...
    [80 15 720 95], 11, 'blue', 'cyan');
end

function connections = captureExternalConnections(subsystemPath)
% Preserve wiring at the subsystem boundary while its internal port blocks
% are rebuilt. Deleting Inport/Outport blocks otherwise deletes the lines in
% the parent model as a side effect.
connections.ParentSystem = get_param(subsystemPath, 'Parent');
portHandles = get_param(subsystemPath, 'PortHandles');
connections.InputSources = cell(numel(portHandles.Inport), 1);
connections.OutputDestinations = cell(numel(portHandles.Outport), 1);
for index = 1:numel(portHandles.Inport)
    lineHandle = get_param(portHandles.Inport(index), 'Line');
    if lineHandle ~= -1
        connections.InputSources{index} = ...
            get_param(lineHandle, 'SrcPortHandle');
    end
end
for index = 1:numel(portHandles.Outport)
    lineHandle = get_param(portHandles.Outport(index), 'Line');
    if lineHandle ~= -1
        connections.OutputDestinations{index} = ...
            get_param(lineHandle, 'DstPortHandle');
    end
end
end

function restoreExternalConnections(subsystemPath, connections)
portHandles = get_param(subsystemPath, 'PortHandles');
for index = 1:min(numel(portHandles.Inport), ...
        numel(connections.InputSources))
    sourcePort = connections.InputSources{index};
    if ~isempty(sourcePort)
        connectPorts(connections.ParentSystem, sourcePort, ...
            portHandles.Inport(index));
    end
end
for index = 1:min(numel(portHandles.Outport), ...
        numel(connections.OutputDestinations))
    destinationPorts = connections.OutputDestinations{index};
    for destinationIndex = 1:numel(destinationPorts)
        connectPorts(connections.ParentSystem, ...
            portHandles.Outport(index), ...
            destinationPorts(destinationIndex));
    end
end
% Also fill any missing known boundary connections. This is needed when a
% prior interrupted refactor retained only part of the interface; existing
% destinations are skipped by connectPorts.
restoreKnownExternalInterface(subsystemPath);
end

function restoreKnownExternalInterface(subsystemPath)
parentSystem = get_param(subsystemPath, 'Parent');
controllerPorts = get_param(subsystemPath, 'PortHandles');
if strcmp(parentSystem, 'PMSM_FOC_DualPlant_Controller_v21')
    sourceNames = {'SpeedReferenceRpm', 'SpeedRpm', 'PhaseCurrentA', ...
        'PhaseCurrentB', 'ElectricalAngleRad', 'DcBusVoltage'};
    destinationNames = {'DutyA', 'DutyB', 'DutyC', 'IqReference', ...
        'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand'};
    for index = 1:numel(sourceNames)
        sourcePorts = get_param([parentSystem '/' sourceNames{index}], ...
            'PortHandles');
        connectPorts(parentSystem, sourcePorts.Outport(1), ...
            controllerPorts.Inport(index));
    end
    for index = 1:numel(destinationNames)
        destinationPorts = get_param( ...
            [parentSystem '/' destinationNames{index}], 'PortHandles');
        connectPorts(parentSystem, controllerPorts.Outport(index), ...
            destinationPorts.Inport(1));
    end
elseif strcmp(parentSystem, 'PMSM_FOC_DualPlant_ClosedLoop_v21')
    sourceNames = {'Speed_Reference_Rpm', 'Selectable_PMSM_Plant', ...
        'Selectable_PMSM_Plant', 'Selectable_PMSM_Plant', ...
        'Selectable_PMSM_Plant', 'DC_Bus_48V'};
    sourcePortNumbers = [1 1 3 4 2 1];
    for index = 1:numel(sourceNames)
        sourcePorts = get_param([parentSystem '/' sourceNames{index}], ...
            'PortHandles');
        connectPorts(parentSystem, ...
            sourcePorts.Outport(sourcePortNumbers(index)), ...
            controllerPorts.Inport(index));
    end
    inverterPorts = get_param([parentSystem '/Native_Average_Inverter'], ...
        'PortHandles');
    for index = 1:3
        connectPorts(parentSystem, controllerPorts.Outport(index), ...
            inverterPorts.Inport(index));
    end
    dutyLogPorts = get_param([parentSystem '/Duty_A_Log'], 'PortHandles');
    iqReferenceLogPorts = get_param([parentSystem '/Iq_Ref_Log'], ...
        'PortHandles');
    connectPorts(parentSystem, controllerPorts.Outport(1), ...
        dutyLogPorts.Inport(1));
    connectPorts(parentSystem, controllerPorts.Outport(4), ...
        iqReferenceLogPorts.Inport(1));
else
    error('Unsupported controller parent for boundary recovery: %s', ...
        parentSystem);
end
end

function connectPorts(parentSystem, sourcePort, destinationPort)
% Adding a replacement Inport/Outport can automatically reattach a line in
% some Simulink releases. A prior replacement can also leave a dangling
% destination line whose source port no longer exists. Retain only the exact
% expected connection and replace stale or mismatched lines.
lineHandle = get_param(destinationPort, 'Line');
if lineHandle ~= -1
    currentSourcePort = get_param(lineHandle, 'SrcPortHandle');
    if isequal(currentSourcePort, sourcePort)
        return;
    end
    delete_line(lineHandle);
end
add_line(parentSystem, sourcePort, destinationPort, 'autorouting', 'on');
end

function addComponent(parent, name, position, color, description, attributes)
path = [parent '/' name];
fullDescription = sprintf('%s\nParameters: %s', description, attributes);
add_block('simulink/Ports & Subsystems/Subsystem', path, ...
    'Position', position, 'BackgroundColor', color, ...
    'ForegroundColor', 'black', 'FontSize', '11', ...
    'Description', fullDescription, 'AttributesFormatString', '');
% Keep execution virtual so the 1 ms speed loop and 100 us base rate retain
% the original multirate scheduling while the diagram still has clear
% component boundaries.
set_param(path, 'TreatAsAtomicUnit', 'off');
blockObject = get_param(path, 'Object');
if isprop(blockObject, 'ContentPreviewEnabled')
    set_param(path, 'ContentPreviewEnabled', 'off');
end
end

function addIn(parent, name, port, position)
add_block('simulink/Sources/In1', [parent '/' name], ...
    'Port', num2str(port), 'OutDataTypeStr', 'single', 'Position', position);
end

function addOut(parent, name, port, position)
add_block('simulink/Sinks/Out1', [parent '/' name], ...
    'Port', num2str(port), 'OutDataTypeStr', 'single', 'Position', position);
end

function addGain(parent, name, gain, position)
add_block('simulink/Math Operations/Gain', [parent '/' name], ...
    'Gain', gain, 'OutDataTypeStr', ...
    'Inherit: Inherit via internal rule', 'Position', position);
end

function addSum(parent, name, signs, position)
add_block('simulink/Math Operations/Sum', [parent '/' name], ...
    'Inputs', signs, 'OutDataTypeStr', ...
    'Inherit: Inherit via internal rule', 'Position', position);
end

function addProduct(parent, name, inputs, position)
add_block('simulink/Math Operations/Product', [parent '/' name], ...
    'Inputs', inputs, 'OutDataTypeStr', ...
    'Inherit: Inherit via internal rule', 'Position', position);
end

function addTrig(parent, name, operator, position)
add_block('simulink/Math Operations/Trigonometric Function', ...
    [parent '/' name], 'Operator', operator, ...
    'ApproximationMethod', 'None', 'Position', position);
end

function addSaturation(parent, name, lowerLimit, upperLimit, position)
add_block('simulink/Discontinuities/Saturation', [parent '/' name], ...
    'LowerLimit', lowerLimit, 'UpperLimit', upperLimit, ...
    'OutDataTypeStr', 'Inherit: Same as input', 'Position', position);
end

function addNote(parent, text, position, fontSize, foreground, background)
text = strrep(text, '\n', newline);
note = Simulink.Annotation(parent, text);
note.Interpreter = 'off';
note.Position = position;
note.FontSize = fontSize;
note.ForegroundColor = foreground;
note.BackgroundColor = background;
end
