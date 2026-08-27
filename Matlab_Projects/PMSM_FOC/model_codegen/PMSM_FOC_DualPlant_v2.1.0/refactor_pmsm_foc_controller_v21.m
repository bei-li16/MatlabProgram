function refactor_pmsm_foc_controller_v21
%REFACTOR_PMSM_FOC_CONTROLLER_V21 Build a componentized FOC architecture.
% The external 6-input/8-output interface is preserved. Internally, signal
% transforms, cascaded PI loops, decoupling, voltage synthesis, and SVPWM
% are separated into documented subsystems with explicit responsibilities.

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

controllerPath = [controllerName '/Native_FOC_Controller_100us'];
harnessControllerPath = [harnessName '/Native_FOC_Controller_100us'];
assert(getSimulinkBlockHandle(controllerPath) ~= -1, ...
    'Controller subsystem is missing from the code-generation model.');
assert(getSimulinkBlockHandle(harnessControllerPath) ~= -1, ...
    'Controller subsystem is missing from the closed-loop model.');

buildComponentizedController(controllerPath);
buildComponentizedController(harnessControllerPath);
save_system(controllerName, controllerFile);
save_system(harnessName, harnessFile);

fprintf('CODEX_FOC_COMPONENTS=%d\n', 10);
fprintf('CODEX_FOC_CONTROLLER_MODEL=%s\n', controllerName);
fprintf('CODEX_FOC_HARNESS_MODEL=%s\n', harnessName);
fprintf('CODEX_FOC_ARCHITECTURE_PASS=1\n');
open_system(controllerPath);
set_param(controllerPath, 'ZoomFactor', 'FitSystem');
end

function buildComponentizedController(parent)
externalConnections = captureExternalConnections(parent);
Simulink.SubSystem.deleteContents(parent);
set_param(parent, 'BackgroundColor', 'white');

inputNames = {'SpeedReferenceRpm', 'SpeedRpm', 'Ia', 'Ib', ...
    'ThetaElectrical', 'Vdc'};
inputY = [70 125 230 285 340 520];
for index = 1:numel(inputNames)
    add_block('simulink/Sources/In1', [parent '/' inputNames{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [20 inputY(index) 50 inputY(index)+14]);
end

outputNames = {'DutyA', 'DutyB', 'DutyC', 'IqReference', ...
    'IdMeasured', 'IqMeasured', 'VdCommand', 'VqCommand'};
outputY = [235 285 335 80 385 425 465 505];
for index = 1:numel(outputNames)
    add_block('simulink/Sinks/Out1', [parent '/' outputNames{index}], ...
        'Port', num2str(index), 'OutDataTypeStr', 'single', ...
        'Position', [1910 outputY(index) 1940 outputY(index)+14]);
end

addComponent(parent, 'Speed_PI_Controller', [165 45 340 135], ...
    'yellow', '1 ms speed loop; speed error to limited Iq reference', ...
    'Ts=FOC_Native_SpeedPeriod | Kp/Ki speed | Iq limit');
buildSpeedPi([parent '/Speed_PI_Controller']);

addComponent(parent, 'Clarke_Transform', [165 205 340 300], ...
    'lightBlue', 'Two-current Clarke transform: phase currents to alpha/beta', ...
    'Ialpha=Ia | Ibeta=(Ia+2Ib)/sqrt(3)');
buildClarke([parent '/Clarke_Transform']);

addComponent(parent, 'Park_Transform', [420 205 595 300], ...
    'lightBlue', 'Park transform: stationary alpha/beta currents to rotating d/q', ...
    'Id/Iq from Ialpha/Ibeta and electrical angle');
buildPark([parent '/Park_Transform']);

add_block('simulink/Sources/Constant', [parent '/Id_Reference_Zero'], ...
    'Value', 'single(0.0)', 'OutDataTypeStr', 'single', ...
    'Position', [630 135 690 165]);

addComponent(parent, 'D_Axis_Current_PI', [690 175 865 265], ...
    'yellow', '100 us d-axis current PI; regulates Id to zero', ...
    'Kp/Ki current | integrator limit | Id*=0 A');
buildCurrentPi([parent '/D_Axis_Current_PI'], 'D');

addComponent(parent, 'Q_Axis_Current_PI', [690 290 865 380], ...
    'yellow', '100 us q-axis current PI; tracks speed-loop Iq reference', ...
    'Kp/Ki current | integrator limit | Iq*=speed PI');
buildCurrentPi([parent '/Q_Axis_Current_PI'], 'Q');

addComponent(parent, 'DQ_Decoupling_Feedforward', [690 430 865 525], ...
    'orange', 'Cross-coupling and back-EMF feedforward compensation', ...
    'Pole pairs | Ld/Lq | PM flux | mechanical speed');
buildDecoupling([parent '/DQ_Decoupling_Feedforward']);

addComponent(parent, 'DQ_Voltage_Command', [950 250 1125 355], ...
    'white', 'Combine PI and feedforward terms; limit d/q voltage commands', ...
    'Vd/Vq limit = FOC_Native_VoltageLimit');
buildVoltageCommand([parent '/DQ_Voltage_Command']);

addComponent(parent, 'Inverse_Park_Transform', [1210 250 1385 355], ...
    'lightBlue', 'Inverse Park transform: rotating d/q voltage to alpha/beta', ...
    'Valpha/Vbeta from Vd/Vq and electrical angle');
buildInversePark([parent '/Inverse_Park_Transform']);

addComponent(parent, 'Inverse_Clarke_Transform', [1470 250 1645 355], ...
    'lightBlue', 'Inverse Clarke transform: alpha/beta voltage to phase A/B/C', ...
    'Va=Valpha | Vb/Vc=-0.5Valpha +/- sqrt(3)/2 Vbeta');
buildInverseClarke([parent '/Inverse_Clarke_Transform']);

addComponent(parent, 'SVPWM_Duty_Calculation', [1715 230 1885 365], ...
    'cyan', 'Common-mode injected SVPWM and three-phase duty limiting', ...
    'Vdc scaling | duty range FOC_Native_DutyMin/Max');
buildSvpwm([parent '/SVPWM_Duty_Calculation']);

addNote(parent, ['FOC CONTROL PIPELINE - explicit component boundaries\n' ...
    'Current loop and transforms: 100 us | Speed loop: 1 ms | Numeric type: single'], ...
    [570 15 1450 70], 13, 'blue', 'lightBlue');
addNote(parent, ['Signal flow: currents -> Clarke -> Park -> PI + decoupling -> ' ...
    'inverse Park -> inverse Clarke -> SVPWM'], ...
    [580 555 1510 600], 11, 'black', 'white');

add_line(parent, 'SpeedReferenceRpm/1', 'Speed_PI_Controller/1', 'autorouting', 'on');
add_line(parent, 'SpeedRpm/1', 'Speed_PI_Controller/2', 'autorouting', 'on');
add_line(parent, 'Speed_PI_Controller/1', 'Q_Axis_Current_PI/1', 'autorouting', 'on');
add_line(parent, 'Speed_PI_Controller/1', 'IqReference/1', 'autorouting', 'on');

add_line(parent, 'Ia/1', 'Clarke_Transform/1', 'autorouting', 'on');
add_line(parent, 'Ib/1', 'Clarke_Transform/2', 'autorouting', 'on');
add_line(parent, 'Clarke_Transform/1', 'Park_Transform/1', 'autorouting', 'on');
add_line(parent, 'Clarke_Transform/2', 'Park_Transform/2', 'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'Park_Transform/3', 'autorouting', 'on');

add_line(parent, 'Id_Reference_Zero/1', 'D_Axis_Current_PI/1', 'autorouting', 'on');
add_line(parent, 'Park_Transform/1', 'D_Axis_Current_PI/2', 'autorouting', 'on');
add_line(parent, 'Park_Transform/2', 'Q_Axis_Current_PI/2', 'autorouting', 'on');
add_line(parent, 'Park_Transform/1', 'IdMeasured/1', 'autorouting', 'on');
add_line(parent, 'Park_Transform/2', 'IqMeasured/1', 'autorouting', 'on');

add_line(parent, 'SpeedRpm/1', 'DQ_Decoupling_Feedforward/1', 'autorouting', 'on');
add_line(parent, 'Park_Transform/1', 'DQ_Decoupling_Feedforward/2', 'autorouting', 'on');
add_line(parent, 'Park_Transform/2', 'DQ_Decoupling_Feedforward/3', 'autorouting', 'on');

add_line(parent, 'D_Axis_Current_PI/1', 'DQ_Voltage_Command/1', 'autorouting', 'on');
add_line(parent, 'Q_Axis_Current_PI/1', 'DQ_Voltage_Command/2', 'autorouting', 'on');
add_line(parent, 'DQ_Decoupling_Feedforward/1', 'DQ_Voltage_Command/3', 'autorouting', 'on');
add_line(parent, 'DQ_Decoupling_Feedforward/2', 'DQ_Voltage_Command/4', 'autorouting', 'on');
add_line(parent, 'DQ_Voltage_Command/1', 'VdCommand/1', 'autorouting', 'on');
add_line(parent, 'DQ_Voltage_Command/2', 'VqCommand/1', 'autorouting', 'on');

add_line(parent, 'DQ_Voltage_Command/1', 'Inverse_Park_Transform/1', 'autorouting', 'on');
add_line(parent, 'DQ_Voltage_Command/2', 'Inverse_Park_Transform/2', 'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'Inverse_Park_Transform/3', 'autorouting', 'on');
add_line(parent, 'Inverse_Park_Transform/1', 'Inverse_Clarke_Transform/1', 'autorouting', 'on');
add_line(parent, 'Inverse_Park_Transform/2', 'Inverse_Clarke_Transform/2', 'autorouting', 'on');
add_line(parent, 'Inverse_Clarke_Transform/1', 'SVPWM_Duty_Calculation/1', 'autorouting', 'on');
add_line(parent, 'Inverse_Clarke_Transform/2', 'SVPWM_Duty_Calculation/2', 'autorouting', 'on');
add_line(parent, 'Inverse_Clarke_Transform/3', 'SVPWM_Duty_Calculation/3', 'autorouting', 'on');
add_line(parent, 'Vdc/1', 'SVPWM_Duty_Calculation/4', 'autorouting', 'on');
add_line(parent, 'SVPWM_Duty_Calculation/1', 'DutyA/1', 'autorouting', 'on');
add_line(parent, 'SVPWM_Duty_Calculation/2', 'DutyB/1', 'autorouting', 'on');
add_line(parent, 'SVPWM_Duty_Calculation/3', 'DutyC/1', 'autorouting', 'on');
restoreExternalConnections(parent, externalConnections);
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

function buildPark(parent)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Ialpha', 1, [20 120 50 134]);
addIn(parent, 'Ibeta', 2, [20 200 50 214]);
addIn(parent, 'ThetaElectrical', 3, [20 300 50 314]);
addOut(parent, 'Id', 1, [520 150 550 164]);
addOut(parent, 'Iq', 2, [520 250 550 264]);
addTrig(parent, 'SinTheta', 'sin', [100 270 150 300]);
addTrig(parent, 'CosTheta', 'cos', [100 320 150 350]);
addProduct(parent, 'Id_CosAlpha', '**', [210 120 250 155]);
addProduct(parent, 'Id_SinBeta', '**', [210 175 250 210]);
addSum(parent, 'Id_Sum', '++', [320 135 350 200]);
addProduct(parent, 'Iq_SinAlpha', '**', [210 230 250 265]);
addGain(parent, 'Negative', 'single(-1.0)', [290 230 350 260]);
addProduct(parent, 'Iq_CosBeta', '**', [210 285 250 320]);
addSum(parent, 'Iq_Sum', '++', [400 245 430 310]);
add_line(parent, 'ThetaElectrical/1', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'CosTheta/1', 'autorouting', 'on');
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
addSum(parent, 'Iq_Reference_Sum', '++', [520 120 550 180]);
addSaturation(parent, 'Iq_Reference_Limit', '-FOC_Native_IqLimit', ...
    'FOC_Native_IqLimit', [590 130 660 175]);
add_block('simulink/Discrete/Zero-Order Hold', [parent '/IqRef_100us'], ...
    'SampleTime', '0.0001', 'Position', [670 130 735 160]);
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
add_line(parent, 'Integrator_Limit/1', 'Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Kp/1', 'Iq_Reference_Sum/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'Iq_Reference_Sum/2', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Sum/1', 'Iq_Reference_Limit/1', 'autorouting', 'on');
add_line(parent, 'Iq_Reference_Limit/1', 'IqRef_100us/1', 'autorouting', 'on');
add_line(parent, 'IqRef_100us/1', 'IqReference/1', 'autorouting', 'on');
addNote(parent, ['RESPONSIBILITY: outer speed PI loop\n' ...
    'Ts=FOC_Native_SpeedPeriod (1 ms)\n' ...
    'Kp=FOC_Native_KpSpeed; Ki=FOC_Native_KiSpeed\n' ...
    'Output limit: +/-FOC_Native_IqLimit; resampled to 100 us'], ...
    [80 15 690 95], 11, 'green', 'yellow');
end

function buildCurrentPi(parent, axisLabel)
Simulink.SubSystem.deleteContents(parent);
addIn(parent, 'Reference', 1, [20 140 50 154]);
addIn(parent, 'Measured', 2, [20 240 50 254]);
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
addSum(parent, 'PI_Sum', '++', [510 145 540 215]);
add_line(parent, 'Reference/1', 'Current_Error/1', 'autorouting', 'on');
add_line(parent, 'Measured/1', 'Current_Error/2', 'autorouting', 'on');
add_line(parent, 'Current_Error/1', 'Kp/1', 'autorouting', 'on');
add_line(parent, 'Current_Error/1', 'KiTs/1', 'autorouting', 'on');
add_line(parent, 'KiTs/1', 'Integrator_Add/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'Integrator_Add/2', 'autorouting', 'on');
add_line(parent, 'Integrator_Add/1', 'Integrator_Limit/1', 'autorouting', 'on');
add_line(parent, 'Integrator_Limit/1', 'Integrator_State/1', 'autorouting', 'on');
add_line(parent, 'Kp/1', 'PI_Sum/1', 'autorouting', 'on');
add_line(parent, 'Integrator_State/1', 'PI_Sum/2', 'autorouting', 'on');
add_line(parent, 'PI_Sum/1', 'VoltagePI/1', 'autorouting', 'on');
addNote(parent, sprintf(['RESPONSIBILITY: %s-axis current PI loop\n' ...
    'Ts=FOC_Native_CurrentPeriod (100 us)\n' ...
    'Kp=FOC_Native_KpCurrent; Ki=FOC_Native_KiCurrent\n' ...
    'Integrator limit: +/-FOC_Native_CurrentIntegratorLimit'], axisLabel), ...
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
addIn(parent, 'ThetaElectrical', 3, [20 300 50 314]);
addOut(parent, 'Valpha', 1, [520 150 550 164]);
addOut(parent, 'Vbeta', 2, [520 260 550 274]);
addTrig(parent, 'SinTheta', 'sin', [100 270 150 300]);
addTrig(parent, 'CosTheta', 'cos', [100 320 150 350]);
addProduct(parent, 'Valpha_CosVd', '**', [220 120 260 155]);
addProduct(parent, 'Valpha_SinVq', '**', [220 180 260 215]);
addSum(parent, 'Valpha_Sum', '+-', [340 135 370 205]);
addProduct(parent, 'Vbeta_SinVd', '**', [220 240 260 275]);
addProduct(parent, 'Vbeta_CosVq', '**', [220 300 260 335]);
addSum(parent, 'Vbeta_Sum', '++', [340 255 370 325]);
add_line(parent, 'ThetaElectrical/1', 'SinTheta/1', 'autorouting', 'on');
add_line(parent, 'ThetaElectrical/1', 'CosTheta/1', 'autorouting', 'on');
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
