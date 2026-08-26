function build_basic_pmsm_foc
%BUILD_BASIC_PMSM_FOC Build and validate a runnable PMSM FOC Simulink model.

modelName = 'PMSM_FOC_Basic_v2';
modelDir = fileparts(mfilename('fullpath'));
modelFile = fullfile(modelDir, [modelName '.slx']);
oldDir = pwd;
cleanupDir = onCleanup(@() cd(oldDir));
cd(modelDir);

if bdIsLoaded(modelName)
    error('Model is already loaded: %s', modelName);
end
if isfile(modelFile)
    error('Model already exists: %s', modelFile);
end

new_system(modelName);
set_param(modelName, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1e-4', ...
    'StartTime', '0.0', ...
    'StopTime', '1.2', ...
    'SignalLogging', 'off', ...
    'SaveOutput', 'off', ...
    'SaveTime', 'off', ...
    'Description', ['Runnable basic PMSM FOC model: 100 us current loop, ' ...
    '1 ms speed loop, SVPWM average inverter, discrete PMSM plant.']);

add_block('simulink/Sources/Step', [modelName '/Speed_Command_rpm'], ...
    'Time', '0.05', 'Before', '0', 'After', '1000', ...
    'SampleTime', '1e-4', 'OutDataTypeStr', 'single', ...
    'Position', [35 80 105 110]);
add_block('simulink/Sources/Constant', [modelName '/DC_Bus_Voltage'], ...
    'Value', 'single(48)', 'SampleTime', '1e-4', ...
    'OutDataTypeStr', 'single', 'Position', [35 190 105 220]);
add_block('simulink/Sources/Step', [modelName '/Load_Torque_Nm'], ...
    'Time', '0.50', 'Before', '0', 'After', '0.20', ...
    'SampleTime', '1e-4', 'OutDataTypeStr', 'single', ...
    'Position', [35 330 105 360]);

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [modelName '/FOC_Controller_100us'], ...
    'FunctionName', 'foc_controller_sfun', ...
    'Position', [240 60 455 285]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [modelName '/SVPWM_Average_Inverter'], ...
    'FunctionName', 'average_inverter_sfun', ...
    'Position', [545 85 725 205]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [modelName '/PMSM_Discrete_Plant'], ...
    'FunctionName', 'pmsm_plant_sfun', ...
    'Position', [845 65 1045 315]);

add_block('simulink/Signal Routing/Mux', [modelName '/Speed_Mux'], ...
    'Inputs', '2', 'Position', [1110 65 1115 115]);
add_block('simulink/Sinks/Scope', [modelName '/Speed_Scope'], ...
    'Position', [1160 65 1220 115]);
add_block('simulink/Signal Routing/Mux', [modelName '/Current_Mux'], ...
    'Inputs', '2', 'Position', [1110 150 1115 200]);
add_block('simulink/Sinks/Scope', [modelName '/Iq_Current_Scope'], ...
    'Position', [1160 150 1220 200]);
add_block('simulink/Signal Routing/Mux', [modelName '/Duty_Mux'], ...
    'Inputs', '3', 'Position', [755 235 760 305]);
add_block('simulink/Sinks/Scope', [modelName '/SVPWM_Duty_Scope'], ...
    'Position', [790 245 850 295]);
add_block('simulink/Sinks/Display', [modelName '/Speed_Display_rpm'], ...
    'Position', [1110 245 1200 275]);

logNames = {'speed_ref_log','speed_rpm_log','iq_ref_log','iq_meas_log', ...
    'torque_log','duty_a_log'};
logPositions = [1270 35; 1270 85; 1270 140; 1270 190; 1270 245; 900 345];
for index = 1:numel(logNames)
    add_block('simulink/Sinks/To Workspace', [modelName '/' logNames{index}], ...
        'VariableName', logNames{index}, 'SaveFormat', 'Timeseries', ...
        'MaxDataPoints', 'inf', ...
        'Position', [logPositions(index,:) logPositions(index,:) + [100 30]]);
end

% Closed-loop controller and plant connections.
add_line(modelName, 'Speed_Command_rpm/1', 'FOC_Controller_100us/1', 'autorouting', 'on');
add_line(modelName, 'DC_Bus_Voltage/1', 'FOC_Controller_100us/6', 'autorouting', 'on');
add_line(modelName, 'DC_Bus_Voltage/1', 'SVPWM_Average_Inverter/4', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/1', 'SVPWM_Average_Inverter/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/2', 'SVPWM_Average_Inverter/2', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/3', 'SVPWM_Average_Inverter/3', 'autorouting', 'on');
add_line(modelName, 'SVPWM_Average_Inverter/1', 'PMSM_Discrete_Plant/1', 'autorouting', 'on');
add_line(modelName, 'SVPWM_Average_Inverter/2', 'PMSM_Discrete_Plant/2', 'autorouting', 'on');
add_line(modelName, 'Load_Torque_Nm/1', 'PMSM_Discrete_Plant/3', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/1', 'FOC_Controller_100us/3', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/2', 'FOC_Controller_100us/4', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/4', 'FOC_Controller_100us/2', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/5', 'FOC_Controller_100us/5', 'autorouting', 'on');

% Scopes and simulation logs.
add_line(modelName, 'Speed_Command_rpm/1', 'Speed_Mux/1', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/4', 'Speed_Mux/2', 'autorouting', 'on');
add_line(modelName, 'Speed_Mux/1', 'Speed_Scope/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/4', 'Current_Mux/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/6', 'Current_Mux/2', 'autorouting', 'on');
add_line(modelName, 'Current_Mux/1', 'Iq_Current_Scope/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/1', 'Duty_Mux/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/2', 'Duty_Mux/2', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/3', 'Duty_Mux/3', 'autorouting', 'on');
add_line(modelName, 'Duty_Mux/1', 'SVPWM_Duty_Scope/1', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/4', 'Speed_Display_rpm/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Command_rpm/1', 'speed_ref_log/1', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/4', 'speed_rpm_log/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/4', 'iq_ref_log/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/6', 'iq_meas_log/1', 'autorouting', 'on');
add_line(modelName, 'PMSM_Discrete_Plant/8', 'torque_log/1', 'autorouting', 'on');
add_line(modelName, 'FOC_Controller_100us/1', 'duty_a_log/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(modelName);
set_param(modelName, 'SimulationCommand', 'update');
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');

speedReference = simulationOutput.get('speed_ref_log');
speedResult = simulationOutput.get('speed_rpm_log');
iqReference = simulationOutput.get('iq_ref_log');
iqResult = simulationOutput.get('iq_meas_log');
torqueResult = simulationOutput.get('torque_log');
dutyResult = simulationOutput.get('duty_a_log');

finalSpeed = double(speedResult.Data(end));
maximumSpeed = max(double(speedResult.Data));
maximumIq = max(abs(double(iqResult.Data)));
minimumDuty = min(double(dutyResult.Data));
maximumDuty = max(double(dutyResult.Data));
simulationPassed = all(isfinite(double(speedResult.Data))) && ...
    all(isfinite(double(iqResult.Data))) && abs(finalSpeed - 1000.0) < 50.0 && ...
    maximumIq <= 8.5 && minimumDuty >= 0.0 && maximumDuty <= 1.0;

assignin('base', 'PMSM_FOC_Basic_LastSim', simulationOutput);
save_system(modelName, modelFile);

resultFigure = figure('Visible', 'off', 'Color', 'white');
tiledlayout(resultFigure, 3, 1, 'TileSpacing', 'compact');
nexttile;
plot(speedReference.Time, speedReference.Data, '--', 'LineWidth', 1.1); hold on;
plot(speedResult.Time, speedResult.Data, 'LineWidth', 1.2); grid on;
ylabel('Speed (rpm)'); legend('Reference', 'Actual', 'Location', 'best');
title('PMSM FOC Basic Closed-Loop Validation');
nexttile;
plot(iqReference.Time, iqReference.Data, '--', 'LineWidth', 1.1); hold on;
plot(iqResult.Time, iqResult.Data, 'LineWidth', 1.2); grid on;
ylabel('q current (A)'); legend('Reference', 'Actual', 'Location', 'best');
nexttile;
plot(torqueResult.Time, torqueResult.Data, 'LineWidth', 1.2); grid on;
ylabel('Torque (N m)'); xlabel('Time (s)');
exportgraphics(resultFigure, fullfile(modelDir, [modelName '_results.png']), ...
    'Resolution', 150);
close(resultFigure);

open_system(modelName);
set_param(modelName, 'ZoomFactor', 'FitSystem');

fprintf('CODEX_FOC_MODEL=%s\n', modelFile);
fprintf('CODEX_FOC_FINAL_SPEED_RPM=%.3f\n', finalSpeed);
fprintf('CODEX_FOC_MAX_SPEED_RPM=%.3f\n', maximumSpeed);
fprintf('CODEX_FOC_MAX_ABS_IQ_A=%.3f\n', maximumIq);
fprintf('CODEX_FOC_DUTY_RANGE=[%.4f, %.4f]\n', minimumDuty, maximumDuty);
fprintf('CODEX_FOC_SIM_PASS=%d\n', simulationPassed);
end
