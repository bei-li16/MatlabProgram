function build_pmsm_foc_dualplant_v21
%BUILD_PMSM_FOC_DUALPLANT_V21 Create a selectable native/MathWorks PMSM model.
% The v2.0.0 native model remains unchanged. This builder copies it into a
% new version package, adds the official Motor Control Blockset PMSM HDL
% plant as a second Variant Subsystem choice, simulates both choices, and
% regenerates the standalone ERT controller code.

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

closeIfLoaded(controllerName);
closeIfLoaded(harnessName);
if exist(controllerFile, 'file') == 0
    copyfile(sourceControllerFile, controllerFile);
end
if exist(harnessFile, 'file') == 0
    copyfile(sourceHarnessFile, harnessFile);
end

load_system('simulink');
load_system('mcbhdlplantlib');
load_system('simulink_hmi_blocks');
load_system(controllerFile);
load_system(harnessFile);

configureDualPlantHarness(harnessName, harnessFile, versionDirectory);
refactor_pmsm_foc_controller_v21;

nativeOutput = simulatePlant(harnessName, 1);
mcbOutput = simulatePlant(harnessName, 2);
nativeMetrics = collectMetrics(nativeOutput);
mcbMetrics = collectMetrics(mcbOutput);

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

createComparisonPlot(nativeOutput, mcbOutput, versionDirectory);
exportArchitectureImages(controllerName, harnessName, versionDirectory);
if ~(nativePass && mcbPass)
    error('At least one PMSM plant choice failed the closed-loop limits.');
end

setPlantSelection(harnessName, 2);
save_system(harnessName, harnessFile);

slbuild(controllerName);
codeReport = inspectGeneratedControllerCode(controllerName, versionDirectory);
variantReport = inspectVariantStructure(harnessName);
architectureReport = inspectControllerArchitecture(controllerName, harnessName);
writeVerificationReport(versionDirectory, controllerName, harnessName, ...
    nativeMetrics, mcbMetrics, nativePass, mcbPass, variantReport, ...
    architectureReport, codeReport);

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
fprintf('CODEX_DUAL_DEFAULT_SELECTION=2\n');

open_system([harnessName '/Selectable_PMSM_Plant']);
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

    % Refresh the compiled interface after adding the second choice.
    set_param(modelName, 'SimulationCommand', 'update');
end

% convertToVariantSubsystem creates label-mode variants. Enforce expression
% mode on every run so PMSM_PLANT_SELECTION genuinely selects the plant.
set_param([variantPath '/Native_Discrete_PMSM'], 'VariantControl', ...
    'PMSM_PLANT_SELECTION == 1');
set_param([variantPath '/MathWorks_MCB_PMSM_HDL'], 'VariantControl', ...
    'PMSM_PLANT_SELECTION == 2');
set_param(variantPath, 'VariantControlMode', 'expression');

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
setPlantSelection(modelName, 2);
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
    'Type', 'annotation');
for annotationIndex = 1:numel(annotationHandles)
    candidate = get_param(annotationHandles(annotationIndex), 'Object');
    if contains(candidate.Text, marker)
        delete(candidate);
    end
end
end

function simulationOutput = simulatePlant(modelName, selection)
setPlantSelection(modelName, selection);
set_param(modelName, 'SimulationCommand', 'update');
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
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

function exportArchitectureImages(controllerName, modelName, versionDirectory)
overviewPath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_architecture.png');
plantPath = [modelName '/Selectable_PMSM_Plant'];
plantImagePath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_plant_variants.png');
controllerImagePath = fullfile(versionDirectory, ...
    'PMSM_FOC_DualPlant_v21_controller_architecture.png');

open_system(modelName);
print(['-s' modelName], '-dpng', '-r160', overviewPath);
open_system(plantPath);
print(['-s' plantPath], '-dpng', '-r160', plantImagePath);
open_system(controllerName);
set_param(controllerName, 'ZoomFactor', 'FitSystem');
print(['-s' controllerName], '-dpng', '-r180', controllerImagePath);
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
report.HasStep = contains(sourceText, [controllerName '_step(void)']);
report.HasInitialize = contains(sourceText, [controllerName '_initialize(void)']);
report.HasInputs = contains(headerText, 'External inputs');
report.HasOutputs = contains(headerText, 'External outputs');
report.HasSFunction = contains(lower(sourceText), 's-function') || ...
    contains(lower(headerText), 's-function');
report.Pass = report.HasStep && report.HasInitialize && report.HasInputs && ...
    report.HasOutputs && ~report.HasSFunction;
end

function report = inspectControllerArchitecture(controllerName, harnessName)
componentPaths = {'Speed_PI_Controller_1ms', ...
    'Clarke_Transform', 'Park_Transform', ...
    'D_Axis_Current_PI', 'Q_Axis_Current_PI', ...
    'DQ_Decoupling_Feedforward', 'DQ_Voltage_Command', ...
    'Inverse_Park_Transform', 'Inverse_Clarke_Transform', ...
    'SVPWM_Duty_Calculation'};
report.ComponentCount = numel(componentPaths);
report.ControllerComponentsPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([controllerName '/' path]) ~= -1, componentPaths));
report.HarnessComponentsPresent = all(cellfun(@(path) ...
    getSimulinkBlockHandle([harnessName '/' path]) ~= -1, componentPaths));
report.ControllerRateTransitionPresent = getSimulinkBlockHandle( ...
    [controllerName '/IqRef_Rate_Transition']) ~= -1;
report.HarnessRateTransitionPresent = getSimulinkBlockHandle( ...
    [harnessName '/IqRef_Rate_Transition']) ~= -1;
report.SpeedTaskSampleTime = str2double(get_param( ...
    [controllerName '/Speed_PI_Controller_1ms/Integrator_State'], 'SampleTime'));
report.CurrentTaskSampleTime = str2double(get_param( ...
    [controllerName '/D_Axis_Current_PI/Integrator_State'], ...
    'SampleTime'));
report.ControllerDanglingLines = countDanglingLines(controllerName);
report.HarnessDanglingLines = countDanglingLines(harnessName);
report.Pass = report.ControllerComponentsPresent && ...
    report.HarnessComponentsPresent && ...
    report.ControllerRateTransitionPresent && ...
    report.HarnessRateTransitionPresent && ...
    abs(report.SpeedTaskSampleTime - 0.001) < eps && ...
    abs(report.CurrentTaskSampleTime - 0.0001) < eps && ...
    report.ControllerDanglingLines == 0 && ...
    report.HarnessDanglingLines == 0;
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
        architectureReport, codeReport)
reportPath = fullfile(versionDirectory, 'verification_report.txt');
reportHandle = fopen(reportPath, 'w');
assert(reportHandle ~= -1, 'Unable to create verification report.');
reportCleanup = onCleanup(@() fclose(reportHandle));
fprintf(reportHandle, 'PMSM FOC Dual Plant v2.1.0 verification\n');
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
fprintf(reportHandle, 'Controller components present: %d\n', ...
    architectureReport.ControllerComponentsPresent);
fprintf(reportHandle, 'Harness components present: %d\n', ...
    architectureReport.HarnessComponentsPresent);
fprintf(reportHandle, 'Controller rate transition present: %d\n', ...
    architectureReport.ControllerRateTransitionPresent);
fprintf(reportHandle, 'Harness rate transition present: %d\n', ...
    architectureReport.HarnessRateTransitionPresent);
fprintf(reportHandle, 'Speed task sample time s: %.9g\n', ...
    architectureReport.SpeedTaskSampleTime);
fprintf(reportHandle, 'Current task sample time s: %.9g\n', ...
    architectureReport.CurrentTaskSampleTime);
fprintf(reportHandle, 'Controller dangling line count: %d\n', ...
    architectureReport.ControllerDanglingLines);
fprintf(reportHandle, 'Harness dangling line count: %d\n', ...
    architectureReport.HarnessDanglingLines);
fprintf(reportHandle, 'Component architecture verification pass: %d\n', ...
    architectureReport.Pass);
writeMetrics(reportHandle, 'Native', nativeMetrics, nativePass);
writeMetrics(reportHandle, 'MathWorks MCB PMSM HDL', mcbMetrics, mcbPass);
fprintf(reportHandle, 'ERT step present: %d\n', codeReport.HasStep);
fprintf(reportHandle, 'ERT initialize present: %d\n', codeReport.HasInitialize);
fprintf(reportHandle, 'ERT inputs present: %d\n', codeReport.HasInputs);
fprintf(reportHandle, 'ERT outputs present: %d\n', codeReport.HasOutputs);
fprintf(reportHandle, 'Generated S-Function text present: %d\n', codeReport.HasSFunction);
fprintf(reportHandle, 'Code verification pass: %d\n', codeReport.Pass);
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
