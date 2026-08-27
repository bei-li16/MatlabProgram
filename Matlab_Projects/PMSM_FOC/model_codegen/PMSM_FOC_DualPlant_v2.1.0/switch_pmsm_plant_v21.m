% SWITCH_PMSM_PLANT_V21 Toggle the active plant in the dual-plant harness.
% This script is invoked by the dashboard one-click switch button.

modelName = 'PMSM_FOC_DualPlant_ClosedLoop_v21';
if isempty(modelName) || ~bdIsLoaded(modelName)
    error('The dual-plant model must be open before switching the plant.');
end
if ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
    error('Stop the simulation before switching the active PMSM plant.');
end

modelWorkspace = get_param(modelName, 'ModelWorkspace');
currentSelection = getVariable(modelWorkspace, 'PMSM_PLANT_SELECTION');
if double(currentSelection) == 1
    nextSelection = 2;
    nextName = 'MathWorks Motor Control Blockset PMSM HDL';
else
    nextSelection = 1;
    nextName = 'Native discrete PMSM';
end

assignin(modelWorkspace, 'PMSM_PLANT_SELECTION', nextSelection);

buttonPath = [modelName '/Switch_PMSM_Plant'];
if getSimulinkBlockHandle(buttonPath) ~= -1
    configuration = jsondecode(get_param(buttonPath, 'Configuration'));
    for componentIndex = 1:numel(configuration.components)
        if strcmp(configuration.components(componentIndex).name, 'ButtonStateComponent')
            states = configuration.components(componentIndex).settings.states;
            for stateIndex = 1:numel(states)
                states(stateIndex).label.text.content = ['Active: ' nextName];
            end
            configuration.components(componentIndex).settings.states = states;
        end
    end
    set_param(buttonPath, 'Configuration', jsonencode(configuration));
end

save_system(modelName);
fprintf('PMSM_PLANT_SELECTION=%d (%s)\n', nextSelection, nextName);

clear modelName modelWorkspace currentSelection nextSelection nextName
clear buttonPath configuration componentIndex states stateIndex
