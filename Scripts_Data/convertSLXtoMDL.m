
modelName = 'sm_vehicle_vtk';

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% test = get_param(0, modelName);

% paraVal = getMaskedBlockParameter(modelName, [modelName, '/Vehicle/Scene/Grid/Mesh Lines x/Solid'], 'ExtrusionCrossSection');

getAllModelParametersIncludingNestedMasks(modelName);

maskInitFcns = getMaskInitFcn(modelName);

getInitializedValuesFromMaskedBlocks(modelName, InitVehicle);

% Get model callbacks
callbacks = get_param(modelName, 'ObjectParameters');
callback_names = fieldnames(callbacks);

% Check each callback for function calls
for i = 1:length(callback_names)
    if contains(callback_names{i}, 'Fcn')  % Only look at callback properties
        callback_code = get_param(modelName, callback_names{i});
        if ~isempty(callback_code)
            fprintf('Callback %s:\n%s\n\n', callback_names{i}, callback_code);
        end
    end
end

modelPath = which([modelName, '.slx']);
[modelFolder, ~, ~] = fileparts(modelPath);

mdl_Folder = fullfile(modelFolder, modelName);
if ~exist(mdl_Folder, 'dir')
    [status, msg, msgID] = mkdir(mdl_Folder);
end

save_system(modelName,fullfile(mdl_Folder,[modelName,'.mdl']),...
    'SaveModelWorkspace', true, 'OverwriteIfChangedOnDisk',true);

libdata = libinfo(modelName);
libs_list = {libdata(:).('Library')};

remove_list = {'sm_lib','nesl_utility'};
libs_list(ismember(libs_list,remove_list)) = [];

unique_libs = unique(vertcat(libs_list));

for i=1:length(unique_libs)
    save_system(unique_libs{i},fullfile(mdl_Folder,[unique_libs{i},'.mdl']),...
    'SaveModelWorkspace', true, 'OverwriteIfChangedOnDisk',true);
end