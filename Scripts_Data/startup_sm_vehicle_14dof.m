%% Copyright 2021-2023 The MathWorks, Inc.
clear;

% Add MF-Swift software to path if it can be found
if(exist('sm_vehicle_startupMFSwift.m','file'))
    [~,MFSwifttbx_folders]=sm_vehicle_startupMFSwift;
    assignin('base','MFSwifttbx_folders',MFSwifttbx_folders);
end

% Parameters
sm_vehicle_2axle_heave_roll_abs_param

% Open Overiew
web('sm_vehicle_2axle_heave_roll_Overview.html');

% Open model
sm_vehicle_vtk

% Set model parameter
sm_vehicle_2axle_heave_roll_param
sm_vehicle_2axle_heave_roll_scene
sm_vehicle_2axle_heave_roll_config_maneuver('sm_vehicle_vtk',VehicleData,'external');


[path,~,~] = fileparts(mfilename('fullpath'));
mat_path = fullfile(path, 'sm_vehicle_vtk_para.mat');
load(mat_path);
% create cleaned struct for parameter output
VehicleDataSimu = getSimulinkStruct(VehicleData);
% createBusObject(VehicleDataSimu, 'VehicleData_t');
busInfo = Simulink.Bus.createObject(VehicleDataSimu);
assignin('base', 'VehicleData_t', eval(busInfo.busName));
% VehicleData_t = cleanStructMemberNames(VehicleData_t);
para = Simulink.Parameter;
para.DataType = 'Bus: VehicleData_t';
para.Value = VehicleDataSimu;
assignin('base', 'VehicleData_P', para);
save(mat_path,'VehicleData_t','VehicleData_P','Body', ...
    'SuspF','TireDataF','RimF','SuspR','TireDataR','RimR',...
    'Heave','Roll','UnsprungMass','Hub');

% Create IO Ports for root model
sm_vehicle_create_io_structs
% create cleaned struct for parameter output
% blk = getSimulinkBlockHandle('sm_vehicle_vtk/Driver Input');
% activeChoice = get_param(blk, 'CompiledActiveChoiceBlock');
% slBus1_ = Simulink.Bus.createObject('sm_vehicle_vtk',...
%     [activeChoice, '/Bus Creator']);
% sm_vehicle_U_t = evalin('base', slBus1_.busName);
% 
% slBus2_ = Simulink.Bus.createObject('sm_vehicle_vtk',...
%     'sm_vehicle_vtk/meas/m');
% sm_vehicle_Y_t = evalin('base', slBus2_.busName);

% function createBusObject(structure, bws_name)
%     busInfo = Simulink.Bus.createObject(structure);
%     assignin('base', bws_name, eval(busInfo.busName));
% end


% function for cleaning up the parameter from non-numeric data
function simuStruct = getSimulinkStruct(initStruct)
    numericStruct = cleanStructFromNonNumeric(initStruct);
    simuStruct = cleanStructFromEmptyMembers(numericStruct);
end

function initStruct = cleanStructMemberNames(initStruct)

    if isa(initStruct, 'Simulink.BusElement')
        if length(initStruct) > 1  % Elements array
            fields = {};
            n_fields = length(initStruct);
        else
            fields = fieldnames(initStruct);
            n_fields = length(fields);
        end
    else
        fields = fieldnames(initStruct);
        n_fields = length(fields);
    end
    
    for n = 1 : n_fields
        if ~isempty(fields) && strcmp(fields{n},'DataType') % && isstruct(initStruct) % && isstring(initStruct.(fields{n}))
            if contains(initStruct.(fields{n}),'SuspF')
                test = 0;
            end
        end
        if isa(initStruct, 'Simulink.BusElement') && isempty(fields)
            val = initStruct(n);
            initStruct(n)= cleanStructMemberNames(val);
        elseif strcmp(fields{n},'Elements')
            val = initStruct.(fields{n});
            initStruct.(fields{n}) = cleanStructMemberNames(val);
        elseif strcmp(fields{n},'DataType')
            val = initStruct.(fields{n});
            if startsWith(val,'slBus')
                val_split= strsplit(val,'_');
                val=erase(val,[val_split(1),'_']);
            end
            initStruct.(fields{n}) = val;
        else
            initStruct.(fields{n}) = initStruct.(fields{n});
        end
    end
end


function initStruct = cleanStructFromNonNumeric(initStruct)
    fields = fieldnames(initStruct);
    n_fields = length(fields);

    for n = 1 : n_fields
        val = initStruct.(fields{n});
        if isstruct(val) && ~isempty(fieldnames(val))
            if strcmp(fields{n},'MODEL')
                temp = initStruct.(fields{n});
                initStruct = rmfield(initStruct, fields{n});
                fields{n} = [fields{n},'_'];
                initStruct.(fields{n}) = temp;
            end
            initStruct.(fields{n}) = cleanStructFromNonNumeric(val);
        elseif ~isempty(val) &&  isnumeric(val)
            initStruct.(fields{n}) = val;
        else
            initStruct = rmfield(initStruct, fields{n});
        end
    end
end

function initStruct = cleanStructFromEmptyMembers(initStruct)
    fields = fieldnames(initStruct);
    n_fields = length(fields);

    for n = 1 : n_fields
        val = initStruct.(fields{n});
        if isstruct(val) && ~isempty(fieldnames(val))
            initStruct.(fields{n}) = cleanStructFromEmptyMembers(val);
        elseif isstruct(val) && isempty(fieldnames(val))
            initStruct = rmfield(initStruct, fields{n});
        else
            initStruct.(fields{n}) = val;
        end
    end
end 

