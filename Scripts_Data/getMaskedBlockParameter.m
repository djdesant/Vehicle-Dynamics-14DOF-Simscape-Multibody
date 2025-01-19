function paramValue = getMaskedBlockParameter(modelName, blockPath, paramName)
    % Load the model if it's not already open
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    % Initialize the model
    set_param(modelName, 'SimulationCommand', 'start');

    % Ensure the model is running or has been initialized
    pause(1);  % Pause for a brief moment to allow for initialization; adjust as needed

    % Access the masked block parameter value
    try
        % Get the parameter value
        rto = get_param(blockPath,'RuntimeObject');
        paramValue = get_param(blockPath, paramName);
        
        % Convert to numeric if it's a string representation of a number
        paramValueNumeric = str2double(paramValue);
        
        if isnan(paramValueNumeric)
            fprintf('The parameter "%s" in block "%s" is not numeric.\n', paramName, blockPath);
            paramValue = paramValue;  % Return the original string value if not numeric
        else
            fprintf('The numeric value of parameter "%s" in block "%s" is: %f\n', paramName, blockPath, paramValueNumeric);
            paramValue = paramValueNumeric;  % Return the numeric value
        end
    catch ME
        fprintf('Error retrieving parameter "%s" from block "%s": %s\n', paramName, blockPath, ME.message);
        paramValue = [];
    end
    
    % Optionally stop the simulation if it was started
    set_param(modelName, 'SimulationCommand', 'stop');
end