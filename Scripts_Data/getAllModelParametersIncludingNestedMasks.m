function getAllModelParametersIncludingNestedMasks(modelName)
    % Load the model if it's not already open
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    
    fprintf('Model Parameters and Initialized Numeric Values:\n');

    % Get parameters from the base workspace and from blocks
    getModelParameters(modelName);
    
    % Get all blocks in the model
    % blocks = find_system(modelName, 'Type', 'Block');
    blocks = find_system(modelName,'MatchFilter',@Simulink.match.activeVariants,'LookUnderMasks','All','FindAll','on','Mask','on','Type','Block');

    for i = 1:length(blocks)
        disp(regexprep(getfullname(blocks(i)), '\r?\n', ' '));
    end

    map = struct;
    
    % Recursively check masked blocks for parameters
    for i = 1:length(blocks)
        
        % block = blocks{i};
        block = blocks(i);
        
        if strcmp(get_param(block, 'Mask'), 'on')
            map = getMaskedBlockParameters(block, map);
        end
    end
end

function getModelParameters(modelName)
    % Find all variables that the model depends on
    vars = Simulink.findVars(modelName);
    
    % Loop through each variable found in the model
    for i = 1:length(vars)
        varName = vars(i).Name;
        % Check if the variable exists in the base workspace
        if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
            varValue = evalin('base', varName);
            % Check if the variable value is numeric
            if isnumeric(varValue)
                fprintf('Parameter: %s, Value: %s\n', varName, mat2str(varValue));
            else
                test = 0;
            end
        else
            fprintf('Parameter: %s is undefined in the base workspace.\n', varName);
        end
    end
end

function map = getMaskedBlockParameters(block, map)

    blockPath = regexprep(getfullname(block), '\r?\n', ' ');
    blockPath = regexprep(blockPath, ' ', '_');
    blockPath = regexprep(blockPath, '-', '_');
    blockPathSplit = strsplit(blockPath, '/');
    % % modelName = blockPathSplit(1);
    blockPath = strjoin([blockPathSplit(2:end)],'/');
    if strcmp(blockPath, 'Vehicle/Body/Body_Inertia')
        test = 0;
    end

    % Get the mask object for the block
    maskObj = Simulink.Mask.get(block);
    maskParams = maskObj.Parameters;
    
    % Loop through each mask parameter
    for j = 1:length(maskParams)
        paramName = maskParams(j).Name;
        isEdit = strcmp(maskParams(j).Type,'edit');
        isHidden = strcmp(maskParams(j).Hidden, 'on');
        isVisible = strcmp(maskParams(j).Visible, 'on');
        isEvaluate = strcmp(maskParams(j).Evaluate, 'on');
        paramValue = get_param(block, paramName);

        % try
        %     test = all(isnumeric(str2num(paramValue))) && ~any(isnan(str2num(paramValue)),'all');
        % catch ME
        %     test = 0;
        % end
        
        % Check if the parameter value is numeric
        if isEdit && ~isHidden && isVisible && isEvaluate && ~isempty(paramValue)
            % if ~ischar(paramValue) && all(isnumeric(str2num(paramValue))) && ~any(isnan(str2num(paramValue)),'all')
            %     fprintf('Masked Block: %s, Parameter: %s, Value: %s\n', block, paramName, paramValue);
            if ischar(paramValue) && ~strcmp(paramValue, '[]')
                if ~isempty(str2num(paramValue))
                    spl = regexp(strjoin({blockPath, paramName},'/'),'\/','split');
                    map = setfield(map, spl{:}, str2num(paramValue));
                elseif contains(paramValue,'.')
                    splVal = regexp(paramValue,'\.','split');
                    splField = regexp(strjoin({blockPath, paramName},'/'),'\/','split');
                    try
                        map = setfield(map, splField{:}, getfield(map, splVal{:}));
                    catch ME
                        test = 0;
                    end
                else
                    % spl = regexp(strjoin({blockPath, paramName},'/'),'\/','split');
                    try
                        map.(paramName) = evalin('base', paramValue);
                    catch ME
                        % rethrow(ME)
                    end
                end

                if ~isempty(fieldnames(map))
                    test = 0;
                end
                
                % if ~isstruct(para)
                %     fprintf('Masked Block: %s, Parameter: %s, Value: %s\n', block, paramName, paramValue);
                % end
            else
                fprintf('Masked Block: %s, Parameter: %s, Value: %s\n', block, paramName, paramValue);
            end
        end
    end

    % Recursively check for nested masked blocks
    % nestedBlocks = find_system(block, 'LookUnderMasks','All', 'SearchDepth', 1, 'Type', 'Block', 'Mask', 'on');
    % nestedBlocks(strcmp(nestedBlocks, block)) = [];
    % for k = 1:length(nestedBlocks)
    %     map = getMaskedBlockParameters(nestedBlocks{k}, map);
    % end
end

function result = getStringUntilDot(inputString)
    % Find the position of the first dot
    dotPosition = strfind(inputString, '.');
    
    % Check if a dot was found
    if isempty(dotPosition)
        % If no dot is found, return the entire string
        result = inputString;
    else
        % Extract the substring up to the first dot
        result = inputString(1:dotPosition(1)-1);
    end
end

function result = removeSubstringAtStart(originalString, substringToRemove)
    % Check if the original string starts with the specified substring
    if strncmp(originalString, substringToRemove, length(substringToRemove))
        % Remove the substring from the start
        result = originalString(length(substringToRemove) + 1:end);
    else
        % Return the original string if the substring is not at the start
        result = originalString;
    end
end