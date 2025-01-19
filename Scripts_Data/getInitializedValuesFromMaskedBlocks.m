function getInitializedValuesFromMaskedBlocks(modelName, para)
    % Load the model if it's not already open
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    
    % Get all blocks in the model
    % allBlocks = find_system(modelName, 'Type', 'Block');
    allBlocks = find_system(modelName,'LookUnderMasks','All','FindAll','on','Mask','on','Type','Block');

    built_in_fcns = {'get_param','set_param','strrep'};
    
    % Loop through each block to check for mask initialization
    for i = 1:length(allBlocks)
        block = allBlocks(i);
        
        % Check if the block has a mask
        if strcmp(get_param(block, 'Mask'), 'on')
            % Get the mask initialization code
            initCode = get_param(block, 'MaskInitialization');

            % Get the mask object
            % maskObj = Simulink.Mask.get(block);
            
            % if ~isempty(maskObj)
            %     fprintf('Block: %s\n', block);
            % 
            %     % Get all mask parameters for this block
            %     maskParams = maskObj.Parameters;
            % 
            %     % Display the name and text of each mask parameter
            %     for j = 1:length(maskParams)
            %         paramName = maskParams(j).Name;
            %         paramValue = maskParams(j).Value;
            %         fprintf('Parameter Name: %s, Text Content: %s\n', paramName, paramValue);
            %     end
            %     fprintf('----------------------\n');
            % end

            for j=1:length(built_in_fcns)
                if ~isempty(initCode)
                    initCode = removeFunctionCalls(initCode, built_in_fcns{j});
                end
            end

            if ~isempty(initCode)
                fprintf('Block: %s\n', block);
                fprintf('Initialization Code:\n%s\n', initCode);
                
                % Extract variable names from initialization code
                variableNames = regexp(initCode, '\<\w+\>', 'match');
                uniqueVars = unique(variableNames);  % Get unique variable names
                
                % Check if variables exist in the base workspace
                missingVars = {};
                for j = 1:length(uniqueVars)
                    if ~evalin('base', sprintf('exist(''%s'', ''var'')', uniqueVars{j}))
                        missingVars{end+1} = uniqueVars{j}; %#ok<AGROW>
                    end
                end
                
                % Display missing variables if any
                if ~isempty(missingVars)
                    fprintf('Missing Variables for Initialization: %s\n', strjoin(missingVars, ', '));
                else
                    % Evaluate the initialization code if no missing variables
                    try
                        evalin('base', initCode);  % Evaluate in base workspace for dependency resolution
                        initVars = evalin('base', 'whos');  % Get list of initialized variables
                        
                        % Display initialized variables and their values
                        for k = 1:length(initVars)
                            varName = initVars(k).name;
                            varValue = evalin('base', varName);
                            fprintf('Variable: %s, Value: %s\n', varName, mat2str(varValue));
                        end
                    catch ME
                        fprintf('Error evaluating initialization code: %s\n', ME.message);
                    end
                end
                fprintf('----------------------\n');
            end
            
            if ~isempty(initCode)
                fprintf('Block: %s\n', block);
                fprintf('Initialization Code:\n%s\n', initCode);
                
                % Use a temporary structure to store variables from eval
                initValues = struct();
                
                % Evaluate the initialization code in the temporary structure
                try
                    evalin('base', initCode);
                    initValues = whos;  % Get list of initialized variables
                    
                    % Display initialized variables and their values
                    for j = 1:length(initValues)
                        varName = initValues(j).name;
                        varValue = eval(varName);
                        fprintf('Variable: %s, Value: %s\n', varName, mat2str(varValue));
                    end
                catch ME
                    fprintf('Error evaluating initialization code: %s\n', ME.message);
                end
                fprintf('----------------------\n');
            end
        end
    end
end

function filteredText = removeLinesContainingString(text, targetString)
    % Split the input character array into lines
    lines = splitlines(text);
    
    % Find lines that do not contain the target string
    linesToKeep = ~contains(lines, targetString);
    
    % Filter out lines that contain the target string
    filteredText = lines(linesToKeep);
    
    % Combine the remaining lines back into a single character array
    filteredText = strjoin(filteredText, newline);
end

function cleanedText = removeFunctionCalls(inputText, functionName)
    % Split the input text into lines
    lines = strsplit(inputText, newline);
    
    % Create a regular expression pattern to match the function call with its arguments
    % This pattern matches the function name followed by any characters (arguments),
    % up to the closing parenthesis, potentially spread across multiple lines.
    funcPattern = sprintf('%s\\s*\\(.*', functionName);
    
    % Initialize an empty cell array to store the cleaned lines
    cleanedLines = {};
    
    % Flag to indicate whether we're inside a function call
    insideFunction = false;
    
    % Loop through each line to find and remove the function calls
    for i = 1:length(lines)
        line = lines{i};
        
        if insideFunction
            % If we are inside the function call (with arguments extending to multiple lines),
            % check if this line contains the closing parenthesis
            if contains(line, ')')
                insideFunction = false;  % End the removal after closing parenthesis is found
            end
            % Skip this line because it's part of the function call
            continue;
        end
        
        % Check if this line contains the function call with its arguments
        if contains(line, [functionName, '('])
            % Check if the line has the full call (if closing parenthesis is present)
            if ~contains(line, ')')
                % If no closing parenthesis is on the same line, start removal for following lines
                insideFunction = true;
            end
            % Skip this line because it contains the function call
            continue;
        end
        
        % Add the line to the cleaned lines if it's not part of the function call
        cleanedLines{end + 1} = line; %#ok<AGROW>
    end
    
    % Combine the cleaned lines back into a single text
    cleanedText = strjoin(cleanedLines, newline);
end