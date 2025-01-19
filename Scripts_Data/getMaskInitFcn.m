function fcnFound = getMaskInitFcn(modelName)
%GETMASKINITFCN Summary of this function goes here
%   Detailed explanation goes here

fcnFound = {};
    
% Get all blocks in the model
% allBlocks = find_system(modelName, 'Type', 'Block');

allBlocks = find_system(modelName,'LookUnderMasks','All','FindAll','on','Mask','on','Type','Block');

% Loop through each block to check for mask initialization code
for i = 1:length(allBlocks)
    block = allBlocks(i);

    % Check if the block has a mask
    if strcmp(get_param(block, 'Mask'), 'on')
        % Get the mask initialization code
        initCode = get_param(block, 'MaskInitialization');

        % Check if there is any initialization code
        if ~isempty(initCode)
            fprintf('Block: %s\n', block);
            fprintf('Initialization Code:\n%s\n', initCode);

            % Detect function calls (simple approach: look for parentheses after names)
            functionCalls = regexp(initCode, '\<\w+\>\s*(?=\()', 'match');

            if ~isempty(functionCalls)
                fprintf('Potential MATLAB Function Calls Detected:\n');
                fcnFound = unique(vertcat([fcnFound, functionCalls]));
                disp(fcnFound);  % Display unique function calls found
            else
                fprintf('No MATLAB function calls detected in initialization code.\n');
            end
            fprintf('----------------------\n');
        end
    end
end

end

