%% Step 1: Open and Load the Model
modelName = 'sm_vehicle_vtk';

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

%% Step 2: Identify All Bodies, Joints, and Rigid Transforms

% Find all Multibody blocks
bodyBlocks = find_system(modelName,'MatchFilter', @Simulink.match.activeVariants, ...
    'LookUnderMasks','All', 'FollowLinks','on', 'FindAll', 'on', ...
    'regexp', 'on', 'Type', 'block', 'BlockType', 'SimscapeMultibodyBlock', 'MaskType',  '.*Solid$'); %'^(?!.*Joint$).*');
disp('******* Multibody Blocks *********');
for i=1:length(bodyBlocks)
    disp([get_param(bodyBlocks(i),'BlockType'), ',', ...
        regexprep(get_param(bodyBlocks(i),'MaskType'), '\r?\n', ' '),',', ...
        regexprep(getfullname(bodyBlocks(i)), '\r?\n', ' ')]);
end

% Find all Joint blocks
jointBlocks = find_system(modelName,'MatchFilter', @Simulink.match.activeVariants, ...
    'LookUnderMasks','All', 'FollowLinks','on', 'FindAll','on', ...
    'regexp','on', 'Type', 'block', 'BlockType', 'SimscapeMultibodyBlock', 'MaskType', '.*Joint$');
disp('******* Joint Blocks *********');
for i=1:length(jointBlocks)
    disp([get_param(jointBlocks(i),'BlockType'), ',', ...
        regexprep(get_param(jointBlocks(i),'MaskType'), '\r?\n', ' '),',', ...
        regexprep(getfullname(jointBlocks(i)), '\r?\n', ' ')]);
end

% Find all Rigid Transform blocks
rigidTransformBlocks = find_system(modelName,'MatchFilter', @Simulink.match.activeVariants, ...
    'LookUnderMasks','All', 'FollowLinks','on', 'FindAll','on', ...
    'regexp','on', 'Type', 'block', 'BlockType', 'SimscapeMultibodyBlock', 'MaskType', '.*Rigid Transform$');
disp('******* Rigid Transform Blocks *********');
for i=1:length(rigidTransformBlocks)
    disp([get_param(rigidTransformBlocks(i),'BlockType'), ',', ...
        regexprep(get_param(rigidTransformBlocks(i),'MaskType'), '\r?\n', ' '),',', ...
        regexprep(getfullname(rigidTransformBlocks(i)), '\r?\n', ' ')]); %[\s\r\n]+', '');
end

%% Step 3: Construct the Connectivity Graph
% Initialize an empty graph
G = graph();

% Add nodes (bodies)
bodyNames = arrayfun(@getfullname, bodyBlocks, 'UniformOutput', false);
G = addnode(G, bodyNames);

% Add edges for joints and rigid transforms
for i = 1:numel(jointBlocks)
    joint = jointBlocks(i);
    jointConn = get_param(joint, 'PortConnectivity');
    
    if numel(jointConn) >= 2
        % Get full path names of connected bodies
        for j=1:numel(jointConn(1).DstBlock)
            for k=1:numel(jointConn(2).DstBlock)
                body1 = getfullname(jointConn(1).DstBlock(j));
                body2 = getfullname(jointConn(2).DstBlock(k));

                % Add edge to graph (undirected for rigid connections)
                G = addedge(G, body1, body2);
            end
        end
    end
end

for i = 1:numel(rigidTransformBlocks)
    rigidTransform = rigidTransformBlocks(i);
    rigidConn = get_param(rigidTransform, 'PortConnectivity');
    
    if numel(rigidConn) >= 2
        % Get full path names of connected bodies
        for j=1:numel(rigidConn(1).DstBlock)
            for k=1:numel(rigidConn(2).DstBlock)
                body1 = getfullname(rigidConn(1).DstBlock(j));
                body2 = getfullname(rigidConn(2).DstBlock(k));

                % Add edge to graph (undirected for rigid connections)
                G = addedge(G, body1, body2);
            end
        end
    end
end

%% Step 4: Identify Rigid Assemblies Using Connected Components
% Find connected components in the graph
componentIndices = conncomp(G);

% Group bodies by their component index
rigidAssemblies = containers.Map('KeyType', 'int32', 'ValueType', 'any');
for i = 1:numel(bodyNames)
    componentIndex = componentIndices(i);
    if isKey(rigidAssemblies, componentIndex)
        rigidAssemblies(componentIndex) = [rigidAssemblies(componentIndex), bodyNames{i}];
    else
        rigidAssemblies(componentIndex) = {bodyNames{i}};
    end
end

% Display the rigid assemblies
fprintf('Rigid Assemblies in the Model:\n');
for key = keys(rigidAssemblies)
    fprintf('Assembly %d: ', key{1});
    disp(rigidAssemblies(key{1}));
end

% allKeys = keys(connectivityGraph);
% for i=1:numel(allKeys)
%     key = allKeys{i};        % Get the current key
%     value = connectivityGraph(key);      % Get the value associated with the key
% 
%     % Display key and value
%     fprintf('%s <-> ', key);
%     disp(value);
% end