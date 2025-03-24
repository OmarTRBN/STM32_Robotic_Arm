function [workspacePoints, configs] = RA_VisualizeWorkspace(robot, maxSamples, plotFlag)
    % RA_VISUALIZEWORKSPACE Visualize and store accessible workspace of robot
    %   [workspacePoints, configs] = RA_VisualizeWorkspace(robot, maxSamples, plotFlag)
    %   generates and optionally visualizes the reachable workspace of the robot
    %
    %   Inputs:
    %       robot - rigidBodyTree object
    %       maxSamples - (optional) Maximum number of joint configurations to sample (default: 5000)
    %       plotFlag - (optional) Boolean flag to control plotting (default: true)
    %
    %   Outputs:
    %       workspacePoints - N×3 matrix of reachable [X,Y,Z] coordinates
    %       configs - N×M matrix of joint configurations corresponding to each point
    
    % Set default values for optional inputs
    if nargin < 2 || isempty(maxSamples)
        maxSamples = 5000;
    end
    
    if nargin < 3 || isempty(plotFlag)
        plotFlag = true;
    end
    
    % Get end effector name (last body in the tree)
    endEffectorName = robot.BodyNames{end};
    
    % Define empty environment (no collision objects)
    environment = {};
    
    % Generate the workspace
    try
        % Using the generateRobotWorkspace function (available in R2024b and later)
        [workspacePoints, configs] = generateRobotWorkspace(robot, environment, ...
            endEffectorName, 'MaxNumSamples', maxSamples, ...
            'IgnoreSelfCollision', 'on');
    catch ME
        % If the function is not available (earlier MATLAB versions),
        % use a custom approach to generate the workspace
        warning(['generateRobotWorkspace function not available. ', ...
                'Using custom workspace generation method.']);
        
        [workspacePoints, configs] = customWorkspaceGeneration(robot, maxSamples);
    end
    
    % Visualize the workspace if requested
    if plotFlag && ~isempty(workspacePoints)
        figure('Name', 'Robot Workspace');
        
        % Plot the robot at home configuration
        homeConfig = homeConfiguration(robot);
        subplot(1, 2, 1);
        show(robot, homeConfig, 'PreservePlot', false);
        title('Robot Model');
        
        % Plot the workspace points
        subplot(1, 2, 2);
        scatter3(workspacePoints(:,1), workspacePoints(:,2), workspacePoints(:,3), 5, 'filled');
        title('Robot Workspace');
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        grid on;
        axis equal;
        colormap jet;
        
        % Convex hull of workspace (optional)
        hold on;
        try
            % Try to create a convex hull of the workspace points
            K = convhull(workspacePoints(:,1), workspacePoints(:,2), workspacePoints(:,3));
            trisurf(K, workspacePoints(:,1), workspacePoints(:,2), workspacePoints(:,3), ...
                'FaceColor', 'cyan', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
        catch
            % Skip convex hull if it fails (can happen with certain point distributions)
        end
        
        % Add colorbar for visualization of density
        colorbar;
        
        % Workspace statistics
        mins = min(workspacePoints, [], 1);
        maxs = max(workspacePoints, [], 1);
        range = maxs - mins;
        volume = prod(range);
        
        disp('Workspace Statistics:');
        fprintf('  Number of points: %d\n', size(workspacePoints, 1));
        fprintf('  X range: [%.3f, %.3f] m\n', mins(1), maxs(1));
        fprintf('  Y range: [%.3f, %.3f] m\n', mins(2), maxs(2));
        fprintf('  Z range: [%.3f, %.3f] m\n', mins(3), maxs(3));
        fprintf('  Bounding box volume: %.3f m³\n', volume);
    end
end

function [workspacePoints, configs] = customWorkspaceGeneration(robot, maxSamples)
    % Custom function to generate workspace points when generateRobotWorkspace is unavailable
    
    % Get number of DoF
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    % Get end effector name
    endEffectorName = robot.BodyNames{end};
    
    % Pre-allocate arrays
    configs = zeros(maxSamples, numJoints);
    workspacePoints = zeros(maxSamples, 3);
    validPointCount = 0;
    
    % Get joint limits
    jointLimits = getJointLimits(robot);
    
    % Sample random configurations within joint limits
    for i = 1:maxSamples
        % Generate random joint configuration within limits
        randomConfig = homeConfiguration(robot);
        for j = 1:numJoints
            minVal = jointLimits(j, 1);
            maxVal = jointLimits(j, 2);
            if minVal ~= maxVal  % Skip fixed joints
                randomConfig(j).JointPosition = minVal + (maxVal - minVal) * rand();
            end
        end
        
        % Get end-effector position for this configuration
        try
            transform = getTransform(robot, randomConfig, endEffectorName);
            position = transform(1:3, 4);
            
            % Store the position and configuration
            validPointCount = validPointCount + 1;
            workspacePoints(validPointCount, :) = position';
            
            % Store joint positions
            for j = 1:numJoints
                configs(validPointCount, j) = randomConfig(j).JointPosition;
            end
        catch
            % Skip invalid configurations
            continue;
        end
        
        % Display progress
        if mod(i, floor(maxSamples/10)) == 0
            fprintf('Workspace generation: %.0f%%\n', 100*i/maxSamples);
        end
    end
    
    % Trim arrays to actual size
    workspacePoints = workspacePoints(1:validPointCount, :);
    configs = configs(1:validPointCount, :);
end

function jointLimits = getJointLimits(robot)
    % Extract joint limits from robot
    numJoints = length(homeConfiguration(robot));
    jointLimits = zeros(numJoints, 2);
    
    % Default limits for revolute joints if not specified
    defaultLimits = [-pi, pi];
    
    % Loop through each joint
    for i = 1:numJoints
        % Get joint information
        jointInfo = robot.Bodies{i}.Joint;
        
        if isa(jointInfo, 'robotics.core.RevoluteJoint')
            % Check if joint has specified limits
            if ~isempty(jointInfo.PositionLimits)
                jointLimits(i, :) = jointInfo.PositionLimits;
            else
                jointLimits(i, :) = defaultLimits;
            end
        else
            % Fixed joint
            jointLimits(i, :) = [0, 0];
        end
    end
end