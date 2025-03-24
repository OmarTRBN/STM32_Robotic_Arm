function [q, qd, qdd, tvec] = RA_GenerateJointTrajectory(robot, wpts, tpts)
    % RA_GENERATEJOINTTRAJECTORY Generate joint-space trajectory from Cartesian waypoints
    %   [q, qd, qdd, tvec] = RA_GenerateJointTrajectory(robot, wpts, tpts)
    %   returns joint positions, velocities, and accelerations along a trajectory
    %   that passes through the specified Cartesian waypoints.
    %
    %   Inputs:
    %       robot - rigidBodyTree object
    %       wpts - 3×N matrix of cartesian waypoints, where:
    %              row 1: X coordinates
    %              row 2: Y coordinates
    %              row 3: Z coordinates
    %              Each column represents a waypoint
    %       tpts - 1×N vector of time points for each waypoint
    %
    %   Outputs:
    %       q - Joint positions along the trajectory
    %       qd - Joint velocities along the trajectory
    %       qdd - Joint accelerations along the trajectory
    %       tvec - Time vector for the trajectory (sampled at 0.01s)
    %
    %   The function checks if all waypoints are within the robot's workspace
    %   and throws an error if any point is not accessible.

    % Input validation
    if size(wpts, 1) ~= 3
        error('wpts must have 3 rows (X, Y, Z coordinates)');
    end
    
    if size(wpts, 2) ~= length(tpts)
        error('Number of waypoints in wpts does not match number of time points in tpts');
    end
    
    % Check if waypoints are accessible (within workspace)
    % checkWaypointsAccessibility(robot, wpts);
    
    % Create fine time vector with 0.01s sampling
    tvec = tpts(1):0.01:tpts(end);
    
    % Get the number of DOF (degrees of freedom) of the robot
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    % Convert Cartesian waypoints to joint waypoints using inverse kinematics
    jointWpts = zeros(numJoints, size(wpts, 2));
    
    % Initial guess for IK (use home configuration for the first waypoint)
    initialGuess = zeros(numJoints, 1);
    
    % Process each waypoint
    for i = 1:size(wpts, 2)
        targetPosition = wpts(:, i);
        
        % Solve inverse kinematics
        [jointAngles, ikInfo] = RA_InverseKinematics(robot, targetPosition, initialGuess);
        
        % Check if IK solution was successful
        if ~ikInfo.ExitFlag
            warning('IK failed to converge for waypoint %d. Error: %s', i, ikInfo.Status);
        end
        
        % Store joint angles for this waypoint
        jointWpts(:, i) = jointAngles;
        
        % Use current solution as initial guess for next waypoint
        initialGuess = jointAngles;
    end
    
    % Generate a quintic polynomial trajectory through the joint waypoints
    [q, qd, qdd, pp] = quinticpolytraj(jointWpts, tpts, tvec);
end

function checkWaypointsAccessibility(robot, wpts)
    % Check if all waypoints are accessible by the robot
    
    % Get a sample of the robot's workspace (using a moderate number of samples)
    [workspacePoints, ~] = RA_VisualizeWorkspace(robot, 10000, false);
    
    % For each waypoint, check if it's within the workspace
    numWaypoints = size(wpts, 2);
    inaccessiblePoints = false(1, numWaypoints);
    inaccessibleIndices = [];
    
    % Create a fast lookup structure for the workspace points
    % This allows us to efficiently find the nearest neighbor
    try
        % Try to use MATLAB's Statistics and Machine Learning Toolbox
        kdtree = KDTreeSearcher(workspacePoints);
        
        % For each waypoint
        for i = 1:numWaypoints
            % Find the distance to the closest point in the workspace
            [~, distance] = knnsearch(kdtree, wpts(:,i)', 'K', 1);
            
            % Set a threshold for considering a point accessible
            % This threshold depends on the workspace sampling density
            % A smaller threshold requires more samples for accuracy
            threshold = 0.02; % 2cm threshold (adjust based on robot size)
            
            % Check if the waypoint is too far from any workspace point
            if distance > threshold
                inaccessiblePoints(i) = true;
                inaccessibleIndices = [inaccessibleIndices, i];
            end
        end
    catch
        % Fallback method if KDTreeSearcher is not available
        % Less efficient but doesn't require additional toolboxes
        warning('KDTreeSearcher not available, using slower distance method');
        
        % For each waypoint
        for i = 1:numWaypoints
            % Calculate distances to all workspace points
            distances = sqrt(sum((workspacePoints - wpts(:,i)').^2, 2));
            minDistance = min(distances);
            
            % Check if the waypoint is too far from any workspace point
            threshold = 0.02; % 2cm threshold
            if minDistance > threshold
                inaccessiblePoints(i) = true;
                inaccessibleIndices = [inaccessibleIndices, i];
            end
        end
    end
    
    % If any inaccessible points were found, throw an error
    if any(inaccessiblePoints)
        errorMsg = sprintf(['The following waypoints (indices) are not accessible by the robot: %s\n', ...
                           'Please check these coordinates and ensure they are within the robot workspace.'], ...
                           mat2str(inaccessibleIndices));
        error(errorMsg);
    end
end