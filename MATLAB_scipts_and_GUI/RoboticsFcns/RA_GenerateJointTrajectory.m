function [q, qd, qdd, pp] = RA_GenerateJointTrajectory(robot, wpts, tpts, initialGuess)
    if size(wpts, 1) ~= 3
        error('wpts must have 3 rows (X, Y, Z coordinates)');
    end
    
    if size(wpts, 2) ~= length(tpts)
        error('Number of waypoints in wpts does not match number of time points in tpts');
    end
    dt = 0.5;
    tvec = tpts(1):dt:tpts(end);
    [q_xyz, ~, ~, ~] = quinticpolytraj(wpts, tpts, tvec);
    indices = round((tpts - tvec(1)) / dt) + 1;
    
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    joint_des = zeros(numJoints, length(tvec));
    guess = initialGuess;
    
    for i = 1:length(tvec)
        [joint_des(:, i), ikInfo] = RA_InverseKinematics(robot, q_xyz(:, i)', guess');
        guess = joint_des(:, i);
        
        if ~ikInfo.ExitFlag
            warning('IK failed to converge for waypoint %d. Error: %s', i, ikInfo.Status);
        end
    end
    
    jointWpts = joint_des(:, indices);
    tvec = tpts(1):0.01:tpts(end);
    [q, qd, qdd, pp] = quinticpolytraj(jointWpts, tpts, tvec);
end



