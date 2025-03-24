function [jointAngles, solInfo] = RA_InverseKinematics(robot, targetPosition, initialGuess)
    % INVERSEKINEMATICS Solve inverse kinematics using numerical methods
    %   [jointAngles, success] = inverseKinematics(robot, targetPosition, initialGuess)
    %   returns joint angles that position the end-effector at the target position
    %
    %   Inputs:
    %       robot - rigidBodyTree object
    %       targetPosition - 3x1 vector [x; y; z] of desired end-effector position
    %       initialGuess - (optional) initial guess for joint angles
    %
    %   Outputs:
    %       jointAngles - vector of joint angles (in radians)
    %       success - boolean indicating whether the algorithm converged
    
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    endEffectorName = robot.BodyNames{end};
    
    ik = inverseKinematics('RigidBodyTree', robot);
    
    % Set weight matrix: prioritize position over orientation
    weights = [1 1 1 1 1 1];  % [x y z roll pitch yaw]
    
    for i = 1:numJoints
        config(i) = initialGuess(i);
    end
    
    % Create the target transformation (position only, ignore orientation)
    targetTransform = eye(4);
    targetTransform(1:3, 4) = targetPosition;
    
    [configSolution, solInfo] = ik(endEffectorName, targetTransform, weights, config);
    
    % Extract joint angles from the solution
    jointAngles = zeros(numJoints, 1);
    for i = 1:numJoints
        jointAngles(i) = configSolution(i);
    end
end