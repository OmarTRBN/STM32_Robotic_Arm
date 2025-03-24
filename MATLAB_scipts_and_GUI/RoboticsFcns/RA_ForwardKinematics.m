function endTransform = RA_ForwardKinematics(robot, jointAngles, plotRobot)
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    if length(jointAngles) ~= numJoints
        error('Number of provided angles (%d) does not match robot DOF (%d)', ...
              length(jointAngles), numJoints);
    end
    
    % Create a configuration structure with the provided angles
    for i = 1:numJoints
        % Get joint limits for the current joint
        joint = robot.Bodies{i}.Joint;
        
        % Only check limits for revolute joints
        if strcmp(joint.Type, 'revolute')
            limits = joint.PositionLimits;
            
            % Check if the angle is outside limits
            if jointAngles(i) < limits(1) || jointAngles(i) > limits(2)
                % Clamp the angle to the limits
                clampedAngle = max(limits(1), min(jointAngles(i), limits(2)));
                
                % Print message about clamping
                fprintf('Joint %d angle (%.2f) outside limits [%.2f, %.2f]. Clamped to %.2f\n', ...
                    i, jointAngles(i), limits(1), limits(2), clampedAngle);
                
                % Update the angle
                jointAngles(i) = clampedAngle;
            end
        end
        
        config(i) = jointAngles(i);
    end
    
    endEffectorName = robot.BodyNames{end};
    endTransform = getTransform(robot, config, endEffectorName);
        
    if plotRobot
        figure;
        show(robot, config);
        title('Robot Model');
        grid on;
    end
end