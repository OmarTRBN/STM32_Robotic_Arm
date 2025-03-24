function endTransform = RA_ForwardKinematics(robot, jointAngles)
    % FORWARDKINEMATICS Compute forward kinematics for a robot
    %   endPosition = forwardKinematics(robot, jointAngles) returns the XYZ position
    %   of the robot's end-effector given the robot model and joint angles
    %
    %   Inputs:
    %       robot - rigidBodyTree object
    %       jointAngles - vector of joint angles (in radians)
    %
    %   Outputs:
    %       endPosition - 3x1 vector [x; y; z] of end-effector position
    
    % Get the number of non-fixed joints in the robot
    config = homeConfiguration(robot);
    numJoints = length(config);
    
    if length(jointAngles) ~= numJoints
        error('Number of provided angles (%d) does not match robot DOF (%d)', ...
              length(jointAngles), numJoints);
    end
    
    % Create a configuration structure with the provided angles
    for i = 1:numJoints
        config(i) = jointAngles(i);
    end
    
    endEffectorName = robot.BodyNames{end};
    endTransform = getTransform(robot, config, endEffectorName);
        
    % show(robot, config);
end