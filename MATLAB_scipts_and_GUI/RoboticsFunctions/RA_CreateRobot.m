function robot = RA_CreateRobot()
    % Create a rigid body tree with column data format
    robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 3);
    
    % Define DH parameters [a, alpha, d, theta]
    % a: link length
    % alpha: link twist
    % d: link offset
    % theta: joint angle (variable for revolute joints)
    dhParams = [
                0  	 0  0.2  0;
                0.5  0  0	0;
                0.7  pi 0	0;
               ];
    
    % Joint types: 'revolute' or 'fixed'
    jointTypes = {'revolute', 'revolute', 'fixed'};
    
    % Names for links and joints
    linkNames = {'link1', 'link2', 'tool'};
    jointNames = {'joint1', 'joint2', 'fix1'};
    
    % Parent links (base is parent of link1, etc.)
    parentNames = {'base', 'link1', 'link2'};
    
    % Create the robot structure using a loop
    for i = 1:length(jointTypes)
        % Create a rigid body
        body = rigidBody(linkNames{i});
        
        % Create a joint of the specified type
        joint = rigidBodyJoint(jointNames{i}, jointTypes{i});
        
        % Set joint axis (Z-axis for revolute joints)
        if strcmp(jointTypes{i}, 'revolute')
            joint.JointAxis = [0 0 1];
        end
        
        % Use DH parameters to set transformation
        a = dhParams(i, 1);
        alpha = dhParams(i, 2);
        d = dhParams(i, 3);
        theta = dhParams(i, 4);
        
        % Create transformation matrix using DH parameters
        T = dhMatrix(a, alpha, d, theta);
        setFixedTransform(joint, T);
        
        % Assign joint to body
        body.Joint = joint;
        
        % Add body to the robot
        addBody(robot, body, parentNames{i});
    end
    
    % Display robot details
    % showdetails(robot);
end

% Helper function to create DH transformation matrix
function T = dhMatrix(a, alpha, d, theta)
    % Standard DH transformation matrix
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end