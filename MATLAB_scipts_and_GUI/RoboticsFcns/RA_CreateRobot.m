function robot = RA_CreateRobot(plotRobot, showDetails)
    robot = rigidBodyTree('DataFormat', 'column');
    
    % a: link length
    % alpha: link twist
    % d: link offset
    % theta: joint angle (variable for revolute joints)
    % Define DH parameters [a, alpha, d, theta]
    dhParams = [
                0  	 0 0.2  0;
                0.5  0  0	0;
                0.7  pi 0	0;
               ];
    
    jointLimits = [
        -pi/2, pi/2;   % Joint 1 limits
        -pi/2, pi/2;   % Joint 2 limits
    ];

    % Automatically generate names based on DH table size
    linkNames = cell(1, size(dhParams, 1));
    jointNames = cell(1, size(dhParams, 1));
    parentNames = cell(1, size(dhParams, 1));
    parentNames{1} = 'base';
    
    for i = 1:size(dhParams, 1)
        if i==size(dhParams, 1)
            linkNames{i} = 'tool';
            jointNames{i} = 'fix';
        else
            linkNames{i} = sprintf('link%d', i);
            jointNames{i} = sprintf('joint%d', i);
        end
        
        % Set parent links (except for the first one which is already set)
        if i > 1
            parentNames{i} = linkNames{i-1};
        end
    end
    % jointTypes = {'revolute', 'revolute', 'fixed'};
    % linkNames = {'link1', 'link2', 'tool'};
    % jointNames = {'joint1', 'joint2', 'fix1'};
    % parentNames = {'base', 'link1', 'link2'};
    
    for i = 1:size(dhParams, 1)
        body = rigidBody(linkNames{i});
        
        % Create a joint of the specified type
        if i<size(dhParams, 1)
            joint = rigidBodyJoint(jointNames{i}, 'revolute');
            joint.JointAxis = [0 0 1];

            joint.PositionLimits = jointLimits(i, :);
        else
            joint = rigidBodyJoint(jointNames{i}, 'fixed');
        end
        
        % Use DH parameters to set transformation
        a = dhParams(i, 1);
        alpha = dhParams(i, 2);
        d = dhParams(i, 3);
        theta = dhParams(i, 4);
        
        T = dhMatrix(a, alpha, d, theta);
        setFixedTransform(joint, T);
        
        body.Joint = joint;
        addBody(robot, body, parentNames{i});
    end
    
    if showDetails
        showdetails(robot);
    end

    if plotRobot
        figure;
        show(robot);
        title('Robot Model');
        grid on;
    end
    
end

% Helper function to create DH transformation matrix
function T = dhMatrix(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end