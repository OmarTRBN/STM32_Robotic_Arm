function RA_AnimateRobotTrajectory(robot, q)
    figure(3);
    title('Robot Motion Animation');

    numPoints = size(q, 2);
    for i = 1:10:numPoints
        % Show robot at different configurations along the trajectory
        config = homeConfiguration(robot);
        for j = 1:length(config)
            config(j) = q(j, i);
        end
        show(robot, config, 'PreservePlot', false);
        drawnow;
        pause(0.1);
    end
end
