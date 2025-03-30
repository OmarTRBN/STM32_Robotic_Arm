function RA_PlotTrajectory(robot, q, T, wpts, tpts, appFig)
    % Check if appFig parameter was provided
    useAppFigure = exist('appFig', 'var') && ~isempty(appFig);
    
    actual_xyz = zeros(3, size(q,2));
    for i = 1:size(q,2)
        jointAngles = q(:, i)';
        % Compute forward kinematics
        Ti = RA_ForwardKinematics(robot, jointAngles, false);
        actual_xyz(:, i) = Ti(1:3, 4);
    end
    
    % Apply threshold to ignore values below 0.0001
    threshold = 0.0001;
    actual_xyz(abs(actual_xyz) < threshold) = 0;  % Set values below the threshold to 0
    
    if ~useAppFigure
        % Create a new figure if not using an app figure
        figure('Name', 'Trajectory with Waypoints');
    end
    
    labels = {'X', 'Y', 'Z'};
    
    for dim = 1:3
        if useAppFigure
            % Use the provided app axes instead of creating new subplots
            axes(appFig(dim));
            cla(appFig(dim)); % Clear the axes
        else
            subplot(3,1,dim);
        end

        plot(T, actual_xyz(dim,:), 'b-', 'LineWidth', 1.5);
        hold on;
        
        % Place waypoints
        for i = 1:size(wpts,2)
            waypointIndex = find(T >= tpts(i), 1);
            plot(T(waypointIndex), wpts(dim,i), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        end
        
        title([labels{dim} ' Position']);
        ylabel([labels{dim} ' (m)']);
        grid on;
        hold off;
        
        if dim == 3 || useAppFigure
            xlabel('Time (s)');
        end
    end

    legend('Trajectory','Waypoints','Location','northeast');
end